#include "stabilizer.hpp"


Stabilizer::Stabilizer(const cv::Mat& frame0) {
    cv::cvtColor(frame0, frame_gray_0_, CV_BGR2GRAY);
    cv::goodFeaturesToTrack(frame_gray_0_, points0_, 1000, .01, 8);
    points_ = points0_;
    status_.resize(points0_.size());
    trust_.resize(points0_.size(), 1.);
    last_frame_gray_ = frame_gray_0_;
    H_ = cv::Mat::eye(3, 3, CV_64F);
}

cv::Mat Stabilizer::stabilize_next(const cv::Mat& frame) {
    cv::Mat frame_gray, H_frame_gray;
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);

    cv::warpPerspective(frame_gray, H_frame_gray, H_, cv::Size(frame_gray.cols, frame_gray.rows));

    std::vector<cv::Point2f> new_points;

    new_points = checked_optical_flow_(H_frame_gray, 1.);

    std::vector<cv::Point2f> good_points0;
    std::vector<cv::Point2f> good_new_points;

    good_points0.reserve(new_points.size());
    good_new_points.reserve(new_points.size());
    for (std::size_t i=0; i<new_points.size(); ++i)
        if (status_[i] && trust_[i] >= .5) {
            good_points0.push_back(points0_[i]);
            good_new_points.push_back(new_points[i]);
         }

    cv::Mat dH = cv::findHomography(good_new_points, good_points0);;

    H_ = dH * H_;

    cv::Mat stabilized_frame;

    //cv::perspectiveTransform(new_points)
    cv::warpPerspective(frame, stabilized_frame, H_, cv::Size(frame.cols, frame.rows));
    cv::perspectiveTransform(points0_, points_, H_.inv());
    last_frame_gray_ = frame_gray;
    ++count_;
    return stabilized_frame;
}

std::vector<cv::Point2f> Stabilizer::checked_optical_flow_(const cv::Mat& frame_gray, float eps) {
    cv::Mat err;
    cv::Mat status1, status2;
    std::vector<cv::Point2f> points1, points2;
    points1.reserve(points0_.size());
    points2.reserve(points0_.size());

    cv::calcOpticalFlowPyrLK(frame_gray_0_, frame_gray, points0_, points1, status1, err);
    cv::calcOpticalFlowPyrLK(frame_gray, frame_gray_0_, points1, points2, status2, err);

    for (std::size_t i=0; i<points_.size(); ++i) {
        status_[i] = (status1.at<bool>(i) && status2.at<bool>(i) && cv::norm(points0_[i] - points2[i]) < eps );
        float new_trust = status_[i] ? 1.f : 0.f;
        trust_[i] = (count_*trust_[i] + new_trust)/(count_+1);
        //trust_[i] = .99 * trust_[i] + .01 * new_trust;
    }

    return points1;
}

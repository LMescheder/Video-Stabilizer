#include "stabilizer.hpp"


Stabilizer::Stabilizer(const cv::Mat& frame0) {
    cv::cvtColor(frame0, frame_gray_0_, CV_BGR2GRAY);
    cv::goodFeaturesToTrack(frame_gray_0_, points0_, 1000, .01, 8);
    points_ = points0_;
    last_frame_gray_ = frame_gray_0_;
}

cv::Mat Stabilizer::stabilize_next(const cv::Mat& frame) {
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);

    std::vector<cv::Point2f> new_points;
    std::vector<bool> status;

    std::tie(new_points, status) = checked_optical_flow_(frame_gray, points_, 1.);

    std::vector<cv::Point2f> points0;

    points0.reserve(new_points.size());
    points_.clear();
    for (std::size_t i=0; i<new_points.size(); ++i)
        if (status[i]) {
            points0.push_back(points0_[i]);
            points_.push_back(new_points[i]);
         }
    points0_ = points0;

    cv::Mat H = cv::findHomography(points_, points0_);;

    cv::Mat stabilized_frame;

    cv::warpPerspective(frame, stabilized_frame, H, cv::Size(frame.cols, frame.rows));
    last_frame_gray_ = frame_gray;

    return stabilized_frame;
}

std::tuple<std::vector<cv::Point2f>, std::vector<bool> > Stabilizer::checked_optical_flow_(const cv::Mat& frame_gray, const std::vector<cv::Point2f>& points, float eps) {
    cv::Mat err;
    cv::Mat status1, status2;
    std::vector<cv::Point2f> points1, points2;
    std::vector<bool> status;
    points1.reserve(points.size());
    points2.reserve(points.size());
    status.resize(points.size(), true);

    cv::calcOpticalFlowPyrLK(last_frame_gray_, frame_gray, points, points1, status1, err);
    cv::calcOpticalFlowPyrLK(frame_gray, last_frame_gray_, points1, points2, status2, err);

    for (std::size_t i=0; i<points.size(); ++i) {
        status[i] = (status1.at<bool>(i) && status2.at<bool>(i) && cv::norm(points[i] - points2[i]) < eps );
    }

    return std::make_tuple(points1, status);
}

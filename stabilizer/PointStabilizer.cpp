#include "PointStabilizer.hpp"


cv::Mat PointStabilizer::stabilize_next(const cv::Mat& next_frame) {
    cv::Mat frame_gray, H_frame_gray;
    cv::cvtColor(next_frame, frame_gray, CV_BGR2GRAY);


    cv::warpPerspective(frame_gray, H_frame_gray, H_, cv::Size(frame_gray.cols, frame_gray.rows));

    cv::Mat dH = get_next_homography_(H_frame_gray);

    visualization_ = next_frame.clone();

    H_ = dH * H_;

    create_visualization_();

    cv::Mat stabilized_frame;
    cv::warpPerspective(next_frame, stabilized_frame, H_, cv::Size(next_frame.cols, next_frame.rows));

    return stabilized_frame;
}

cv::Mat PointStabilizer::visualization() const {
    return visualization_;
}



PointStabilizer::PointStabilizer(const cv::Mat& frame0, WarpingGroup mode) {
    cv::cvtColor(frame0, frame_gray_0_, CV_BGR2GRAY);
    cv::goodFeaturesToTrack(frame_gray_0_, points0_, 1000, .01, 8);
    points_ = points0_;
    status_.resize(points0_.size());
    trust_.resize(points0_.size(), .5);
    last_frame_gray_ = frame_gray_0_;
    H_ = cv::Mat::eye(3, 3, CV_64F);
}

cv::Mat PointStabilizer::get_next_homography_(const cv::Mat &next_image)
{
    std::vector<cv::Point2f> new_points;

    new_points = checked_optical_flow_(next_image, .1);

    std::vector<cv::Point2f> good_points0;
    std::vector<cv::Point2f> good_new_points;

    good_points0.reserve(new_points.size());
    good_new_points.reserve(new_points.size());
    for (std::size_t i=0; i<new_points.size(); ++i)
        if (status_[i] && trust_[i] >= .5) {
            good_points0.push_back(points0_[i]);
            good_new_points.push_back(new_points[i]);
         }

    return find_homography(good_new_points, good_points0, mode_);
}

void PointStabilizer::create_visualization_() {
    std::vector<cv::Point2f> new_points_vis;
    cv::perspectiveTransform(points0_, new_points_vis, H_.inv());
    for (std::size_t i=0; i<points0_.size(); ++i) {
        cv::Scalar color;
        if (trust_[i] < .5)
            color = cv::Scalar(0, 0, 255);
        else if (status_[i])
            color = cv::Scalar(0, 255, 0);
        else
            color = cv::Scalar(0, 255, 255);

        cv::circle(visualization_, new_points_vis[i], 2, color);
        cv::line(visualization_,  points0_[i],  new_points_vis[i], color);
    }
}

std::vector<cv::Point2f> PointStabilizer::checked_optical_flow_(const cv::Mat& frame_gray, float eps) {
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
        //trust_[i] = (count_*trust_[i] + new_trust)/(count_+1);
        trust_[i] = .95 * trust_[i] + .05 * new_trust;
    }

    return points1;
}

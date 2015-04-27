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
    new_points.reserve(points_.size());

    cv::calcOpticalFlowPyrLK(last_frame_gray_, frame_gray, points_, new_points, status_, error_);

    std::vector<cv::Point2f> points0;

    points0.reserve(new_points.size());
    points_.clear();
    for (std::size_t i=0; i<new_points.size(); ++i)
        if (status_.at<bool>(i)) {
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

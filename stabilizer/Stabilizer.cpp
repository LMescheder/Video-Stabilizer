#include "Stabilizer.hpp"

cv::Mat Stabilizer::stabilize_next(const cv::Mat& next_frame) {
    cv::Mat frame_gray, H_frame_gray;
    cv::cvtColor(next_frame, frame_gray, CV_BGR2GRAY);

    cv::warpPerspective(frame_gray, H_frame_gray, H_, cv::Size(frame_gray.cols, frame_gray.rows));

    cv::Mat dH = get_next_homography_(H_frame_gray);

    H_ = dH * H_;

    cv::Mat stabilized_frame;
    cv::warpPerspective(next_frame, stabilized_frame, H_, cv::Size(next_frame.cols, next_frame.rows));

    return stabilized_frame;
}

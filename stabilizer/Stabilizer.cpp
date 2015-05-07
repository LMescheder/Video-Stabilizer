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

cv::Mat Stabilizer::find_homography_(const cv::vector<cv::Point2f>& points0, const cv::vector<cv::Point2f>& points1) {
    cv::Mat H, A;

    switch (mode_) {
    case homography :
        H = cv::findHomography(points0, points1);
        break;
    case affine :
        A = cv::estimateRigidTransform(points0, points1, true);
        H = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H(cv::Range(0, 2), cv::Range(0, 3)));
        break;
    case rigid :
        A = cv::estimateRigidTransform(points0, points1, false);
        H = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H(cv::Range(0, 2), cv::Range(0, 3)));
        break;
    }
    return H;
}

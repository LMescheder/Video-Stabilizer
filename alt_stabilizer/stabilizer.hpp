#ifndef STABILIZER_HPP
#define STABILIZER_HPP

#include <vector>
#include "opencv2/opencv.hpp"

class Stabilizer {
public:
    Stabilizer(const cv::Mat& frame0);

    cv::Mat stabilize_next(const cv::Mat& frame);

    const std::vector<cv::Point2f>& points0() const {
        return points0_;
    }

    const std::vector<cv::Point2f>& points() const {
        return points_;
    }

    const std::vector<bool>& status() const {
        return status_;
    }

    const std::vector<float>& trust() const {
        return trust_;
    }

private:
    cv::Mat frame_gray_0_;
    cv::Mat last_frame_gray_;

    cv::Mat H_;

    std::vector<cv::Point2f> points0_;
    std::vector<cv::Point2f> points_;
    std::vector<bool> status_;
    cv::Mat error_;

    unsigned long count_ = 1;
    std::vector<float> trust_;

    std::vector<cv::Point2f> checked_optical_flow_(const cv::Mat& frame_gray, float eps);
};

#endif // STABILIZER_HPP

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

    const cv::Mat& status() const {
        return status_;
    }

private:
    cv::Mat frame_gray_0_;
    cv::Mat last_frame_gray_;

    std::vector<cv::Point2f> points0_;
    std::vector<cv::Point2f> points_;
    cv::Mat status_;
    cv::Mat error_;

    std::tuple<std::vector<cv::Point2f>, std::vector<bool> > checked_optical_flow_(const cv::Mat& frame_gray, const std::vector<cv::Point2f>& points, float eps);
};

#endif // STABILIZER_HPP

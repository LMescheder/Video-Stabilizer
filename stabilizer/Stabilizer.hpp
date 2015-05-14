#ifndef STABILIZER_HPP
#define STABILIZER_HPP


#include "opencv2/opencv.hpp"


/**
 * @brief Abstract base class for video stabilization.
 */

class Stabilizer
{
public:
    enum Mode {
        homography,
        affine,
        rigid
    };

    Stabilizer (const cv::Mat& frame_0, Mode mode)
        : mode_{mode} {
        H_ = cv::Mat::eye(3, 3, CV_64FC1);
        cv::cvtColor(frame_0, frame_gray_0_, CV_BGR2GRAY);
    }

    virtual cv::Mat stabilize_next(const cv::Mat& next_frame);

    virtual Mode mode() const {
        return mode_;
    }

    virtual void set_mode(const Mode &mode) {
        mode_ = mode;
    }

    virtual cv::Mat visualization() const {
        return visualization_;
    }

protected:
    virtual cv::Mat get_next_homography_(const cv::Mat& next_image) = 0;
    virtual cv::Mat find_homography_(const cv::vector<cv::Point2f>& points0, const cv::vector<cv::Point2f>& points1);

    cv::Mat H_;
    cv::Mat frame_gray_0_;
    cv::Mat visualization_;
    Mode mode_ = homography;

};

#endif // STABILIZER_HPP

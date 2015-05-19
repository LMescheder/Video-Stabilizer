#ifndef STABILIZER_HPP
#define STABILIZER_HPP


#include "opencv2/opencv.hpp"
#include "utilities.h"

/**
 * @brief Interface for video stabilization.
 */

class Stabilizer
{
public:

    Stabilizer (const cv::Mat& frame_0, WarpingGroup warping=WarpingGroup::homography, bool warping_back=true, bool visualize=true)
        : warping_{warping}, warping_back_{warping_back}, visualize_{visualize} {
        H_ = cv::Mat::eye(3, 3, CV_64FC1);
        cv::cvtColor(frame_0, frame_gray_0_, CV_BGR2GRAY);
    }

    virtual cv::Mat stabilize_next(const cv::Mat& next_frame);
    virtual cv::Mat visualization() const = 0;

protected:
    virtual cv::Mat get_next_homography(const cv::Mat& next_frame) = 0;
    virtual void create_visualization(const cv::Mat& next_frame) = 0;

protected:
    // Parameters
    WarpingGroup warping_ = WarpingGroup::homography;
    bool visualize_ = true;
    bool warping_back_;

    cv::Mat H_;
    cv::Mat frame_gray_0_;
    cv::Mat frame;
    cv::Mat visualization_;
    cv::Mat frame_gray_, H_frame_gray_;
};

#endif // STABILIZER_HPP

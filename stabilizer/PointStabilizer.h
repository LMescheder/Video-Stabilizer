#ifndef PointStabilizer_HPP
#define PointStabilizer_HPP

#include <vector>
#include "opencv2/opencv.hpp"
#include "Stabilizer.h"

/**
 * @brief The PointStabilizer class implements a basic video stabilizer based on key point tracking.
 */
class PointStabilizer : public Stabilizer{

public:
    PointStabilizer(const cv::Mat& frame_0, Warping warping=Warping::HOMOGRAPHY, Mode mode=Mode::DIRECT);

protected:
    virtual void track_ref();
    virtual cv::Mat get_next_homography (const cv::Mat& next_image);
    virtual void create_visualization ();

private:
    std::vector<cv::Point2f> checked_optical_flow_(const cv::Mat& frame_gray, float eps);

private:
    // Parameters
    double max_flow_err = 1e-1;
    int features_maxN_ = 2000;
    double features_quality_ = 1e-2;
    int features_mindist_ = 4;

    // Status
    cv::Mat last_frame_gray_;

    std::vector<cv::Point2f> points_0_;
    std::vector<cv::Point2f> ref_points_;
    std::vector<cv::Point2f> points_;

    std::vector<bool> status_;
    cv::Mat error_;

    unsigned long count_ = 1;
    std::vector<float> trust_;


};

#endif // PointStabilizer_HPP

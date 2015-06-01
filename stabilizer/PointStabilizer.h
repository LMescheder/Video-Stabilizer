//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

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
    PointStabilizer(const cv::Mat& frame_0, Warping warping=Warping::HOMOGRAPHY, Mode mode=Mode::DIRECT,
                    bool use_checked_flow=true, bool use_ransac=false, int lk_levels=2, int lk_levels_retrieve=4);

protected:
    virtual void track_ref();
    virtual cv::Mat get_next_homography (const cv::Mat& next_image);
    virtual void create_visualization ();

private:
    std::vector<cv::Point2f> calc_optical_flow_(const cv::Mat& frame_gray, float eps);

private:
    // Parameters
    double max_flow_err = 1.e-1;
    double max_flow_err_retrieve = 2.e-1;
    int min_points = 100;
    int lk_levels_ = 2;
    int lk_levels_retrieve_ = 4;

    int features_maxN_ = 2000;
    double features_quality_ = 1e-2;
    int features_mindist_ = 5;

    bool use_checked_optical_flow_ = true;
    bool use_ransac_ = false;

    // Status
    cv::Mat last_frame_gray_;

    std::vector<cv::Point2f> points_0_;
    std::vector<cv::Point2f> ref_points_;
    std::vector<cv::Point2f> points_;

    std::vector<bool> status_;
    cv::Mat error_;

    size_t good_points_count_;
    std::vector<float> trust_;


};

#endif // PointStabilizer_HPP

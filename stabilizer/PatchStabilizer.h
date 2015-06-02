//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

#ifndef PATCHSTABILIZER_H
#define PATCHSTABILIZER_H

#include "Stabilizer.h"
#include <tuple>
#include <vector>

class PatchStabilizer : public Stabilizer
{
public:
    struct OpticalFlowParameters {
        OpticalFlowParameters() = default;
        OpticalFlowParameters(float max_err_p, int lk_levels_p, bool use_checked_optical_flow_p)
            : max_err{max_err_p}, lk_levels{lk_levels_p}, use_checked_optical_flow(use_checked_optical_flow_p) {}

        float max_err = 1.e0;
        int lk_levels = 3;
        bool use_checked_optical_flow = true;
    };

public:
    PatchStabilizer(const cv::Mat &frame_0);


protected:
    virtual cv::Mat get_next_homography(const cv::Mat& next_frame);
    virtual void create_visualization();

private:
    std::vector<cv::Point2f> calc_optical_flow_(const cv::Mat& frame_gray, float eps);

private:
    using Vec8f = cv::Vec<float, 8>;
    using Matx88f = cv::Matx<float, 8, 8>;

    OpticalFlowParameters flow_params_, flow_params_retrieve_;

    std::vector<cv::Point2f> points_0_;
    std::vector<cv::Point2f> points_;
    std::vector<bool> status_;

    std::vector<cv::Matx22f> Ais_;
    cv::Mat_<cv::Vec2f> gradI0_;
    cv::Mat frame0_;

    size_t good_points_count_;
    std::vector<float> trust_;

private:
    void init (const cv::Mat& frame);

    static constexpr int N_PATCHES_X = 50;
    static constexpr int N_PATCHES_Y = 50;
    static constexpr float EPS = 1e-3;


};
#endif // PATCHSTABILIZER_H

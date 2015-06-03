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

        float max_err_weighted = .5e-1;
        float max_err = 5e9;
        int lk_levels = 3;
        bool use_checked_optical_flow = true;
        double minEigThreshold = 1e-6;
    };

    struct PatchParameters {
        PatchParameters() = default;
        PatchParameters(unsigned int Nx_p, unsigned int Ny_p)
            : Nx{Nx_p}, Ny{Ny_p} {}

        unsigned int Nx = 40;
        unsigned int Ny = 30;
        float sigma_grad = 0.7f;
        float sigma_integrate = 2.f;
    };

    struct HomographyEstimationParameters {
        float regularize = .0f;
        float reweight_lambda = 1.f;
        int maxiter = 100;
    };

public:
    PatchStabilizer(const cv::Mat &frame_0, PatchParameters patch_params={},HomographyEstimationParameters homography_params={},
                    OpticalFlowParameters flow_params={}, OpticalFlowParameters flow_params_retrieve={});


protected:
    virtual cv::Mat get_next_homography(const cv::Mat& next_frame);
    virtual void create_visualization();

private:
    std::vector<cv::Point2f> calc_optical_flow_(const cv::Mat& frame_gray, float eps, float eps_weighted);

private:
    // parameters
    OpticalFlowParameters flow_params_, flow_params_retrieve_;
    PatchParameters patch_params_;
    HomographyEstimationParameters homography_params_;

    // status
    using Vec8f = cv::Vec<float, 8>;
    using Matx88f = cv::Matx<float, 8, 8>;

    std::vector<cv::Point2f> points_0_;
    std::vector<cv::Point2f> points_;
    std::vector<bool> status_;

    std::vector<cv::Matx22f> Ais_;
    std::vector<float> max_eigvals_;
    cv::Mat_<cv::Vec2f> gradI0_;
    cv::Mat frame0_;

    size_t good_points_count_;
    std::vector<float> trust_;

private:
    void init (const cv::Mat& frame);

};
#endif // PATCHSTABILIZER_H

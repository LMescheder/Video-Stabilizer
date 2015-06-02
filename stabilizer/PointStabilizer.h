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
    struct FeatureExtractionParameters {
        FeatureExtractionParameters() = default;
        FeatureExtractionParameters(int maxN_p, double quality_p, double mindist_p)
            : maxN{maxN_p}, quality{quality_p}, mindist{mindist_p} {}

        int maxN = 1000;
        double quality = 1e-2;
        double mindist = 5.;
    };

    struct OpticalFlowParameters {
        OpticalFlowParameters() = default;
        OpticalFlowParameters(float max_err_p, int lk_levels_p, bool use_checked_optical_flow_p)
            : max_err{max_err_p}, lk_levels{lk_levels_p}, use_checked_optical_flow(use_checked_optical_flow_p) {}

        float max_err = 1.e-1;
        int lk_levels = 3;
        bool use_checked_optical_flow = true;
    };

    struct HomographyEstimationParameters {
        HomographyEstimationParameters() = default;
        HomographyEstimationParameters(int min_points_p, bool use_ransac_p)
            : min_points{min_points_p}, use_ransac{use_ransac_p} {}

        int min_points = 100;
        bool use_ransac = false;
    };

public:
    PointStabilizer(const cv::Mat& frame_0, Warping warping=Warping::HOMOGRAPHY, Mode mode=Mode::DIRECT,
                    FeatureExtractionParameters feature_params = {2000, 1e-2, 5},
                    OpticalFlowParameters flow_params = {1.e-1, 3, true},
                    OpticalFlowParameters flow_params_retrieve = {2.e-1, 4, true},
                    HomographyEstimationParameters homography_params = {100, false});

protected:
    virtual void track_ref();
    virtual cv::Mat get_next_homography (const cv::Mat& next_image);
    virtual void create_visualization ();

private:
    std::vector<cv::Point2f> calc_optical_flow_(const cv::Mat& frame_gray, float eps);

private:
    // Parameters

    FeatureExtractionParameters feature_params_;
    OpticalFlowParameters flow_params_, flow_params_retrieve_;
    HomographyEstimationParameters homography_params_;

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

//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

#include "PointStabilizer.h"


PointStabilizer::PointStabilizer(const cv::Mat& frame_0, Warping warping, Mode mode,
                                 FeatureExtractionParameters feature_params,
                                 OpticalFlowParameters flow_params,
                                 OpticalFlowParameters flow_params_retrieve,
                                 HomographyEstimationParameters homography_params)
    : Stabilizer(frame_0, warping, mode, true),
      feature_params_(feature_params), flow_params_(flow_params), flow_params_retrieve_(flow_params_retrieve),
      homography_params_(homography_params)
{
    cv::goodFeaturesToTrack(ref_frame_gray_, points_0_, feature_params_.maxN, feature_params_.quality, feature_params_.mindist);
    points_ = ref_points_ = points_0_;
    status_.resize(ref_points_.size());
    trust_.resize(ref_points_.size(), .5);
    last_frame_gray_ = ref_frame_gray_;
}


void PointStabilizer::track_ref()
{
    Stabilizer::track_ref();
    ref_points_ = points_;
}

cv::Mat PointStabilizer::get_next_homography(const cv::Mat &next_image)
{
    if (good_points_count_ < homography_params_.min_points)
        points_ = calc_optical_flow_(next_image, flow_params_retrieve_.max_err);
    else
        points_ = calc_optical_flow_(next_image, flow_params_.max_err);

    // select only good points
    std::vector<cv::Point2f> good_points_0;
    std::vector<cv::Point2f> good_points;

    good_points_0.reserve(points_.size());
    good_points.reserve(points_.size());
    for (std::size_t i=0; i<points_.size(); ++i)
        if (status_[i] && trust_[i] >= .5) {
            good_points_0.push_back(points_0_[i]);
            good_points.push_back(points_[i]);
         }

    // estimate homography
    good_points_count_ = good_points.size();
    if (good_points_count_ < homography_params_.min_points)
        return cv::Mat();
    else
      return find_homography(good_points, good_points_0, warping_, homography_params_.use_ransac);
}

void PointStabilizer::create_visualization() {
    std::vector<cv::Point2f> vis_points = points_;
    if (mode_ == Mode::WARP_BACK)
        cv::perspectiveTransform(vis_points, vis_points, H_.inv());

    for (std::size_t i=0; i<ref_points_.size(); ++i) {
        // select color
        cv::Scalar color;
        bool active = false;
        if (trust_[i] < .5)
            color = cv::Scalar(0, 0, 255);
        else if (!status_[i])
            color = cv::Scalar(0, 255, 255);
        else {
            color = cv::Scalar(0, 255, 0);
            active = true;
        }

        // draw points and lines
        cv::circle(visualization_, points_0_[i], 2, color);
        if (active) {
            cv::circle(visualization_, vis_points[i], 3, color);
            cv::line(visualization_,  points_0_[i],  vis_points[i], color);
        }
    }
}

std::vector<cv::Point2f> PointStabilizer::calc_optical_flow_(const cv::Mat& frame_gray, float eps) {

    // select levels of lukas kanade
    int lk_levels = (good_points_count_ >= homography_params_.min_points) ? flow_params_.lk_levels : flow_params_retrieve_.lk_levels;
    cv::Mat err;
    std::vector<cv::Point2f> points1;

    if (flow_params_.use_checked_optical_flow) {
        cv::Mat status1, status2;
        std::vector<cv::Point2f> points2;
        points1.reserve(ref_points_.size());
        points2.reserve(ref_points_.size());


        cv::calcOpticalFlowPyrLK(ref_frame_gray_, frame_gray, ref_points_, points1, status1, err, cv::Size(21, 21), lk_levels);
        cv::calcOpticalFlowPyrLK(frame_gray, ref_frame_gray_, points1, points2, status2, err, cv::Size(21, 21), lk_levels);

        for (std::size_t i=0; i<points_.size(); ++i) {
            status_[i] = (status1.at<bool>(i) && status2.at<bool>(i) && cv::norm(ref_points_[i] - points2[i]) < eps );
            float new_trust = status_[i] ? 1.f : 0.f;
            //trust_[i] = (count_*trust_[i] + new_trust)/(count_+1);
            trust_[i] = .95 * trust_[i] + .05 * new_trust;
        }
    } else {
        cv::Mat status;
        cv::calcOpticalFlowPyrLK(ref_frame_gray_, frame_gray, ref_points_, points1, status, err, cv::Size(21, 21), lk_levels);
        for (std::size_t i=0; i<points_.size(); ++i)
            status_[i] = status.at<bool>(i);
    }

    return points1;
}


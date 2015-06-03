//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

#include "PatchStabilizer.h"

PatchStabilizer::PatchStabilizer(const cv::Mat& frame_0, PatchParameters patch_params, HomographyParameters homography_params, OpticalFlowParameters flow_params, OpticalFlowParameters flow_params_retrieve)
    : Stabilizer(frame_0, Warping::HOMOGRAPHY, Mode::WARP_BACK, true),
      patch_params_{patch_params}, homography_params_{homography_params}, flow_params_{flow_params}, flow_params_retrieve_{flow_params_retrieve}
{
    init(ref_frame_gray_);
}

cv::Mat PatchStabilizer::get_next_homography(const cv::Mat &next_image)
{
    points_ = calc_optical_flow_(next_image, flow_params_.max_err, flow_params_.max_err_weighted);
    // calculate homography
    Vec8f h = 0;
    cv::Mat H = cv::Mat::eye(3, 3, CV_32F);
    Matx88f A;
    Vec8f b;

    std::vector<cv::Point2f> Hinv_points = points_;
   // good_points_count_ = 0;
    // accumulate A and b

    for (unsigned iter = 0; iter < homography_params_.maxiter; ++iter) {
        if (iter != 0)
            cv::perspectiveTransform(points_, Hinv_points, H.inv());

        float sum_weights = 0.;
        for (unsigned i = 0; i < patch_params_.Nx*patch_params_.Ny; ++i) {
             if (status_[i] && trust_[i] >= .5) {
                 //++good_points_count_;
                 cv::Vec2f p0 = points_0_[i];
                 cv::Vec2f Hinv_p1 = Hinv_points[i];
                 cv::Vec2f p1 = points_[i];

                 cv::Vec2f dist = Hinv_p1 - p0;
                 float D = h(6) * p0(0) + h(7) * p0(1) + 1.;
                 float lambda = homography_params_.reweight_lambda;
                 float weight = (iter == 0) ? 1.f : 1./(1 + lambda * std::sqrt(dist.dot(Ais_[i] * dist)));
                 cv::Matx<float, 2, 8> Jx = {p0[0],   p0[1],   1.f, 0.f, 0.f, 0.f, -p1[0]*p0[0], -p1[0]*p0[1],
                                             0.f, 0.f, 0.f, p0[0],   p0[1],   1.f, -p1[0]*p0[1], -p1[1]*p0[1]};
                 b += weight/D * Jx.t() * Ais_[i] * (p1 - p0);
                 A += weight/D * Jx.t() * Ais_[i] * Jx;
                 sum_weights += weight;
             }
        }
        A = 1./sum_weights * A + homography_params_.regularize * Matx88f::eye();
        b =  1./sum_weights * b;
        // solve for h
        cv::solve(A, b, h, cv::DECOMP_CHOLESKY);

        H = (cv::Mat_<double>(3, 3) << 1 + h(0), h(1), h(2),
                                      h(3), 1 + h(4), h(5),
                                      h(6), h(7), 1.f );

    }

    return H.inv();
}


void PatchStabilizer::create_visualization()
{
    std::vector<cv::Point2f> vis_points = points_;
    if (mode_ == Mode::WARP_BACK)
        cv::perspectiveTransform(vis_points, vis_points, H_.inv());

    for (std::size_t i=0; i<points_0_.size(); ++i) {
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




void PatchStabilizer::init(const cv::Mat& frame0)
{
    // some parameters
    int width = frame0.cols;
    int height = frame0.rows;

    float patch_width = static_cast<float>(width) / patch_params_.Nx;
    float patch_height = static_cast<float>(height) / patch_params_.Ny;


    // clone reference frame
    frame0_ = frame0.clone();

    // compute derivative of reference image
    cv::Mat frame_blurred, frame_blurred0;
    cv::GaussianBlur(frame0_, frame_blurred0, cv::Size(5, 5), 1.);
    frame_blurred0.convertTo(frame_blurred, CV_32F);

    cv::Mat I0x, I0y;
    cv::Sobel(frame_blurred, I0x, CV_32F, 1, 0);
    cv::Sobel(frame_blurred, I0y, CV_32F, 0, 1);

    cv::Mat Fxx, Fyy, Fxy;
    cv::multiply(I0x, I0x, Fxx);
    cv::multiply(I0x, I0y, Fxy);
    cv::multiply(I0y, I0y, Fyy);

    cv::GaussianBlur(Fxx, Fxx, cv::Size(5, 5), 2.);
    cv::GaussianBlur(Fyy, Fyy, cv::Size(5, 5), 2.);
    cv::GaussianBlur(Fxy, Fxy, cv::Size(5, 5), 2.);


    Ais_.resize(patch_params_.Nx * patch_params_.Ny);
    points_0_.resize(patch_params_.Nx * patch_params_.Ny);

    for (int ip_y = 0; ip_y < patch_params_.Ny; ++ip_y) {
        for (int ip_x = 0; ip_x < patch_params_.Nx; ++ip_x) {
            int idx = ip_y * patch_params_.Nx + ip_x;

            int xmin = static_cast<int>(ip_x * patch_width);
            int xmax = static_cast<int>((ip_x+1) * patch_width);
            int ymin = static_cast<int>(ip_y * patch_height);
            int ymax = static_cast<int>((ip_y+1) * patch_height);

            float max_eig = -1;

            // find best keypoint
            for (int i_y = ymin; i_y < ymax; ++i_y) {
                for (int i_x = xmin; i_x < xmax; ++i_x) {
                    cv::Matx22f Ai = {Fxx.at<float>(i_y, i_x), Fxy.at<float>(i_y, i_x),
                                      Fxy.at<float>(i_y, i_x), Fyy.at<float>(i_y, i_x)};
                    float D = (Ai(0, 0) - Ai(1, 1))*(Ai(0, 0) - Ai(1, 1)) + Ai(1, 0)*Ai(0, 1);
                    float eig = Ai(0, 0) + Ai(1, 1); // + std::sqrt(D));
                    if (eig > max_eig) {
                        Ais_[idx] = Ai;
                        points_0_[idx] = {i_x, i_y};
                        max_eig = eig;
                    }
                }
            }

            assert(max_eig >= 0.f);

        }
    }

    status_.resize(points_0_.size(), true);
    trust_.resize(points_0_.size(), 1.);


}

std::vector<cv::Point2f> PatchStabilizer::calc_optical_flow_(const cv::Mat& frame_gray, float eps, float eps_weighted) {
    int lk_levels = flow_params_.lk_levels;

    std::vector<cv::Point2f> points1, points2;
    cv::Mat status1, status2;
    points1.reserve(points_0_.size());
    points2.reserve(points_0_.size());

    cv::Mat err;

    cv::calcOpticalFlowPyrLK(ref_frame_gray_, frame_gray, points_0_, points1, status1, err, cv::Size(21, 21), lk_levels,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), 0, flow_params_.minEigThreshold);
    cv::calcOpticalFlowPyrLK(frame_gray, ref_frame_gray_, points1, points2, status2, err, cv::Size(21, 21), lk_levels,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), 0, flow_params_.minEigThreshold);

    for (std::size_t i=0; i < points_0_.size(); ++i) {
        cv::Vec2f dist = points_0_[i] - points2[i];
        float error2 = dist.dot(Ais_[i] * dist);
        status_[i] = (status1.at<bool>(i) && status2.at<bool>(i) && cv::norm(dist) < eps && (error2 < eps_weighted * eps_weighted) );
        float new_trust = status_[i] ? 1.f : 0.f;
        trust_[i] = .95 * trust_[i] + .05 * new_trust;
    }

    return points1;
}

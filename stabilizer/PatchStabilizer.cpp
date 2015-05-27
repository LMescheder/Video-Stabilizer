#include "PatchStabilizer.h"

PatchStabilizer::PatchStabilizer(const cv::Mat& frame_0, Warping warping)
    : Stabilizer(frame_0, warping, Mode::WARP_BACK, true)
{
    init(ref_frame_gray_);
}

cv::Mat PatchStabilizer::get_next_homography(const cv::Mat &next_frame)
{
    size_t width = ref_frame_gray_.cols;
    size_t height = ref_frame_gray_.rows;

    int patch_width = width / N_PATCHES_X;
    int patch_height = height / N_PATCHES_Y;
    float one_over_patch_N = 1./(patch_width * patch_height);

    cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
    Vec8f h = 0;
    cv::Mat frame = next_frame.clone();

    for (int i = 0; i < MAXITER; ++i) {
        cv::Mat_<uchar> error = frame - ref_frame_gray_;
        Vec8f b = 0;
        Matx88f A = 0;

        // accumulate A and b
        for (int ip_y = 0; ip_y < N_PATCHES_Y; ++ip_y) {
            for (int ip_x = 0; ip_x < N_PATCHES_X; ++ip_x) {
                int idx = ip_y * N_PATCHES_Y + ip_x;

                // compute bi
                cv::Vec2f bi = 0;
                int N = 0;
                for (int i_y = ip_y * patch_height; i_y < (ip_y + 1) * patch_height; ++i_y)
                    for (int i_x = ip_x * patch_width; i_x < (ip_x + 1) * patch_width; ++i_y)
                        if (frame.at<uchar>(i_y, i_x) != 0) {
                            bi += static_cast<float>(error(i_y, i_x)) * gradI0_(i_y, i_x);
                            ++N;
                        }

                float weight = N * one_over_patch_N;
                float x = pis_[idx].x;
                float y = pis_[idx].y;
                cv::Matx<float, 2, 8> Jx = {x,   y,   1.f, 0.f, 0.f, 0.f, -x*x, -x*y,
                                            0.f, 0.f, 0.f, x,   y,   1.f, -x*y, -y*y};
                b += weight * Jx.t() * bi;
                A += weight * Jx.t() * Ais_[idx] * Jx;
            }
        }

        // solve for h
        cv::solve(A, b, h, cv::DECOMP_CHOLESKY);

        cv::Mat dH = (cv::Mat_<double>(3, 3) << 1 + h(0), h(1), h(2),
                                              h(3), 1 + h(4), h(5),
                                              h(6), h(7), 1.f );

        H = H * dH;
        cv::warpPerspective(next_frame, frame, H.inv(), cv::Size(width, height));
        if (cv::norm(h) < EPS)
            break;
    }

    return H.inv();

}
void PatchStabilizer::create_visualization()
{

}

void PatchStabilizer::init(const cv::Mat& frame)
{
    int width = frame.cols;
    int height = frame.rows;

    int patch_width = width / N_PATCHES_X;
    int patch_height = height / N_PATCHES_Y;
    float one_over_patch_N = 1./(patch_width * patch_height);

    // compute derivative of reference image
    cv::Mat frame_blurred, frame_blurred0;
    cv::GaussianBlur(frame, frame_blurred0, cv::Size(5, 5), 1.);
    frame_blurred0.convertTo(frame_blurred, CV_32F);

    cv::Mat gradI0_channels[2];
    cv::Sobel(frame_blurred, gradI0_channels[0], CV_32F, 1, 0);
    cv::Sobel(frame_blurred, gradI0_channels[1], CV_32F, 0, 1);

    // compute gradI0
    cv::merge(gradI0_channels, 2, gradI0_);

    // compute Ai and pis
    Ais_.resize(N_PATCHES_X * N_PATCHES_Y);
    pis_.resize(N_PATCHES_X * N_PATCHES_Y);

    for (int ip_y = 0; ip_y < N_PATCHES_Y; ++ip_y) {
        for (int ip_x = 0; ip_x < N_PATCHES_X; ++ip_x) {
            int idx = ip_y * N_PATCHES_Y + ip_x;
            pis_[idx] = cv::Point2f((ip_x + .5)*patch_width, (ip_y + .5)*patch_height);
            for (int i_y = ip_y * patch_height; i_y < (ip_y + 1) * patch_height; ++i_y) {
                for (int i_x = ip_x * patch_width; i_x < (ip_x + 1) * patch_width; ++i_y) {
                    Ais_[idx] += one_over_patch_N * gradI0_(i_y, i_x) * gradI0_(i_y, i_x).t();
                }
            }
        }
    }

}


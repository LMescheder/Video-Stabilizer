#include "PatchStabilizer.h"

PatchStabilizer::PatchStabilizer(const cv::Mat& frame_0, Warping warping)
    : Stabilizer(frame_0, warping, Mode::DIRECT, true)
{
    init(ref_frame_gray_);
}

cv::Mat PatchStabilizer::get_next_homography(const cv::Mat &next_frame)
{
    std::vector<cv::Mat> next_frame_pyr;
    next_frame_pyr.reserve(PYRAMID_N);
    // create pyramid
    cv::Mat next_frame_down = next_frame.clone();

    for (int i = 0; i < PYRAMID_N; ++i) {
        if (i != 0)
            cv::pyrDown(next_frame_down, next_frame_down);
        next_frame_pyr.push_back(next_frame_down.clone());
    }

    // estimate homography
    cv::Mat H = H_.clone();

    H(cv::Range(2, 3), cv::Range(0, 2)) *= (1 << (PYRAMID_N));
    H(cv::Range(0, 2), cv::Range(2, 3)) /= (1 << (PYRAMID_N));

    for (int i = PYRAMID_N-1; 0 <= i; --i) {
        // adapt previous H to new scaling
        H(cv::Range(2, 3), cv::Range(0, 2)) /= 2;
        H(cv::Range(0, 2), cv::Range(2, 3)) *= 2;

        int width = next_frame_pyr[i].cols;
        int height = next_frame_pyr[i].rows;
        cv::warpPerspective(next_frame_pyr[i], next_frame_pyr[i], H, cv::Size(width, height));

        cv::Mat new_H = get_homography(next_frame_pyr[i], frame0_pyr_[i], gradI0_pyr_[i], Ais_pyr_[i]);
        H = new_H * H;
        //std::cout << "new_H = " << new_H << std::endl;
        //std::cout << "H = " << H << std::endl;
    }

    return H;
}
void PatchStabilizer::create_visualization()
{

}

cv::Mat PatchStabilizer::get_homography(const cv::Mat& next_frame, const cv::Mat& frame0, const cv::Mat_<cv::Vec2f>& gradI, const std::vector<cv::Matx22f>& Ais)
{
    size_t width = frame0.cols;
    size_t height = frame0.rows;

    int patch_width = width / N_PATCHES_X;
    int patch_height = height / N_PATCHES_Y;
    float one_over_patch_N = 1./(patch_width * patch_height);

    cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
    Vec8f h = 0;
    cv::Mat frame = next_frame.clone();

    for (int i = 0; i < MAXITER; ++i) {
        cv::Mat_<uchar> error = frame - frame0;
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
                    for (int i_x = ip_x * patch_width; i_x < (ip_x + 1) * patch_width; ++i_x)
                        if (frame.at<uchar>(i_y, i_x) != 0) {
                            bi -= error(i_y, i_x) * gradI(i_y, i_x);
                            ++N;
                        }

                float weight = N * one_over_patch_N;
                float x = (ip_x + .5) * patch_width;
                float y = (ip_y + .5) * patch_height;

                cv::Matx<float, 2, 8> Jx = {x,   y,   1.f, 0.f, 0.f, 0.f, -x*x, -x*y,
                                            0.f, 0.f, 0.f, x,   y,   1.f, -x*y, -y*y};
                b += Jx.t() * bi;
                A += weight * Jx.t() * Ais[idx] * Jx;
            }
        }

        // solve for h
        cv::solve(A, b, h, cv::DECOMP_CHOLESKY);

        cv::Mat new_H = (cv::Mat_<double>(3, 3) << 1 + h(0), h(1), h(2),
                                              h(3), 1 + h(4), h(5),
                                              h(6), h(7), 1.f );

        H = H * new_H;
        cv::warpPerspective(next_frame, frame, H.inv(), cv::Size(width, height));
        if (cv::norm(h) < EPS)
            break;
    }

    return H.inv();
}

std::tuple<cv::Mat, std::vector<cv::Matx22f>> PatchStabilizer::compute_Ais_and_gradI0(const cv::Mat& frame)
{
    cv::Mat_<cv::Vec2f> gradI0;
    std::vector<cv::Matx22f> Ais;

    // some parameters
    int width = frame.cols;
    int height = frame.rows;

    int patch_width = width / N_PATCHES_X;
    int patch_height = height / N_PATCHES_Y;

    // compute derivative of reference image
    cv::Mat frame_blurred, frame_blurred0;
    cv::GaussianBlur(frame, frame_blurred0, cv::Size(5, 5), 1.);
    frame_blurred0.convertTo(frame_blurred, CV_32F);

    cv::Mat gradI0_channels[2];
    cv::Sobel(frame_blurred, gradI0_channels[0], CV_32F, 1, 0);
    cv::Sobel(frame_blurred, gradI0_channels[1], CV_32F, 0, 1);

    // compute gradI0
    cv::merge(gradI0_channels, 2, gradI0);

    // compute Ai
    Ais.resize(N_PATCHES_X * N_PATCHES_Y);

    for (int ip_y = 0; ip_y < N_PATCHES_Y; ++ip_y) {
        for (int ip_x = 0; ip_x < N_PATCHES_X; ++ip_x) {
            int idx = ip_y * N_PATCHES_Y + ip_x;

            Ais[idx] = 0;
            for (int i_y = ip_y * patch_height; i_y < (ip_y + 1) * patch_height; ++i_y) {
                for (int i_x = ip_x * patch_width; i_x < (ip_x + 1) * patch_width; ++i_x) {
                    Ais[idx] += gradI0(i_y, i_x) * gradI0(i_y, i_x).t();
                }
            }
        }
    }

    return std::make_tuple(gradI0, Ais);
}

void PatchStabilizer::init(const cv::Mat& frame0)
{
    Ais_pyr_.reserve(PYRAMID_N);
    gradI0_pyr_.reserve(PYRAMID_N);
    frame0_pyr_.reserve(PYRAMID_N);

    cv::Mat frame0_down = frame0.clone();

    for (int i = 0; i < PYRAMID_N; ++i) {
        if (i != 0)
            cv::pyrDown(frame0_down, frame0_down);
        std::vector<cv::Matx22f> Ais;
        cv::Mat gradI0;
        std::tie(gradI0, Ais) = compute_Ais_and_gradI0(frame0_down);
        gradI0_pyr_.push_back(gradI0.clone());
        Ais_pyr_.push_back(std::move(Ais));
        frame0_pyr_.push_back(frame0_down.clone());
    }
}


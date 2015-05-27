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

    cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
    Vec8f h = 0;
    cv::Mat frame = next_frame.clone();

    constexpr float EPS = 1e-2;
    constexpr int MAXITER = 50;

    for (int i = 0; i < MAXITER; ++i) {
        cv::Mat error = frame - ref_frame_gray_;
        Vec8f b = 0;
        Matx88f A = 0;

        // accumulate A and b
        int N = 0;
        for (size_t i = 0; i < height; ++i)
            for (size_t j = 0; j < width; ++j)
                if (frame.at<uchar>(i, j) != 0) {
                    int idx = i * height + j;
                    A += A_[idx];
                    b -= static_cast<float>(error.at<uchar>(i, j)) * Ji_[idx];
                    ++N;
                }
        A = 1./N * A;
        b = 1./N * b;

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

    // compute derivative of reference image
    cv::Mat frame_blurred, frame_blurred0;
    cv::Mat gradIx, gradIy;
    cv::GaussianBlur(frame, frame_blurred0, cv::Size(5, 5), 1.);
    frame_blurred0.convertTo(frame_blurred, CV_32F);
    cv::Sobel(frame_blurred, gradIx, CV_32F, 1, 0);
    cv::Sobel(frame_blurred, gradIy, CV_32F, 0, 1);

    // compute A and Ji
    Ji_.resize(width * height);
    A_.resize(width * height);

    for (int i=0; i<height; ++i) {
        for (int j=0; j<width; ++j) {
            int idx = i * height + j;
            float x = static_cast<float> (j);
            float y = static_cast<float> (i);
            cv::Matx<float, 2, 8> Jx = {x,   y,   1.f, 0.f, 0.f, 0.f, -x*x, -x*y,
                                        0.f, 0.f, 0.f, x,   y,   1.f, -x*y, -y*y};
            cv::Vec2f gradI (gradIx.at<float>(i, j), gradIy.at<float>(i, j));

            A_[idx] = Jx.t() * gradI * gradI.t() * Jx;
            Ji_[idx] =  Jx.t() * gradI;
        }
    }
}


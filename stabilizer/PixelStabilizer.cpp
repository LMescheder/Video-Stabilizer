#include "PixelStabilizer.h"

PixelStabilizer::PixelStabilizer(const cv::Mat& frame_0, Warping warping)
    : Stabilizer(frame_0, warping, Mode::DIRECT, true)
{
    init(ref_frame_gray_);
}

cv::Mat PixelStabilizer::get_next_homography(const cv::Mat &next_frame)
{
    int width = ref_frame_gray_.cols;
    int height = ref_frame_gray_.rows;

    cv::Mat error = next_frame - ref_frame_gray_;
    Vec8f b = 0;

    // accumulate b
    for (size_t i = 0; i < height; ++i)
        for (size_t j = 0; j < width; ++j)
            b += error.at<uchar>(i, j) * Ji_.at<Vec8f>(i, j);

    Vec8f h = Ainv_ * b;

    cv::Mat H = (cv::Mat_<double>(3, 3) << 1 + h(0), h(1), h(2),
                                          h(3), 1 + h(4), h(5),
                                          h(6), h(7), 1.f );

    return H.inv();

}
void PixelStabilizer::create_visualization()
{

}

void PixelStabilizer::init(const cv::Mat& frame)
{
    int width = ref_frame_gray_.cols;
    int height = ref_frame_gray_.rows;

    // compute derivative of reference image
    cv::Mat ref_frame_gray_blurred;
    cv::Mat gradIx, gradIy;
    cv::GaussianBlur(ref_frame_gray_, ref_frame_gray_blurred, cv::Size(5, 5), .01);
    ref_frame_gray_blurred.convertTo(ref_frame_gray_blurred, CV_32F);
    cv::Sobel(ref_frame_gray_blurred, gradIx, CV_32F, 1, 0);
    cv::Sobel(ref_frame_gray_blurred, gradIy, CV_32F, 0, 1);

    // compute A
    Matx88f A = 0;
    Ji_ = cv::Mat::zeros(height, width, CV_32FC(8));
    int N = width * height;
    for (int i=0; i<height; ++i) {
        for (int j=0; j<width; ++j) {
            float x = static_cast<float> (j);
            float y = static_cast<float> (i);
            cv::Matx<float, 2, 8> Jx = {x,   y,   1.f, 0.f, 0.f, 0.f, -x*x, -x*y,
                                        0.f, 0.f, 0.f, x,   y,   1.f, -x*y, -y*y};
            cv::Vec2f gradI (256 * gradIx.at<float>(i, j), 256 * gradIy.at<float>(i, j));

            A += Jx.t() * gradI * gradI.t() * Jx;
            Ji_.at<Vec8f>(i, j) =  Jx.t() * gradI;
        }
    }

    Ainv_ = A.inv();
}


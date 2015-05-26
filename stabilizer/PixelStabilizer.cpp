#include "PixelStabilizer.h"

PixelStabilizer::PixelStabilizer(const cv::Mat& frame_0, Warping warping, Mode mode)
  : Stabilizer(frame_0, warping, mode, true)
{


}

cv::Mat PixelStabilizer::get_next_homography(const cv::Mat &next_frame)
{

}

cv::Mat PixelStabilizer::get_b(cv::Mat frame)
{

  cv::Mat b;

  int width = ref_frame_gray_.cols;
  int height = ref_frame_gray_.rows;

}

cv::Mat PixelStabilizer::get_A(cv::Mat frame)
{
  int width = ref_frame_gray_.cols;
  int height = ref_frame_gray_.rows;

  // compute derivative of reference image
  cv::Mat ref_frame_gray_blurred;
  cv::Mat gradIx, gradIy;
  cv::GaussianBlur(ref_frame_gray_, ref_frame_gray_blurred, cv::Size(5, 5), 1.);
  cv::Sobel(ref_frame_gray_blurred, gradIx, 1, 1, 0);
  cv::Sobol(ref_frame_gray_blurred, gradIy, 1, 0, 1);

  // compute A
  for (int i=0; i<width; ++i) {
    for (int j=0; j<height; ++j) {
      // to do compute contribution to A
    }
  }
}


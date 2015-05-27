#ifndef PATCHSTABILIZER_H
#define PATCHSTABILIZER_H

#include "Stabilizer.h"

class PatchStabilizer : public Stabilizer
{
public:
  PatchStabilizer(const cv::Mat &frame_0, Warping warping);


protected:
  virtual cv::Mat get_next_homography(const cv::Mat& next_frame);
  virtual void create_visualization();

 private:
  using Vec8f = cv::Vec<float, 8>;
  using Matx88f = cv::Matx<float, 8, 8>;

  std::vector<cv::Matx22f> Ais_;
  std::vector<cv::Point2f> pis_;
  cv::Mat_<cv::Vec2f> gradI0_;

private:
  void init (const cv::Mat& frame);

  static constexpr int N_PATCHES_X = 100;
  static constexpr int N_PATCHES_Y = 100;
  static constexpr float EPS = 1e-3;
  static constexpr int MAXITER = 50;

};
#endif // PATCHSTABILIZER_H

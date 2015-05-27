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

  std::vector<Matx88f> A_;
  std::vector<Vec8f> Ji_;

private:
  void init (const cv::Mat& frame);
};
#endif // PATCHSTABILIZER_H

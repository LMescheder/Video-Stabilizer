#ifndef PIXELSTABILIZER_H
#define PIXELSTABILIZER_H

#include "opencv2/opencv.hpp"
#include "Stabilizer.h"

class PixelStabilizer : public Stabilizer
{
public:
  PixelStabilizer(const cv::Mat &frame_0, Warping warping, Mode mode);


protected:
  virtual cv::Mat get_next_homography(const cv::Mat& next_frame);
  virtual void create_visualization();

 private:
  cv::Mat A_, b_;

private:
  cv::Mat get_b (cv::Mat frame);
  cv::Mat get_A (cv::Mat frame);
};

#endif // PIXELSTABILIZER_H

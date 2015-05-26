#ifndef PIXELSTABILIZER_H
#define PIXELSTABILIZER_H

#include "opencv2/opencv.hpp"
#include "Stabilizer.h"

class PixelStabilizer
{
public:
  PixelStabilizer();


protected:
  virtual cv::Mat get_next_homography(const cv::Mat& next_frame) = 0;
  virtual void create_visualization() = 0;
};

#endif // PIXELSTABILIZER_H

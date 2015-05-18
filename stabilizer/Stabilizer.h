#ifndef STABILIZER_HPP
#define STABILIZER_HPP


#include "opencv2/opencv.hpp"
#include "utilities.h"

/**
 * @brief Interface for video stabilization.
 */

class Stabilizer
{
public:
    virtual cv::Mat stabilize_next(const cv::Mat& next_frame) = 0;
    virtual cv::Mat visualization() const = 0;

};

#endif // STABILIZER_HPP

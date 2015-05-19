#ifndef PointStabilizer_HPP
#define PointStabilizer_HPP

#include <vector>
#include "opencv2/opencv.hpp"
#include "Stabilizer.h"

/**
 * @brief The PointStabilizer class implements a basic video stabilizer based on key point tracking.
 */
class PointStabilizer : public Stabilizer{
public:
    PointStabilizer(const cv::Mat& frame_0, Warping warping=Warping::HOMOGRAPHY, Mode mode=Mode::DIRECT);

    /**
     * @brief Implementation of stabilize_next method of the abstract Stabilizer class
     * @param next_frame        The next colored frame to stabilize.
     * @return                  The stabilized frame.
     */
    virtual cv::Mat stabilize_next(const cv::Mat& next_frame);

protected:
    virtual cv::Mat get_next_homography (const cv::Mat& next_image);
    virtual void create_visualization ();

private:
    std::vector<cv::Point2f> checked_optical_flow_(const cv::Mat& frame_gray, float eps);

private:
    // Parameters
    double max_flow_err = 1e-1;

    // Status
    cv::Mat last_frame_gray_;

    std::vector<cv::Point2f> points_0_;
    std::vector<cv::Point2f> ref_points_;
    std::vector<cv::Point2f> points_;

    std::vector<bool> status_;
    cv::Mat error_;

    unsigned long count_ = 1;
    std::vector<float> trust_;


};

#endif // PointStabilizer_HPP

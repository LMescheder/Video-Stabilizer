#ifndef PointStabilizer_HPP
#define PointStabilizer_HPP

#include <vector>
#include "opencv2/opencv.hpp"
#include "Stabilizer.hpp"

/**
 * @brief The PointStabilizer class implements a basic video stabilizer based on key point tracking.
 */
class PointStabilizer : public Stabilizer{
public:
    PointStabilizer(const cv::Mat& frame0, WarpingGroup mode=WarpingGroup::homography);

    virtual cv::Mat stabilize_next(const cv::Mat& next_frame);
    virtual cv::Mat visualization() const;

    const std::vector<cv::Point2f>& points0() const {
        return points0_;
    }

    const std::vector<cv::Point2f>& points() const {
        return points_;
    }

    const std::vector<bool>& status() const {
        return status_;
    }

    const std::vector<float>& trust() const {
        return trust_;
    }

private:
    cv::Mat last_frame_gray_;

    cv::Mat H_;
    cv::Mat frame_gray_0_;
    cv::Mat visualization_;
    WarpingGroup mode_ = WarpingGroup::homography;


    std::vector<cv::Point2f> points0_;
    std::vector<cv::Point2f> points_;
    std::vector<bool> status_;
    cv::Mat error_;

    unsigned long count_ = 1;
    std::vector<float> trust_;

    std::vector<cv::Point2f> checked_optical_flow_(const cv::Mat& frame_gray, float eps);
    cv::Mat get_next_homography_(const cv::Mat& next_image);
    void create_visualization_();
};

#endif // PointStabilizer_HPP

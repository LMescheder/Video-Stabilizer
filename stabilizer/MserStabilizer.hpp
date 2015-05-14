#ifndef MATMserStabilizer_HPP
#define MATMserStabilizer_HPP

#include "mser_tools/MatMserTracker.hpp"
#include "Stabilizer.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

class MserStabilizer : public Stabilizer {
public:
    using ComponentStats = MatMserTracker::ComponentStats;

    MserStabilizer(MatMser mser_detector, cv::Mat template_frame, Mode mode=homography)
        : Stabilizer(template_frame, mode),
          detector_{mser_detector}, tracker_{mser_detector}, count_{0} {
        recompute_msers_(frame_gray_0_);
    }

    std::vector<ComponentStats> msers() {
        return tracker_.msers();
    }

    const std::vector<cv::Point2f>& points() const {
        return points_;
    }

    const std::vector<cv::Point2f>& points0() const {
        return points0_;
    }

private:
    MatMserTracker tracker_;
    MatMser detector_;
    //cv::Mat frame_gray_0_;

    std::vector<ComponentStats> up_msers_0_;
    std::vector<ComponentStats> down_msers_0_;
    unsigned int count_;
    unsigned int recompute_T_ = 50;

    cv::Mat get_next_homography_(const cv::Mat& next_image);
    void create_visualization_();

    void recompute_msers_(cv::Mat image);
    void extract_points_(std::vector<cv::Point2f>& points, const ComponentStats& comp );

    std::vector<cv::Point2f> points_, points0_;

    Mode mode_ = homography;
};

#endif // MATMserStabilizer_HPP

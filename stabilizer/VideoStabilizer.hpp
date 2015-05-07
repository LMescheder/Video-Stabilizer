#ifndef MATVIDEOSTABILIZER_HPP
#define MATVIDEOSTABILIZER_HPP

#include "mser_tools/MatMserTracker.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

class VideoStabilizer
{
public:
    using ComponentStats = MatMserTracker::ComponentStats;

    enum Mode {
        homography,
        affine,
        rigid
    };

    VideoStabilizer(MatMser mser_detector, cv::Mat image0)
        : detector_{mser_detector}, tracker_{mser_detector}, H_(cv::Mat::eye(3, 3, CV_64FC1)), count_{0} {
        cv::Mat gray;
        cv::cvtColor(image0, gray, CV_BGR2GRAY);

        recompute_msers_(gray);
    }

    cv::Mat stabilze_next(cv::Mat next_image);

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
    cv::Mat gray0_;

    std::vector<ComponentStats> up_msers_0_;
    std::vector<ComponentStats> down_msers_0_;
    cv::Mat H_;
    unsigned int count_;
    unsigned int recompute_T_ = 50;

    void recompute_msers_(cv::Mat image);
    void extract_points_(std::vector<cv::Point2f>& points, const ComponentStats& comp );

    std::vector<cv::Point2f> points_, points0_;

    Mode mode_ = homography;
};

#endif // MATVIDEOSTABILIZER_HPP

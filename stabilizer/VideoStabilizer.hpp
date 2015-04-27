#ifndef MATVIDEOSTABILIZER_HPP
#define MATVIDEOSTABILIZER_HPP

#include "MatMserTracker.hpp"
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

    VideoStabilizer(MatMserTracker tracker, cv::Mat image0)
        : tracker_{tracker}, H0_(cv::Mat::eye(3, 3, CV_64FC1)), count_{0} {
        cv::Mat gray;
        cv::cvtColor(image0, gray, CV_BGR2GRAY);

        recompute_msers_(gray);
    }

    cv::Mat stabilze_next(cv::Mat next_image);

    std::vector<ComponentStats> msers() {
        return tracker_.msers();
    }

private:
    MatMserTracker tracker_;
    std::vector<ComponentStats> msers_0_;
    cv::Mat H0_;
    unsigned int count_;
    unsigned int recompute_T_ = 10;

    void recompute_msers_(cv::Mat image);
    void extract_points_(std::vector<cv::Point2f>& points, const ComponentStats& comp );

    Mode mode_ = homography;
};

#endif // MATVIDEOSTABILIZER_HPP

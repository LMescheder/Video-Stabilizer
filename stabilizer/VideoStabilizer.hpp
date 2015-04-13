#ifndef MATVIDEOSTABILIZER_HPP
#define MATVIDEOSTABILIZER_HPP

#include "MatMserTracker.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

class VideoStabilizer
{
public:
    using ComponentStats = MatMserTracker::ComponentStats;


    VideoStabilizer(MatMserTracker tracker, cv::Mat image0)
        : tracker_{tracker}, H0_(cv::Mat::eye(3, 3, CV_64FC1)), count_{0} {
        cv::Mat gray;
        cv::cvtColor(image0, gray, CV_BGR2GRAY);

        recompute_msers_(gray);
    }

    cv::Mat stabilze_next(cv::Mat next_image) {
        cv::Mat gray;
        cv::cvtColor(next_image, gray, CV_BGR2GRAY);

        tracker_.update(gray);
        cv::Mat H = cv::findHomography(MatMser::extract_means(tracker_.msers()),
                                       MatMser::extract_means(tracker_.msers_0()));
        H = H0_ * H;
        cv::Mat stabilized;
        cv::warpPerspective(next_image, stabilized, H, cv::Size(next_image.cols, next_image.rows));

        ++count_;
        if (count_ % recompute_T_ == 0) {
           recompute_msers_(gray);
           H0_ = H.clone();
        }

        return stabilized;
    }

    std::vector<ComponentStats> msers() {
        return tracker_.msers();
    }

private:
    MatMserTracker tracker_;
    std::vector<ComponentStats> msers_0_;
    cv::Mat H0_;
    unsigned int count_;
    unsigned int recompute_T_ = 20;

    void recompute_msers_(cv::Mat image) {
        tracker_.reset();
        // TODO: merge this into reset
        tracker_.update(image);
        msers_0_ = tracker_.msers();
    }
};

#endif // MATVIDEOSTABILIZER_HPP

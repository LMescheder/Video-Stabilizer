#ifndef MATMSERTRACKER_HPP
#define MATMSERTRACKER_HPP

#include <vector>
#include "opencv2/opencv.hpp"

#include "MatMser.hpp"

class MatMserTracker
{
public:
    using ComponentStats = MatMser::ComponentStats;

    MatMserTracker (MatMser mser_detector)
        : mser_detector_(mser_detector) {}

    void update (const cv::Mat& image);

    const std::vector<ComponentStats>& up_msers () const {
        return up_msers_;
    }

    const std::vector<ComponentStats>& down_msers () const {
        return down_msers_;
    }

    std::vector<ComponentStats> msers () const;

    const std::vector<ComponentStats>& up_msers_0 () const {
        return up_msers_0_;
    }

    const std::vector<ComponentStats>& down_msers_0 () const {
        return down_msers_0_;
    }

    std::vector<ComponentStats> msers_0 () const;

    const std::vector<cv::Point2f>& up_means () const {
        return up_means_;
    }

    const std::vector<cv::Point2f>& down_means () const {
        return down_means_;
    }
private:

    MatMser mser_detector_;
    int count_ = 0;
    std::vector<ComponentStats> up_msers_;
    std::vector<ComponentStats> down_msers_;
    cv::Mat last_image_;

    std::vector<ComponentStats> up_msers_0_;
    std::vector<ComponentStats> down_msers_0_;

    cv::Size lk_window_ = cv::Size(40, 40);

    // TODO: delete
    std::vector<cv::Point2f> up_means_;
    std::vector<cv::Point2f> down_means_;



    void track_means_(std::vector<ComponentStats>& msers, std::vector<cv::Point2f>& means,
                      const cv::Mat& new_image);
};

#endif // MATMSERTRACKER_HPP

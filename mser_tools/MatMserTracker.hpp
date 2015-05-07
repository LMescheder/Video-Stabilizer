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

    void reset() {

    }

    /**
     * @brief Tracks msers.
     * @param old_image
     * @param new_image
     * @param Msers to be tracked.
     * @return The tracked msers and information if the region could be found.
     */

    std::vector<MatMserTracker::ComponentStats> track (const cv::Mat& old_image, const cv::Mat& new_image, const std::vector<ComponentStats>& msers, bool reverse=false);

    const std::vector<ComponentStats>& up_msers () const {
        return up_msers_;
    }

    const std::vector<ComponentStats>& down_msers () const {
        return down_msers_;
    }

    std::vector<ComponentStats> msers () const;


    const std::vector<cv::Point2f>& up_means () const {
        return up_means_;
    }

    const std::vector<cv::Point2f>& down_means () const {
        return down_means_;
    }
private:

    MatMser mser_detector_;

    std::vector<ComponentStats> up_msers_;
    std::vector<ComponentStats> down_msers_;


    cv::Size lk_window_ = cv::Size(40, 40);

    // TODO: delete
    std::vector<cv::Point2f> up_means_;
    std::vector<cv::Point2f> down_means_;



    void track_msers_(const cv::Mat& old_image, const cv::Mat& new_image, std::vector<ComponentStats>& msers);
    std::tuple<std::vector<cv::Point2f>, std::vector<bool>> checked_optical_flow(const cv::Mat& old_image, const cv::Mat& new_image, const std::vector<cv::Point2f>& points, float eps=1.);

};

#endif // MATMSERTRACKER_HPP

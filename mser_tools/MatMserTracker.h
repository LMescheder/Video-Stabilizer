#ifndef MATMSERTRACKER_HPP
#define MATMSERTRACKER_HPP

#include <vector>
#include "opencv2/opencv.hpp"

#include "MatMser.h"

/**
 * @brief Class to track msers.
 *
 * This class builds on MatMser to track msers over a sequence of images.
 */
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
     * @param old_image               The last image from where the msers should be tracked.
     * @param new_image               The new image to which the msers should be tracked.
     * @param msers                   The msers to be tracked.
     * @return The tracked msers and information if the region could be found.
     */
    std::vector<MatMserTracker::ComponentStats> track (const cv::Mat& old_image, const cv::Mat& new_image, const std::vector<ComponentStats>& msers, bool reverse=false);

private:

    MatMser mser_detector_;
    cv::Size lk_window_ = cv::Size(40, 40);


     /**
     * @brief Estimate new ComponentStats of the msers using optical flow.
     * @param old_image               The last image from where the msers should be tracked.
     * @param new_image               The new image to which the msers should be tracked.
     * @param msers                   The msers to be tracked. Will be overwritten.
     */
    void track_msers_(const cv::Mat& old_image, const cv::Mat& new_image, std::vector<ComponentStats>& msers);

    /**
     * @brief Compute two directional optical flow using the Lucas Kanad algorithm.
     *
     * This function first tracks point by optical flow from old_image to new_image and then tracks the
     * new points back from new_image to old_image. Only if the error from the original points to the so obtained
     * points is lower than a certain threshold `eps` their status is set to `true`.
     * @param old_image               The last image from where the points should be tracked.
     * @param new_image               The new image to which the points should be tracked.
     * @param points                  The points to be tracked.
     * @param eps                     The error threshold.
     * @return                        The tracked points and a vector indicating their status.
     */
    std::tuple<std::vector<cv::Point2f>, std::vector<bool>> checked_optical_flow(const cv::Mat& old_image, const cv::Mat& new_image, const std::vector<cv::Point2f>& points, float eps=1.);

};

#endif // MATMSERTRACKER_HPP

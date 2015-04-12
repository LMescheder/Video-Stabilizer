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

    void update (cv::Mat image) {
        if (count_ == 0) {
            up_msers_ = mser_detector_.detect_msers(image, MatMser::upwards);
            down_msers_ = mser_detector_.detect_msers(image, MatMser::downwards);
        } else {
            up_msers_ = mser_detector_.retrieve_msers(image, up_msers_, false);
            down_msers_ = mser_detector_.retrieve_msers(image, down_msers_, true);
        }

        ++count_;
        last_image_ = image;
    }

    const std::vector<ComponentStats>& up_msers () const {
        return up_msers_;
    }

    const std::vector<ComponentStats>& down_msers () const {
        return down_msers_;
    }

private:

    MatMser mser_detector_;
    int count_ = 0;
    std::vector<ComponentStats> up_msers_;
    std::vector<ComponentStats> down_msers_;
    cv::Mat last_image_;
};

#endif // MATMSERTRACKER_HPP

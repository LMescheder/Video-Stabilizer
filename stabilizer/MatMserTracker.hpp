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

    void update (const cv::Mat& image) {
        if (count_ == 0) {
            up_msers_ = mser_detector_.detect_msers(image, MatMser::upwards);
            up_means_ = MatMser::extract_means(up_msers_);
            down_msers_ = mser_detector_.detect_msers(image, MatMser::downwards);
            down_means_ = MatMser::extract_means(down_msers_);
        } else {
            track_means_(up_msers_, up_means_, image);
            track_means_(down_msers_, down_means_, image);
            up_msers_ = mser_detector_.retrieve_msers(image, up_msers_, false);
            down_msers_ = mser_detector_.retrieve_msers(image, down_msers_, true);
        }

        ++count_;
        last_image_ = image.clone();
    }

    const std::vector<ComponentStats>& up_msers () const {
        return up_msers_;
    }

    const std::vector<ComponentStats>& down_msers () const {
        return down_msers_;
    }

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

    std::vector<cv::Point2f> up_means_;
    std::vector<cv::Point2f> down_means_;

    void track_means_(std::vector<ComponentStats>& msers, std::vector<cv::Point2f>& means,
                      const cv::Mat& new_image) {

        means = MatMser::extract_means(msers);
        std::vector<cv::Point2f> new_means(means.size());
        cv::Mat err, status;

        cv::calcOpticalFlowPyrLK(last_image_, new_image, means, new_means, status, err);
        means = new_means;

        for (std::size_t i=0; i<msers.size(); ++i) {
            cv::Point2f t = new_means[i] - msers[i].mean;
            cv::Point2i tint{static_cast<int>(t.x), static_cast<int>(t.y)};
            msers[i].mean += t;
            msers[i].min_point = MatMser::clamp(msers[i].min_point + tint, new_image);
            msers[i].max_point += MatMser::clamp(msers[i].max_point + tint, new_image);
            msers[i].source += MatMser::clamp(msers[i].source + tint, new_image);
        }
    }
};

#endif // MATMSERTRACKER_HPP

#include "MatMserTracker.hpp"



void MatMserTracker::update(const cv::Mat &image) {
    if (count_ == 0) {
        up_msers_ = mser_detector_.detect_msers(image, MatMser::upwards);
        up_msers_0_ = up_msers_;
        down_msers_ = mser_detector_.detect_msers(image, MatMser::downwards);
        down_msers_0_ = down_msers_;

    } else {
        track_means_(up_msers_, up_means_, image);
        track_means_(down_msers_, down_means_, image);
        up_msers_ = mser_detector_.retrieve_msers(image, up_msers_, false);
        down_msers_ = mser_detector_.retrieve_msers(image, down_msers_, true);
    }

    ++count_;
    last_image_ = image.clone();
}

std::vector<MatMserTracker::ComponentStats> MatMserTracker::msers() const {
    std::vector<ComponentStats> result;
    result.reserve(up_msers_.size() + down_msers_.size());
    for (auto& m : up_msers_)
        if (m.N > 0)
            result.push_back(m);
    for (auto& m : down_msers_)
        if (m.N > 0)
            result.push_back(m);
    return result;
}

std::vector<MatMserTracker::ComponentStats> MatMserTracker::msers_0() const {
    std::vector<ComponentStats> result;
    result.reserve(up_msers_0_.size() + down_msers_0_.size());
    for (std::size_t i = 0; i < up_msers_.size(); ++i)
        if (up_msers_[i].N > 0)
            result.push_back(up_msers_0_[i]);
    for (std::size_t i = 0; i < down_msers_.size(); ++i)
        if (down_msers_[i].N > 0)
            result.push_back(down_msers_0_[i]);
    return result;
}

void MatMserTracker::track_means_(std::vector<MatMserTracker::ComponentStats> &msers, std::vector<cv::Point2f> &means, const cv::Mat &new_image) {

    means = MatMser::extract_means(msers);
    std::vector<cv::Point2f> new_means(means.size());
    cv::Mat err, status;

    cv::calcOpticalFlowPyrLK(last_image_, new_image, means, new_means, status, err, lk_window_);
    means = new_means;

    for (std::size_t i=0; i<msers.size(); ++i) {
        cv::Point2f t = new_means[i] - msers[i].mean;
        cv::Point2i tint{static_cast<int>(t.x), static_cast<int>(t.y)};
        msers[i].mean = msers[i].mean + t;
        msers[i].min_point = MatMser::clamp(msers[i].min_point + tint, new_image);
        msers[i].max_point = MatMser::clamp(msers[i].max_point + tint, new_image);
        msers[i].source =    MatMser::clamp(msers[i].source + tint, new_image);
    }
}

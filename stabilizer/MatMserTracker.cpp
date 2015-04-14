#include "MatMserTracker.hpp"



void MatMserTracker::update(const cv::Mat &image) {
    if (count_ == 0) {
        up_msers_ = mser_detector_.detect_msers(image, MatMser::upwards);
        up_msers_0_ = up_msers_;
        down_msers_ = mser_detector_.detect_msers(image, MatMser::downwards);
        down_msers_0_ = down_msers_;

    } else {
        track_msers_(up_msers_, up_means_, image);
        track_msers_(down_msers_, down_means_, image);
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

void MatMserTracker::track_msers_(std::vector<MatMserTracker::ComponentStats> &msers, std::vector<cv::Point2f> &means, const cv::Mat &new_image) {

    std::vector<cv::Point2i> points;
    std::vector<cv::Point2i> hull0;
    std::vector<cv::Point2f> hull0f;
    std::vector<cv::Point2f> hull1f;
    cv::Mat err, status;

    for (auto& mser : msers) {
        if (mser.N == 0)
            continue;
        points = MatMser::stats_to_points(mser, last_image_);
        cv::convexHull(points, hull0);

        hull0f.reserve(10);
        int stepsize = hull0.size() <= 10 ? 1 : hull0.size() / 10;
        for (int i = 0; i < 10; ++i) {
            auto& p = hull0[i * stepsize];
            hull0f.emplace_back(p.x, p.y);
        }

        cv::calcOpticalFlowPyrLK(last_image_, new_image, hull0f, hull1f, status, err, lk_window_);
        cv::Matx23f Ab = cv::estimateRigidTransform(hull0f, hull1f, true);

        // update mser
        mser.mean.x = Ab(0, 0) * mser.mean.x +  Ab(0, 1) * mser.mean.y + Ab(0, 2);
        mser.mean.y = Ab(1, 0) * mser.mean.x +  Ab(1, 1) * mser.mean.y + Ab(1, 2);
    }


}

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

    const int max_N = 100;

    std::vector<cv::Point2f> points0;
    std::vector<cv::Point2f> points1;

    std::vector<int> hull_Ns;

    std::vector<cv::Point2i> mser_points;
    std::vector<cv::Point2i> hull_points;

    hull_Ns.reserve(msers.size());
    points0.reserve(max_N * msers.size());

    // compute 10 points of convex hull
    for (auto& mser : msers) {
        if (mser.N != 0) {
            mser_points = MatMser::stats_to_points(mser, last_image_);
            cv::convexHull(mser_points, hull_points);
            int stepsize = 0;
            int number = 0;
            if (hull_points.size() <= max_N) {
                stepsize = 1;
                number = hull_points.size();
            } else {
                stepsize = hull_points.size() / max_N;
                number = 10;
            }
            hull_Ns.push_back(number);

            for (int i = 0; i < number; ++i) {
                auto& p = hull_points[i * stepsize];
                points0.emplace_back(p.x, p.y);
            }
        }
    }

    // compute optical flow on points
    assert(points0.size() > 0);
    cv::Mat err, status;
    points1.reserve(points0.size());
    cv::calcOpticalFlowPyrLK(last_image_, new_image, points0, points1, status, err, lk_window_);

    cv::Mat points0_mat(points0);
    cv::Mat points1_mat(points1);

    assert(points0.size() == points1.size());
    /*
    int j1 = 0;
    int j2 = 0;
    for (std::size_t i=0; i<msers.size(); ++i) {
        j1 = j2;
        j2 += hull_Ns[i];
        cv::Matx23f Ab = cv::estimateRigidTransform(points0_mat(cv::Range(j1, j2), cv::Range()),
                                                    points1_mat(cv::Range(j1, j2), cv::Range()),
                                                    true);

        // update mser
        auto& mser = msers[i];
        mser.mean.x = Ab(0, 0) * mser.mean.x +  Ab(0, 1) * mser.mean.y + Ab(0, 2);
        mser.mean.y = Ab(1, 0) * mser.mean.x +  Ab(1, 1) * mser.mean.y + Ab(1, 2);
    }
    */
}

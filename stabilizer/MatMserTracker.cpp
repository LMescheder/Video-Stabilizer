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
        } else {
            hull_Ns.push_back(0);
        }
    }

    // compute optical flow on points
    //assert(points0.size() > 0);
    if (points0.size() == 0)
        return;

    cv::Mat err, status;
    points1.reserve(points0.size());
    cv::calcOpticalFlowPyrLK(last_image_, new_image, points0, points1, status, err, lk_window_, 3);

    assert(points0.size() == points1.size());
    assert(msers.size() == hull_Ns.size());

    int j1 = 0;
    int j2 = 0;
    for (std::size_t i=0; i<msers.size(); ++i) {
        if (hull_Ns[i] == 0)
            continue;

        auto& mser = msers[i];
        j1 = j2;
        j2 += hull_Ns[i];

        // get points
        std::vector<cv::Point2f> mser_points0, mser_points1;
        mser_points0.reserve(hull_Ns[i]);
        mser_points1.reserve(hull_Ns[i]);

        for (int j=j1; j<j2; ++j) {
            if (status.at<bool>(j)) {
                mser_points0.push_back(points0[j]);
                mser_points1.push_back(points1[j]);
            }
        }

        if (mser_points0.size() < 3) {
            mser.N = 0;
            continue;
        }

        // estimate affine transformation
        cv::Mat_<float> A = cv::estimateRigidTransform(mser_points0, mser_points1, true);


        // update mser


        if (A.empty()) {
            mser.N = 0;
            continue;
        }

        cv::Point2f new_mean;
        cv::Matx22f new_cov;
        cv::Point2i new_source;
        cv::Point2i new_min_point;
        cv::Point2i new_max_point;
        int new_N;

        new_mean.x = A(0, 0) * mser.mean.x + A(0, 1) * mser.mean.y + A(0, 2);
        new_mean.y = A(1, 0) * mser.mean.x + A(1, 1) * mser.mean.y + A(1, 2);

        for (int idx1 : {0, 1}) for (int idx2 : {0, 1})
        for (int idx3 : {0, 1}) for (int idx4 : {0, 1})
            new_cov(idx1, idx3) += A(idx1, idx2) * A(idx3, idx4) * mser.cov(idx2, idx4);

        new_source.x = static_cast<int>(A(0, 0) * mser.source.x + A(0, 1) * mser.source.y + A(0, 2));
        new_source.y = static_cast<int>(A(1, 0) * mser.source.x + A(1, 1) * mser.source.y + A(1, 2));

        cv::Point2i new_corner11, new_corner12, new_corner21, new_corner22;

        new_corner11.x = static_cast<int>(A(0, 0) * mser.min_point.x + A(0, 1) * mser.min_point.y + A(0, 2));
        new_corner11.y = static_cast<int>(A(1, 0) * mser.min_point.x + A(1, 1) * mser.min_point.y + A(1, 2));

        new_corner12.x = static_cast<int>(A(0, 0) * mser.min_point.x + A(0, 1) * mser.max_point.y + A(0, 2));
        new_corner12.y = static_cast<int>(A(1, 0) * mser.min_point.x + A(1, 1) * mser.max_point.y + A(1, 2));

        new_corner21.x = static_cast<int>(A(0, 0) * mser.max_point.x + A(0, 1) * mser.min_point.y + A(0, 2));
        new_corner21.y = static_cast<int>(A(1, 0) * mser.max_point.x + A(1, 1) * mser.min_point.y + A(1, 2));

        new_corner22.x = static_cast<int>(A(0, 0) * mser.max_point.x + A(0, 1) * mser.max_point.y + A(0, 2));
        new_corner22.y = static_cast<int>(A(1, 0) * mser.max_point.x + A(1, 1) * mser.max_point.y + A(1, 2));

        new_min_point.x = std::min({new_corner11.x, new_corner12.x, new_corner21.x, new_corner22.x});
        new_min_point.y = std::min({new_corner11.y, new_corner12.y, new_corner21.y, new_corner22.y});

        new_max_point.x = std::max({new_corner11.x, new_corner12.x, new_corner21.x, new_corner22.x});
        new_max_point.y = std::max({new_corner11.y, new_corner12.y, new_corner21.y, new_corner22.y});

        new_N = static_cast<int> ((A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0)) * mser.N);

        mser.mean = new_mean;
        mser.cov = new_cov;
        mser.source = new_source;
        mser.min_point = new_min_point;
        mser.max_point = new_max_point;
        mser.N = new_N;
    }


}





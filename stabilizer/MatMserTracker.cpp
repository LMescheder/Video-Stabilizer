#include "MatMserTracker.hpp"



void MatMserTracker::update(const cv::Mat &image) {
    if (count_ == 0) {
        up_msers_ = mser_detector_.detect_msers(image, MatMser::upwards);
        up_msers_0_ = up_msers_;
        down_msers_ = mser_detector_.detect_msers(image, MatMser::downwards);
        down_msers_0_ = down_msers_;

    } else {
        track_msers_(up_msers_, image);
        track_msers_(down_msers_, image);
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
        result.push_back(m);
    for (auto& m : down_msers_)
        result.push_back(m);
    return result;
}


void MatMserTracker::track_msers_(std::vector<MatMserTracker::ComponentStats> &msers, const cv::Mat &new_image) {

    const int max_N = 100;

    std::vector<cv::Point2f> points0;
    std::vector<cv::Point2f> points1;
    std::vector<bool> status;

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
                number = max_N;
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

    std::tie(points1, status) = checked_optical_flow(new_image, points0);

    assert(points0.size() == points1.size());
    assert(msers.size() == hull_Ns.size());

    // point range corresponding to current mser
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
            if (status[j]) {
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

        mser.transform_affine(A);
    }
    assert(j2 == points0.size());

}

std::tuple<std::vector<cv::Point2f>, std::vector<bool> > MatMserTracker::checked_optical_flow(const cv::Mat& image, const std::vector<cv::Point2f>& points, float eps) {
    cv::Mat err;
    cv::Mat status1, status2;
    std::vector<cv::Point2f> points1, points2;
    std::vector<bool> status;
    points1.reserve(points.size());
    points2.reserve(points.size());
    status.resize(points.size(), true);

    cv::calcOpticalFlowPyrLK(last_image_, image, points, points1, status1, err, lk_window_, 3);
    cv::calcOpticalFlowPyrLK(image, last_image_, points1, points2, status2, err, lk_window_, 3);

    for (std::size_t i=0; i<points.size(); ++i) {
        status[i] = (status1.at<bool>(i) && status2.at<bool>(i) && cv::norm(points[i] - points2[i]) < eps );
    }

    return std::make_tuple(points1, status);
}





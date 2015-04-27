#include "VideoStabilizer.hpp"

cv::Mat VideoStabilizer::stabilze_next(cv::Mat next_image) {
    cv::Mat gray;
    cv::cvtColor(next_image, gray, CV_BGR2GRAY);

    tracker_.update(gray);

    auto msers = tracker_.msers();
    assert(msers.size() == msers_0_.size());
    points_.clear();
    points0_.clear();
    for (std::size_t i=0; i<msers.size(); ++i)
        if (msers[i].N > 0) {
            extract_points_(points_, msers[i]);
            extract_points_(points0_, msers_0_[i]);
        }

    cv::Mat H, A;

    switch (mode_) {
    case homography :
        H = cv::findHomography(points_, points0_);
        break;
    case affine :
        A = cv::estimateRigidTransform(points_, points0_, true);
        H = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H(cv::Range(0, 2), cv::Range(0, 3)));
        break;
    case rigid :
        A = cv::estimateRigidTransform(points_, points0_, false);
        H = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H(cv::Range(0, 2), cv::Range(0, 3)));
        break;
    }


    H = H0_ * H;
    cv::Mat stabilized;
    cv::warpPerspective(next_image, stabilized, H, cv::Size(next_image.cols, next_image.rows));

    ++count_;
    if (count_ % recompute_T_ == 0) {
        recompute_msers_(gray);
        H0_ = H.clone();
    }

    return stabilized;
}

void VideoStabilizer::recompute_msers_(cv::Mat image) {
    tracker_.reset();
    // TODO: merge this into reset
    tracker_.update(image);
    msers_0_ = tracker_.msers();
    points_.reserve(msers_0_.size());
    points0_.reserve(msers_0_.size());
}

void VideoStabilizer::extract_points_(std::vector<cv::Point2f> &points, const VideoStabilizer::ComponentStats &comp) {

    // affinitely invariant coordinate system

    // cholesky decomposition
    cv::Matx22f D;
    D(0, 0) = std::sqrt(comp.cov(0, 0));
    D(0, 1) = 0.;
    D(1, 0) = comp.cov(0, 1)/D(0, 0);
    D(1, 1) = std::sqrt(comp.cov(1, 1)- D(1, 0)*D(1, 0));

    // rotation corresponding to orientation
    cv::Matx22f R;
    cv::Vec2f orient(std::cos(comp.angle), std::sin(comp.angle));;
    cv::Vec2f r = D.inv() * orient;
    r = r/cv::norm(r);
                             ;
    R(0, 0) = r(0);
    R(1, 0) = r(1);
    R(0, 1) = -r(1);
    R(1, 1) = r(0);


    // affinitely invariant coordinate system
    cv::Matx22f N = D * R;

    cv::Point2f d1(N(0, 0), N(1, 0));
    cv::Point2f d2(N(0, 1), N(1, 1));

    float eps = 1.f;
    points.push_back(comp.mean);
    points.push_back(comp.mean + eps * d1);
    points.push_back(comp.mean + eps * d2);
    points.push_back(comp.mean - eps * d1);
    points.push_back(comp.mean - eps * d2);
}


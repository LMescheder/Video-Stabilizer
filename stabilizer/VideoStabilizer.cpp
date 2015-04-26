#include "VideoStabilizer.hpp"

cv::Mat VideoStabilizer::stabilze_next(cv::Mat next_image) {
    cv::Mat gray;
    cv::cvtColor(next_image, gray, CV_BGR2GRAY);

    tracker_.update(gray);

    std::vector<cv::Point2f> points, points0;\
    auto msers = tracker_.msers();
    assert(msers.size() == msers_0_.size());
    points.reserve(5*msers.size());
    points0.reserve(5*msers.size());
    for (std::size_t i=0; i<msers.size(); ++i)
        if (msers[i].N > 0) {
            extract_points_(points, msers[i]);
            extract_points_(points0, msers_0_[i]);
        }

    cv::Mat H = cv::findHomography(points, points0);

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
    r = r/std::sqrt(r(0)*r(0) + r(1)*r(1));
                             ;
    R(0, 0) = r(0);
    R(1, 0) = r(1);
    R(0, 1) = -r(1);
    R(1, 1) = r(0);


    // affinitely invariant coordinate system
    cv::Matx22f N = D * R;

    cv::Point2f d1(N(0, 0), N(1, 0));
    cv::Point2f d2(N(0, 1), N(1, 1));

    float eps = .1f;
    points.push_back(comp.mean);
    points.push_back(comp.mean + eps * d1);
    points.push_back(comp.mean + eps * d2);
    points.push_back(comp.mean - eps * d1);
    points.push_back(comp.mean - eps * d2);
}

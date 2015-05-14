#include "MserStabilizer.hpp"
#include "utilities.hpp"

MserStabilizer::MserStabilizer(MatMser mser_detector, cv::Mat frame_0, WarpingGroup mode)
    : detector_{mser_detector}, tracker_{mser_detector}, count_{0} {
    H_ = cv::Mat::eye(3, 3, CV_64FC1);
    cv::cvtColor(frame_0, frame_gray_0_, CV_BGR2GRAY);
    recompute_msers_(frame_gray_0_);
}

cv::Mat MserStabilizer::stabilize_next(const cv::Mat& next_frame) {
    cv::Mat frame_gray, H_frame_gray;
    cv::cvtColor(next_frame, frame_gray, CV_BGR2GRAY);


    cv::warpPerspective(frame_gray, H_frame_gray, H_, cv::Size(frame_gray.cols, frame_gray.rows));

    cv::Mat dH = get_next_homography_(H_frame_gray);

    visualization_ = next_frame.clone();
    create_visualization_();

    H_ = dH * H_;

    cv::Mat stabilized_frame;
    cv::warpPerspective(next_frame, stabilized_frame, H_, cv::Size(next_frame.cols, next_frame.rows));

    return stabilized_frame;
}

cv::Mat MserStabilizer::visualization() const {
    return visualization_;
}



cv::Mat MserStabilizer::get_next_homography_(const cv::Mat& H_gray) {
    std::vector<ComponentStats> up_msers, down_msers;

   up_msers = tracker_.track(frame_gray_0_, H_gray, up_msers_0_);
   down_msers = tracker_.track(frame_gray_0_, H_gray, down_msers_0_, true);


    points_.clear();
    points0_.clear();
    // TODO: add trust checking
    for (std::size_t i=0; i<up_msers_0_.size(); ++i)
        if (up_msers[i].N > 0) {
            extract_points_(points_, up_msers[i]);
            extract_points_(points0_, up_msers_0_[i]);
        }

    for (std::size_t i=0; i<down_msers_0_.size(); ++i)
        if (down_msers[i].N > 0) {
            extract_points_(points_, down_msers[i]);
            extract_points_(points0_, down_msers_0_[i]);
        }

    return find_homography(points_, points0_, mode_);
}

void MserStabilizer::create_visualization_() {

}

void MserStabilizer::recompute_msers_(cv::Mat image) {
    //tracker_.reset();
    // TODO: merge this into reset
    frame_gray_0_ = image;
    up_msers_0_ = detector_.detect_msers(image, MatMser::upwards);
    down_msers_0_ = detector_.detect_msers(image, MatMser::downwards);

//    msers_0_ = tracker_.msers();
//    points_.reserve(msers_0_.size());
//    points0_.reserve(msers_0_.size());
}

void MserStabilizer::extract_points_(std::vector<cv::Point2f> &points, const MserStabilizer::ComponentStats &comp) {

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


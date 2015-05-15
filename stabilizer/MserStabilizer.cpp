#include "MserStabilizer.hpp"
#include "utilities.hpp"

MserStabilizer::MserStabilizer(MatMser mser_detector, cv::Mat frame_0, WarpingGroup mode, VisualizationFlags visualization_flags)
    : detector_{mser_detector}, tracker_{mser_detector}, count_{0}, visualization_flags_(visualization_flags) {
    H_ = cv::Mat::eye(3, 3, CV_64FC1);
    cv::cvtColor(frame_0, frame_gray_0_, CV_BGR2GRAY);
    recompute_msers_(frame_gray_0_);
}

cv::Mat MserStabilizer::stabilize_next(const cv::Mat& next_frame) {
    cv::cvtColor(next_frame, frame_gray_, CV_BGR2GRAY);

    cv::warpPerspective(frame_gray_, H_frame_gray_, H_, cv::Size(frame_gray_.cols, frame_gray_.rows));

    cv::Mat dH = get_next_homography_(H_frame_gray_);

    if (visualize_)
        create_visualization_(next_frame);
    else
        visualization_.release();

    H_ = dH * H_;

    cv::Mat stabilized_frame;
    cv::warpPerspective(next_frame, stabilized_frame, H_, cv::Size(next_frame.cols, next_frame.rows));

    return stabilized_frame;
}

cv::Mat MserStabilizer::visualization() const {
    return visualization_;
}

std::vector<MserStabilizer::ComponentStats> MserStabilizer::msers() {
    std::vector<ComponentStats> result;
    result.reserve(up_msers_.size() + down_msers_.size());
    for (auto& m : up_msers_)
        result.push_back(m);
    for (auto& m : down_msers_)
        result.push_back(m);
    return result;
}



cv::Mat MserStabilizer::get_next_homography_(const cv::Mat& H_gray) {
   up_msers_ = tracker_.track(frame_gray_0_, H_gray, up_msers_0_);
   down_msers_ = tracker_.track(frame_gray_0_, H_gray, down_msers_0_, true);


    points_.clear();
    points0_.clear();
    // TODO: add trust checking
    for (std::size_t i=0; i<up_msers_0_.size(); ++i)
        if (up_msers_[i].N > 0) {
            extract_points_(points_, up_msers_[i]);
            extract_points_(points0_, up_msers_0_[i]);
        }

    for (std::size_t i=0; i<down_msers_0_.size(); ++i)
        if (down_msers_[i].N > 0) {
            extract_points_(points_, down_msers_[i]);
            extract_points_(points0_, down_msers_0_[i]);
        }

    return find_homography(points_, points0_, mode_);
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

void MserStabilizer::create_visualization_(const cv::Mat& frame) {
    cv::Mat H_vis;
    cv::warpPerspective(frame, H_vis, H_, cv::Size(frame.cols, frame.rows));

    std::vector<ComponentStats> all_msers = msers();


   if (visualization_flags_ & visualize_hulls)
        visualize_regions_hulls_(H_vis, all_msers);
   if (visualization_flags_ & visualize_means)
        visualize_points(H_vis, all_msers, true);
   if (visualization_flags_ & visualize_stab_points)
        visualize_stabilization_points(H_vis, true);
   if (visualization_flags_ & visualize_cov)
        visualize_regions_cov(H_vis, all_msers);
   if (visualization_flags_ & visualize_boxes)
        visualize_regions_box(H_vis, all_msers);

   cv::warpPerspective(H_vis, visualization_, H_.inv(), cv::Size(frame.cols, frame.rows));
}
\


void MserStabilizer::visualize_points (cv::Mat& image, const std::vector<MatComponentStats>& msers, bool orientation) {
    for (auto& mser : msers)
        if (mser.N > 0) {
            cv::circle(image, mser.mean, 3, cv::Scalar(255, 255, 0));
            if (orientation) {
                cv::Point2f dir(std::cos(mser.angle), std::sin(mser.angle));
                cv::line(image, mser.mean, mser.mean + 10 * dir, cv::Scalar(255, 255, 0), 1.);
            }
        }
}


void MserStabilizer::visualize_stabilization_points (cv::Mat&  image, bool lines) {
    assert(points_.size() == points0_.size());
    for (std::size_t i=0; i<points_.size(); ++i) {
        cv::Point2f point = points_[i];
        cv::Point2f point0 =  points0_[i];
        cv::circle(image, point, 3, cv::Scalar(255, 0, 255));
        if (lines)
            cv::line(image, point, point0, cv::Scalar(255, 0, 255), 1.);
    }
}


void MserStabilizer::visualize_regions_hulls_ (cv::Mat& image, const std::vector<MatComponentStats>& msers){
    for (auto& mser : msers)
        if (mser.N > 0) {
            auto points = MatMser::stats_to_points(mser, H_frame_gray_);
            std::vector<cv::Point> hull;
            cv::convexHull(points, hull);
            cv::polylines(image, hull, true, cv::Scalar(0, 255, 0), 1.5);
        }
}


void MserStabilizer::visualize_regions_cov (cv::Mat& image, const std::vector<MatComponentStats>& msers){
    for (auto& mser : msers)
        if (mser.N > 0) {
            // compute eigenvalue and eigenvectors
            cv::Mat eigenvals, eigenvecs;
            cv::eigen(mser.cov, true, eigenvals, eigenvecs);

            // get angle of first eigenvector
            float angle = atan2(eigenvecs.at<float>(0,1), eigenvecs.at<float>(0,0));

            // convert angle to degrees
            angle = 180*angle/3.14159265359;

            // calculate axis length
            float axis1_length = 2*sqrt(eigenvals.at<float>(0));
            float axis2_length = 2*sqrt(eigenvals.at<float>(1));

            // draw ellipse
            cv::ellipse(image, mser.mean, cv::Size(axis1_length, axis2_length), angle, 0, 360, cv::Scalar(255, 0, 255), 1.5);

        }
}


void MserStabilizer::visualize_regions_box (cv::Mat& image, const std::vector<MatComponentStats>& msers){
    for (auto& mser : msers)
        if (mser.N > 0)
            cv::rectangle(image, mser.min_point, mser.max_point, cv::Scalar(0, 165, 255), 1.5);
}




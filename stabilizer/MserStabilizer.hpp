#ifndef MATMserStabilizer_HPP
#define MATMserStabilizer_HPP

#include "mser_tools/MatMserTracker.hpp"
#include "Stabilizer.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

class MserStabilizer : public Stabilizer {
public:
    using ComponentStats = MatMserTracker::ComponentStats;

    MserStabilizer(MatMser mser_detector, cv::Mat frame_0, WarpingGroup mode=WarpingGroup::homography);

    virtual cv::Mat stabilize_next(const cv::Mat& next_frame);
    virtual cv::Mat visualization() const;

    std::vector<ComponentStats> msers() {
        return tracker_.msers();
    }

    const std::vector<cv::Point2f>& points() const {
        return points_;
    }

    const std::vector<cv::Point2f>& points0() const {
        return points0_;
    }

private:
    MatMserTracker tracker_;
    MatMser detector_;
    //cv::Mat frame_gray_0_;

    cv::Mat H_;
    cv::Mat frame_gray_0_;
    cv::Mat frame;
    cv::Mat visualization_;
    cv::Mat frame_gray_, H_frame_gray_;

    WarpingGroup mode_ = WarpingGroup::homography;

    std::vector<ComponentStats> up_msers_0_;
    std::vector<ComponentStats> down_msers_0_;
    std::vector<ComponentStats> up_msers_, down_msers_;

    unsigned int count_;
    unsigned int recompute_T_ = 50;

    cv::Mat get_next_homography_(const cv::Mat& next_image);
    void create_visualization_();
    void visualize_stabilization_points (cv::Mat&  image, const std::vector<MatComponentStats>& msers, const std::vector<MatComponentStats>& msers0, bool lines);
    void visualize_regions_hulls_ (cv::Mat& image, const std::vector<MatComponentStats>& msers);
    void visualize_points (cv::Mat& image, const std::vector<MatComponentStats>& msers, bool orientation);

    void recompute_msers_(cv::Mat image);
    void extract_points_(std::vector<cv::Point2f>& points, const ComponentStats& comp );

    std::vector<cv::Point2f> points_, points0_;

};

#endif // MATMserStabilizer_HPP

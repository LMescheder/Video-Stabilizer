#ifndef MATMserStabilizer_HPP
#define MATMserStabilizer_HPP

#include "mser_tools/MatMserTracker.hpp"
#include "Stabilizer.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

class MserStabilizer : public Stabilizer {
public:
    using ComponentStats = MatMserTracker::ComponentStats;

    enum VisualizationFlags {
        visualize_hulls = 1 << 0,
        visualize_means = 1 << 1,
        visualize_cov = 1 << 2,
        visualize_boxes = 1 << 3,
        visualize_stab_points = 1 << 4
    };

    MserStabilizer(MatMser mser_detector, cv::Mat frame_0, WarpingGroup mode=WarpingGroup::homography,
                   VisualizationFlags vis_flags=static_cast<VisualizationFlags> (visualize_hulls | visualize_means));

    virtual cv::Mat stabilize_next(const cv::Mat& next_frame);
    virtual cv::Mat visualization() const;

    std::vector<ComponentStats> msers();

    const std::vector<cv::Point2f>& points() const {
        return points_;
    }

    const std::vector<cv::Point2f>& points0() const {
        return points0_;
    }

    bool visualize() const {
        return visualize_;
    }

    void set_visualize (bool visualize) {
        visualize_ = visualize;
    }

private:
    WarpingGroup mode_ = WarpingGroup::homography;
    bool visualize_ = true;
    VisualizationFlags visualization_flags_;

    MatMserTracker tracker_;
    MatMser detector_;
    //cv::Mat frame_gray_0_;

    cv::Mat H_;
    cv::Mat frame_gray_0_;
    cv::Mat frame;
    cv::Mat visualization_;
    cv::Mat frame_gray_, H_frame_gray_;


    std::vector<ComponentStats> up_msers_0_;
    std::vector<ComponentStats> down_msers_0_;
    std::vector<ComponentStats> up_msers_, down_msers_;

    unsigned int count_;
    unsigned int recompute_T_ = 50;

    cv::Mat get_next_homography_(const cv::Mat& next_image);
    void create_visualization_(const cv::Mat& frame);

    void visualize_regions_hulls_ (cv::Mat& image, const std::vector<MatComponentStats>& msers);
    void visualize_points (cv::Mat& image, const std::vector<MatComponentStats>& msers, bool orientation);
    void visualize_stabilization_points (cv::Mat&  image, bool lines);
    void visualize_regions_cov (cv::Mat& image, const std::vector<MatComponentStats>& msers);
    void visualize_regions_box (cv::Mat& image, const std::vector<MatComponentStats>& msers);

    void recompute_msers_(cv::Mat image);
    void extract_points_(std::vector<cv::Point2f>& points, const ComponentStats& comp );

    std::vector<cv::Point2f> points_, points0_;

};


// definition of the visualization flags
inline MserStabilizer::VisualizationFlags operator| (MserStabilizer::VisualizationFlags lhs, MserStabilizer::VisualizationFlags rhs) {
    return (MserStabilizer::VisualizationFlags)(static_cast<int>(lhs) | static_cast<int>(rhs));
}

inline MserStabilizer::VisualizationFlags& operator|= (MserStabilizer::VisualizationFlags& lhs, MserStabilizer::VisualizationFlags rhs) {
    lhs = (MserStabilizer::VisualizationFlags)(static_cast<int>(lhs) | static_cast<int>(rhs));
    return lhs;
}

inline MserStabilizer::VisualizationFlags operator& (MserStabilizer::VisualizationFlags lhs, MserStabilizer::VisualizationFlags rhs) {
    return (MserStabilizer::VisualizationFlags)(static_cast<int>(lhs) & static_cast<int>(rhs));
}

inline MserStabilizer::VisualizationFlags& operator&= (MserStabilizer::VisualizationFlags& lhs, MserStabilizer::VisualizationFlags rhs) {
    lhs = (MserStabilizer::VisualizationFlags)(static_cast<int>(lhs) & static_cast<int>(rhs));
    return lhs;
}

#endif // MATMserStabilizer_HPP

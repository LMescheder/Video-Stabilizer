#ifndef MATMserStabilizer_HPP
#define MATMserStabilizer_HPP

#include "mser_tools/MatMserTracker.h"
#include "Stabilizer.h"
#include "opencv2/opencv.hpp"
#include <vector>

/**
 * @brief The MserStabilizer implements a basic video stabilizer based on mser tracking
 */
class MserStabilizer : public Stabilizer {
public:
    using ComponentStats = MatMserTracker::ComponentStats;

    enum VisualizationFlags {
        VIS_HULLS = 1 << 0,
        VIS_MEANS = 1 << 1,
        VIS_COV = 1 << 2,
        VIS_BOXES = 1 << 3,
        VIS_STABPOINTS = 1 << 4
    };

public:
    /**
     * @brief Constructor of the MserStabilizer class.
     * @param mser_detector     A MatMser object used for detection and tracking of the msers.
     * @param frame_0           Template frame or first frame of the video sequence to compare against.
     * @param warping           What kind of underlying warping should be used for the stabilization.
     * @param warping_back      Should we first warp the current frame with the previous homography and directly
     *                          compare against the first frame/the template frame?
     * @param vis_flags         What aspects of the msers are going to be visualized.
     */
    MserStabilizer(MatMser mser_detector, cv::Mat frame_0,
                   Warping warping=Warping::HOMOGRAPHY, Mode mode=Mode::TRACK_REF,
                   int vis_flags= (VIS_HULLS | VIS_MEANS));

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

protected:
    virtual void track_ref();

    virtual cv::Mat get_next_homography(const cv::Mat& next_image);
    virtual void create_visualization();

private:
    void recompute_msers_(cv::Mat image);
    void extract_points_(std::vector<cv::Point2f>& points, const ComponentStats& comp );

    void visualize_regions_hulls_ (cv::Mat& image, const std::vector<MatComponentStats>& msers);
    void visualize_points (cv::Mat& image, const std::vector<MatComponentStats>& msers, bool orientation);
    void visualize_stabilization_points (cv::Mat&  image, bool lines);
    void visualize_regions_cov (cv::Mat& image, const std::vector<MatComponentStats>& msers);
    void visualize_regions_box (cv::Mat& image, const std::vector<MatComponentStats>& msers);

private:
    int visualization_flags_;
    std::vector<cv::Point2f> points_, points0_;
    MatMser detector_;
    MatMserTracker tracker_;

    std::vector<ComponentStats> up_msers_, down_msers_;
    std::vector<ComponentStats> ref_up_msers_, ref_down_msers_;
    std::vector<ComponentStats> up_msers_0_, down_msers_0_;

    unsigned int count_;
    unsigned int recompute_T_ = 50;

};


#endif // MATMserStabilizer_HPP

#ifndef PATCHSTABILIZER_H
#define PATCHSTABILIZER_H

#include "Stabilizer.h"
#include <tuple>
#include <vector>

class PatchStabilizer : public Stabilizer
{
public:
    PatchStabilizer(const cv::Mat &frame_0, Warping warping);


protected:
    virtual cv::Mat get_next_homography(const cv::Mat& next_frame);
    virtual void create_visualization();

private:
    cv::Mat get_homography(const cv::Mat& frame, const cv::Mat& frame0, const cv::Mat_<cv::Vec2f>& gradI, const std::vector<cv::Matx22f>& Ais);
    std::tuple<cv::Mat, std::vector<cv::Matx22f> > compute_Ais_and_gradI0(const cv::Mat& frame);
private:
    using Vec8f = cv::Vec<float, 8>;
    using Matx88f = cv::Matx<float, 8, 8>;

    std::vector<std::vector<cv::Matx22f>> Ais_pyr_;
    std::vector<cv::Mat_<cv::Vec2f>> gradI0_pyr_;
    std::vector<cv::Mat> frame0_pyr_;

private:
    void init (const cv::Mat& frame);

    static constexpr int N_PATCHES_X = 150;
    static constexpr int N_PATCHES_Y = 100;
    static constexpr float EPS = 1e-3;
    static constexpr int MAXITER = 50;
    static constexpr int PYRAMID_N = 3;


};
#endif // PATCHSTABILIZER_H

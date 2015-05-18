#ifndef MATCOMPONENTSTATS_HPP
#define MATCOMPONENTSTATS_HPP

#include "opencv2/opencv.hpp"

/**
 * @brief The properties of a component.
 */
struct MatComponentStats {
    unsigned int N = 0;
    float stability = 0.;
    cv::Point2f mean = cv::Point2f(0., 0.);
    cv::Matx22f cov = cv::Matx22f(0., 0., 0., 0.);
    cv::Point2i min_point = cv::Point2i(0., 0.);
    cv::Point2i max_point = cv::Point2i(0., 0.);
    uchar min_val = 255;
    uchar max_val = 0;
    float mean_val = 0;
    cv::Point2i source = cv::Point2i(0., 0.);
    float angle = 0.;


    MatComponentStats() = default;

    /**
     * @brief Initialize with a given point and a certain level.
     * @param point    The point to initialize the component with.
     * @param value    The point's level.
     */
    MatComponentStats(cv::Point2i point, uchar value)
        : N{1}, min_val{value}, max_val{value}, mean_val{static_cast<float>(value)} {
        mean = point;
        min_point = point;
        max_point = point;
        source = point;
    }

    /**
     * @brief Merge with another component.
     * @param comp1   The component to merge with.
     */
    void merge(const MatComponentStats& comp1);

    /**
     * @brief Merge with another point.
     *
     * This is equivalent to merging with a new component consisting of just one point.
     *
     * @param point   The point to merge with.
     * @param value   The point's level.
     */
    void merge(cv::Point2i point, uchar value) {
        merge(MatComponentStats{point, value});
    }

    /**
     * @brief Transform the component properties with an affine transformation.
     * @param A the affine transformation.
     */
    void transform_affine(const cv::Matx23f& A);
};
#endif // MATCOMPONENTSTATS_HPP

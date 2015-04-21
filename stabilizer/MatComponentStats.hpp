#ifndef MATCOMPONENTSTATS_HPP
#define MATCOMPONENTSTATS_HPP

#include "opencv2/opencv.hpp"

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

    // remove?
    MatComponentStats() = default;

    MatComponentStats(cv::Point2i point, uchar value)
        : N{1}, mean{point}, min_point{point}, max_point{point},
          min_val{value}, max_val{value}, mean_val{static_cast<float>(value)},
          source{point} {}

    void merge(const MatComponentStats& comp1);

    void merge(cv::Point2i point, uchar value) {
        merge(MatComponentStats{point, value});
    }

    void transform_affine(const cv::Matx23f& A);
};
#endif // MATCOMPONENTSTATS_HPP

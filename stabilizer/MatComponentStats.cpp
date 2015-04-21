#include "MatComponentStats.hpp"


void MatComponentStats::transform_affine(const cv::Matx23f& A) {
    cv::Point2f new_mean;
    cv::Matx22f new_cov;
    cv::Point2i new_source;
    cv::Point2i new_min_point;
    cv::Point2i new_max_point;
    int new_N;

    new_mean.x = A(0, 0) * mean.x + A(0, 1) * mean.y + A(0, 2);
    new_mean.y = A(1, 0) * mean.x + A(1, 1) * mean.y + A(1, 2);

    for (int idx1 : {0, 1}) for (int idx2 : {0, 1})
        for (int idx3 : {0, 1}) for (int idx4 : {0, 1})
            new_cov(idx1, idx3) += A(idx1, idx2) * A(idx3, idx4) * cov(idx2, idx4);

    new_source.x = static_cast<int>(A(0, 0) * source.x + A(0, 1) * source.y + A(0, 2));
    new_source.y = static_cast<int>(A(1, 0) * source.x + A(1, 1) * source.y + A(1, 2));

    cv::Point2i new_corner11, new_corner12, new_corner21, new_corner22;

    new_corner11.x = static_cast<int>(A(0, 0) * min_point.x + A(0, 1) * min_point.y + A(0, 2));
    new_corner11.y = static_cast<int>(A(1, 0) * min_point.x + A(1, 1) * min_point.y + A(1, 2));

    new_corner12.x = static_cast<int>(A(0, 0) * min_point.x + A(0, 1) * max_point.y + A(0, 2));
    new_corner12.y = static_cast<int>(A(1, 0) * min_point.x + A(1, 1) * max_point.y + A(1, 2));

    new_corner21.x = static_cast<int>(A(0, 0) * max_point.x + A(0, 1) * min_point.y + A(0, 2));
    new_corner21.y = static_cast<int>(A(1, 0) * max_point.x + A(1, 1) * min_point.y + A(1, 2));

    new_corner22.x = static_cast<int>(A(0, 0) * max_point.x + A(0, 1) * max_point.y + A(0, 2));
    new_corner22.y = static_cast<int>(A(1, 0) * max_point.x + A(1, 1) * max_point.y + A(1, 2));

    new_min_point.x = std::min({new_corner11.x, new_corner12.x, new_corner21.x, new_corner22.x});
    new_min_point.y = std::min({new_corner11.y, new_corner12.y, new_corner21.y, new_corner22.y});

    new_max_point.x = std::max({new_corner11.x, new_corner12.x, new_corner21.x, new_corner22.x});
    new_max_point.y = std::max({new_corner11.y, new_corner12.y, new_corner21.y, new_corner22.y});

    new_N = static_cast<int> ((A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0)) * N);

    mean = new_mean;
    cov = new_cov;
    source = new_source;
    min_point = new_min_point;
    max_point = new_max_point;
    N = new_N;
}

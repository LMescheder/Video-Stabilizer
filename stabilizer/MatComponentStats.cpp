#include "MatComponentStats.hpp"

void MatComponentStats::merge(const MatComponentStats &comp1) {
    auto& comp2 = *this;
    auto p = float(comp1.N) / float(comp1.N + comp2.N);
    auto q = float(comp2.N) / float(comp1.N + comp2.N);

    cv::Vec2f dmean = comp2.mean - comp1.mean;
    for (auto i : {0, 1})
        for (auto j : {0, 1})
            comp2.cov(i, j) = p * comp1.cov(i, j) + q * comp2.cov(i, j) + p*q*dmean(i) * dmean(j);

    comp2.N = comp1.N + comp2.N;
    comp2.mean = p * comp1.mean + q * comp2.mean;

    comp2.min_point.x = std::min(comp1.min_point.x, comp2.min_point.x);
    comp2.min_point.y = std::min(comp1.min_point.y, comp2.min_point.y);
    comp2.max_point.x = std::max(comp1.max_point.x, comp2.max_point.x);
    comp2.max_point.y = std::max(comp1.max_point.y, comp2.max_point.y);

    comp2.min_val = std::min(comp1.min_val, comp2.min_val);
    comp2.max_val = std::max(comp1.max_val, comp2.max_val);
    comp2.mean_val = p * comp1.mean_val + q * comp2.mean_val;
    assert(min_point.x - 1e4 < mean.x && mean.x < max_point.x + 1e4);
    assert(min_point.y - 1e4 < mean.y && mean.y < max_point.y + 1e4);
}


void MatComponentStats::transform_affine(const cv::Matx23f& A) {
    cv::Point2f new_mean;
    cv::Matx22f new_cov;
    cv::Point2i new_source;
    cv::Point2i new_min_point;
    cv::Point2i new_max_point;
    int new_N;

    float new_angle;

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

    new_angle = std::atan2(A(1, 0)*std::cos(angle) + A(1, 1)*std::sin(angle),
                           A(0, 0)*std::cos(angle) + A(0, 1)*std::sin(angle));
    mean = new_mean;
    cov = new_cov;
    source = new_source;
    min_point = new_min_point;
    max_point = new_max_point;
    N = new_N;
    angle = new_angle;
}

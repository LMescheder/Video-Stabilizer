#ifndef MATMSER_HPP
#define MATMSER_HPP

#include <vector>
#include "opencv2/core.hpp"
#include "ComponentTreeParser.hpp"
#include "MatAccessor.hpp"
#include "MatAnalyzer.hpp"

class MatMser
{
public:
    using ComponentStats = MatMserAnalyzer::ComponentStats;

    MatMser(unsigned int delta=5, unsigned int min_N=60, unsigned int max_N=14400,
            float min_stability = 20.f, float min_diversity=.5f)
        : delta_(delta), min_N_(min_N), max_N_(max_N),
          min_stability_(min_stability), min_diversity_(min_diversity) {}

    std::vector<ComponentStats> find_msers (const cv::Mat& image) {
        ComponentTreeParser<MatAccessor, MatMserAnalyzer> parser{};

        MatAccessor graph1(image);
        MatMserAnalyzer analyzer1(delta_, min_N_, max_N_, min_stability_, min_diversity_);
        auto mser_stats1 = parser(graph1, analyzer1);

        MatAccessor graph2(image, true);
        MatMserAnalyzer analyzer2(delta_, min_N_, max_N_, min_stability_, min_diversity_);
        auto mser_stats2 = parser(graph2, analyzer2);

        std::vector<ComponentStats> result;
        for (auto& stat : mser_stats1)
            result.push_back(stat);
        for (auto& stat : mser_stats2)
            result.push_back(stat);
        return result;
    }



private:
    unsigned int delta_;
    unsigned int min_N_;
    unsigned int max_N_;
    float min_stability_;
    float min_diversity_;

    std::vector<cv::Point> find_points_ (MatMserAnalyzer::ComponentStats stats) {

    }
};

#endif // MATMSER_HPP

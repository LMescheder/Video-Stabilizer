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
    MatMser(unsigned int delta=5, unsigned int min_N=60, unsigned int max_N=14400,
            float min_stability = 20.f, float min_diversity=.5f)
        : delta_(delta), min_N_(min_N), max_N_(max_N),
          min_stability_(min_stability), min_diversity_(min_diversity) {}

    std::vector<std::vector<cv::Point>> operator() (const cv::Mat& image) {
        MatAccessor graph(image);
        MatMserAnalyzer analyzer(delta_, min_N_, max_N_, min_stability_, min_diversity_);
        ComponentTreeParser<MatAccessor, MatMserAnalyzer> parser{};

        auto mser_stats = parser(graph, analyzer);
    }



private:
    unsigned int delta_;
    unsigned int min_N_;
    unsigned int max_N_;
    float min_stability_;
    float min_diversity_;

    std::vector<cv::Point> find_points_ () {

    }
};

#endif // MATMSER_HPP

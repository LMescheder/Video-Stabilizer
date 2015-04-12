#ifndef MATMSER_HPP
#define MATMSER_HPP

#include <vector>
#include "opencv2/opencv.hpp"
#include "ComponentTreeParser.hpp"
#include "MatAccessor.hpp"
#include "MatAnalyzer.hpp"

class MatMser
{
public:
    using ComponentStats = MatMserAnalyzer::ComponentStats;
    enum direction {
        upwards = 1,
        downwards = 2
    };

    MatMser(unsigned int delta=5, unsigned int min_N=60, unsigned int max_N=14400,
            float min_stability = 20.f, float min_diversity=.2f,
            float min_retrieval_stability=.5f, float max_retrieval_error=1e4)
        : delta_(delta), min_N_(min_N), max_N_(max_N),
          min_stability_(min_stability), min_diversity_(min_diversity),
          min_retrieval_stability_{min_retrieval_stability}, max_retrieval_error_{max_retrieval_error}
        {}

    std::vector<ComponentStats> detect_msers (const cv::Mat& image, int dir=upwards|downwards);

    std::vector<std::vector<cv::Point2i>> detect_msers_points (const cv::Mat& image);

    ComponentStats retrieve_mser (const cv::Mat& image, const ComponentStats& target_stats, bool reverse);

    std::vector<ComponentStats> retrieve_msers (const cv::Mat& image,
                                   const std::vector<ComponentStats>& target_stats,
                                   bool reverse);

    std::vector<cv::Point2i> stats_to_points (const MatMserAnalyzer::ComponentStats& stats, const cv::Mat& im);

    std::vector<std::vector<cv::Point2i>> mult_stats_to_points (const std::vector<MatMserAnalyzer::ComponentStats>& stats,
                                                                const cv::Mat& image);

private:
    unsigned int delta_;
    unsigned int min_N_;
    unsigned int max_N_;
    float min_stability_;
    float min_diversity_;
    float min_retrieval_stability_;
    float max_retrieval_error_;


};

#endif // MATMSER_HPP

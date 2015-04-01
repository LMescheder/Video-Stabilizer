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

	MatMser(unsigned int delta=2, unsigned int min_N=60, unsigned int max_N=14400,
			float min_stability = 40.f, float min_diversity=.2f)
        : delta_(delta), min_N_(min_N), max_N_(max_N),
          min_stability_(min_stability), min_diversity_(min_diversity) {}

    std::vector<ComponentStats> find_msers (const cv::Mat& image) {
        ComponentTreeParser<MatAccessor, MatMserAnalyzer> parser{};
        MatMserAnalyzer analyzer(delta_, min_N_, max_N_, min_stability_, min_diversity_);

        auto mser_stats1 = parser(image, analyzer, false);
        auto mser_stats2 = parser(image, analyzer, true);

        std::vector<ComponentStats> result;
        for (auto& stat : mser_stats1)
           result.push_back(stat);
        for (auto& stat : mser_stats2)
            result.push_back(stat);
        return result;
    }

    std::vector<std::vector<cv::Point2i>> find_msers_points (const cv::Mat& image) {
        std::vector<ComponentStats> msers = find_msers(image);
        std::vector<std::vector<cv::Point2i>> points;
        points.reserve(msers.size());

        for (auto& stats : msers)
            points.push_back(find_points(stats, image));

        return points;
    }

    std::vector<cv::Point2i> find_points (const MatMserAnalyzer::ComponentStats& stats, const cv::Mat& im) {
        std::vector<MatAccessor::NodeIndex> toprocess;
        std::vector<cv::Point2i> points;

        points.reserve(stats.N);
        toprocess.reserve(stats.N);

        cv::Mat ROI = im(cv::Range(stats.min_point.y, stats.max_point.y+1),
                         cv::Range(stats.min_point.x, stats.max_point.x+1));

        MatAccessor graph(ROI);
        toprocess.push_back(graph.get_index(stats.source - stats.min_point) );
        while (!toprocess.empty()) {
            auto current_node = toprocess.back();
            toprocess.pop_back();

            while(auto next_neighbor_or_none = graph.get_next_neighbor(current_node)) {
                auto next_neighbor = *next_neighbor_or_none;
                auto value = graph.value(next_neighbor);
                if ( stats.min_val <= value && value <= stats.max_val)
                    toprocess.push_back(next_neighbor);
            }

            points.push_back(stats.min_point + graph.node(current_node));
        }

        assert(points.size() == stats.N);
        return points;
    }


private:
    unsigned int delta_;
    unsigned int min_N_;
    unsigned int max_N_;
    float min_stability_;
    float min_diversity_;


};

#endif // MATMSER_HPP

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
    enum direction {
        upwards = 1,
        downwards = 2
    };

    MatMser(unsigned int delta=5, unsigned int min_N=60, unsigned int max_N=14400,
            float min_stability = 40.f, float min_diversity=.2f)
        : delta_(delta), min_N_(min_N), max_N_(max_N),
          min_stability_(min_stability), min_diversity_(min_diversity) {}

    std::vector<ComponentStats> detect_msers (const cv::Mat& image, int dir=upwards|downwards) {
        ComponentTreeParser<MatAccessor, MatMserAnalyzer> parser{};
        MatMserAnalyzer analyzer(delta_, min_N_, max_N_, min_stability_, min_diversity_);

        std::vector<ComponentStats> result;

        if (dir & upwards) {
            auto mser_stats1 = parser(image, analyzer, false);
            for (auto& stat : mser_stats1)
               result.push_back(stat);
        }

        if (dir & downwards) {
            auto mser_stats2 = parser(image, analyzer, true);
            for (auto& stat : mser_stats2)
               result.push_back(stat);
        }

        return result;
    }

    std::vector<std::vector<cv::Point2i>> detect_msers_points (const cv::Mat& image) {
        return mult_stats_to_points(detect_msers(image), image);
    }

    ComponentStats retrieve_mser (const cv::Mat& image, const ComponentStats& target_stats, bool reverse) {
        ComponentTreeParser<MatAccessor, MatFindMserAnalyzer> parser{};
        MatFindMserAnalyzer analyzer(target_stats, delta_);

        int dx = static_cast<int> (.05 * (target_stats.max_point.x - target_stats.min_point.x));
        int dy = static_cast<int> (.05 * (target_stats.max_point.y - target_stats.min_point.y));
        cv::Point2i p1{std::max(target_stats.min_point.x - dx, 0),
                       std::max(target_stats.min_point.y - dy, 0)};
        cv::Point2i p2{std::min(target_stats.max_point.x + dx, image.cols-1),
                       std::min(target_stats.max_point.y + dy, image.rows-1)};

        cv::Mat ROI = image(cv::Range(p1.y, p2.y+1), cv::Range(p1.x, p2.x+1));
        MatAccessor graph(ROI, reverse, p1);

        auto val = ROI.at<uchar>(0, 0);
        return parser(graph, analyzer, reverse);
    }

    std::vector<ComponentStats> retrieve_msers (const cv::Mat& image,
                                   const std::vector<ComponentStats>& target_stats,
                                   bool reverse) {
        std::vector<ComponentStats> result;
        result.reserve(target_stats.size());
        for (auto& s : target_stats)
            result.push_back(retrieve_mser(image, s, reverse));
        return result;
    }

    std::vector<cv::Point2i> stats_to_points (const MatMserAnalyzer::ComponentStats& stats, const cv::Mat& im) {
        if (stats.N == 0)
            return {};

        std::vector<MatAccessor::NodeIndex> toprocess;
        std::vector<cv::Point2i> points;

        points.reserve(stats.N);
        toprocess.reserve(stats.N);

        cv::Mat ROI = im(cv::Range(stats.min_point.y, stats.max_point.y+1),
                         cv::Range(stats.min_point.x, stats.max_point.x+1));

        MatAccessor graph(ROI, false, stats.min_point);
        toprocess.push_back(graph.get_index(stats.source) );
        while (!toprocess.empty()) {
            auto current_node = toprocess.back();
            toprocess.pop_back();

            while(auto next_neighbor_or_none = graph.get_next_neighbor(current_node)) {
                auto next_neighbor = *next_neighbor_or_none;
                auto value = graph.value(next_neighbor);
                if ( stats.min_val <= value && value <= stats.max_val)
                    toprocess.push_back(next_neighbor);
            }

            points.push_back(graph.node(current_node));
        }
        if (points.size() != stats.N)
            std::cout << points.size() << ' ' << stats.N << std::endl;
        //assert(points.size() == stats.N);
        return points;
    }

    std::vector<std::vector<cv::Point2i>> mult_stats_to_points (const std::vector<MatMserAnalyzer::ComponentStats>& stats,
                                                                const cv::Mat& image) {
        std::vector<std::vector<cv::Point2i>> points;
        points.reserve(stats.size());

        for (auto& s : stats)
          points.push_back(stats_to_points(s, image));

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

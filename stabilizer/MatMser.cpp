#include "MatMser.hpp"

using ComponentStats = typename MatMser::ComponentStats;

// mser detection
std::vector<ComponentStats> MatMser::detect_msers(const cv::Mat &image, int dir) {
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

std::vector<std::vector<cv::Point2i> > MatMser::detect_msers_points(const cv::Mat &image) {
    return mult_stats_to_points(detect_msers(image), image);
}

// mser retrieval
MatMser::ComponentStats MatMser::retrieve_mser(const cv::Mat &image, const MatMser::ComponentStats &target_stats, bool reverse) {
    ComponentTreeParser<MatAccessor, MatFindMserAnalyzer> parser{};
    MatFindMserAnalyzer analyzer(target_stats, delta_, max_retrieval_error_, min_retrieval_stability_);

    int dx = static_cast<int> (.01 * std::max(image.rows, image.cols));
    int dy = dx;

    cv::Point2i p1{std::max(target_stats.min_point.x - dx, 0),
                std::max(target_stats.min_point.y - dy, 0)};
    cv::Point2i p2{std::min(target_stats.max_point.x + dx, image.cols-1),
                std::min(target_stats.max_point.y + dy, image.rows-1)};

    cv::Mat ROI = image(cv::Range(p1.y, p2.y+1), cv::Range(p1.x, p2.x+1));
    MatAccessor graph(ROI, p1, reverse);

    return parser(graph, analyzer, reverse);
}

std::vector<MatMser::ComponentStats> MatMser::retrieve_msers(const cv::Mat &image, const std::vector<MatMser::ComponentStats> &target_stats, bool reverse) {
    std::vector<ComponentStats> result;
    result.reserve(target_stats.size());
    for (auto& s : target_stats)
        result.push_back(retrieve_mser(image, s, reverse));
    return result;
}

// mser conversion
std::vector<cv::Point2i> MatMser::stats_to_points(const MatMserAnalyzer::ComponentStats &stats, const cv::Mat &im) {
    if (stats.N == 0)
        return {};

    std::vector<MatAccessor::NodeIndex> toprocess;
    std::vector<cv::Point2i> points;

    points.reserve(stats.N);
    toprocess.reserve(stats.N);

    cv::Mat ROI = im(cv::Range(stats.min_point.y, stats.max_point.y+1),
                     cv::Range(stats.min_point.x, stats.max_point.x+1));

    MatAccessor graph(ROI, stats.min_point, false);
    toprocess.push_back(graph.get_source(stats.source) );
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

    assert(points.size() == stats.N);
    return points;
}

std::vector<std::vector<cv::Point2i> > MatMser::mult_stats_to_points(const std::vector<MatMserAnalyzer::ComponentStats> &stats, const cv::Mat &image) {
    std::vector<std::vector<cv::Point2i>> points;
    points.reserve(stats.size());

    for (auto& s : stats)
        points.push_back(stats_to_points(s, image));

    return points;
}

std::vector<cv::Point2f> MatMser::extract_means(const std::vector<ComponentStats>& stats)
{
    std::vector<cv::Point2f> result;
    result.reserve(stats.size());

    for (auto& s : stats)
        result.push_back(s.mean);

    return result;
}






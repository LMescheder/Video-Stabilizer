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

    // utility functions
    static std::vector<cv::Point2i> stats_to_points (const MatMserAnalyzer::ComponentStats& stats, const cv::Mat& im);

    static std::vector<std::vector<cv::Point2i>> mult_stats_to_points (const std::vector<MatMserAnalyzer::ComponentStats>& stats,
                                                                const cv::Mat& image);

    static std::vector<cv::Point2f> extract_means (const std::vector<ComponentStats>& stats);

    static cv::Point2i clamp(cv::Point2i p, const cv::Mat& image  ) {
        int x = std::min(std::max(p.x, 0), image.cols-1);
        int y = std::min(std::max(p.y, 0), image.rows-1);

        return {x, y};
    }

    // getter and setter functions
    unsigned int delta () const {
        return delta_;
    }
    void set_delta (unsigned int delta) {
        delta_ = delta;
    }

    unsigned int min_N() const {
        return min_N_;
    }
    void set_min_N(unsigned int min_N) {
        min_N_ = min_N;
    }

    unsigned int max_N() const {
        return max_N_;
    }
    void set_max_N(unsigned int max_N) {
        max_N_ = max_N;
    }

    float min_stability() const {
        return min_stability_;
    }
    void set_min_stability(float min_stability) {
        min_stability_ = min_stability;
    }

    float min_diversity() const {
        return min_diversity_;
    }
    void set_min_diversity(float min_diversity) {
        min_diversity_ = min_diversity;
    }

    float min_retrieval_stability() const {
        return min_retrieval_stability_;
    }
    void set_min_retrieval_stability(float min_retrieval_stability) {
        min_retrieval_stability_ = min_retrieval_stability;
    }

    float max_retrieval_error() const {
        return max_retrieval_error_;
    }
    void set_max_retrieval_error(float max_retrieval_error) {
        max_retrieval_error_ = max_retrieval_error;
    }

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

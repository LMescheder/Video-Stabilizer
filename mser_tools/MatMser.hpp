#ifndef MATMSER_HPP
#define MATMSER_HPP

#include <vector>
#include "opencv2/opencv.hpp"
#include "ComponentTreeParser.hpp"
#include "MatAccessor.hpp"
#include "MatAnalyzer.hpp"

/**
 * @brief Class to extract and retrieve msers from a gray level image.
 *
 * This class parametrizes the ComponentTreeParser template with MatAccessor for
 * accessing the image and MatMserAnalyzer/MatFindMserAnalyzer for extracting/retrieving msers
 * from images. It also provides some other functionality like finding the points corresponding to a
 * mser or extract all means into a vector.
 */
class MatMser
{
public:
    using ComponentStats = MatMserAnalyzer::ComponentStats;
    enum direction {
        upwards = 1,
        downwards = 2
    };

    /**
     * @brief Constructor.
     * @param delta             Delta parameter for the stability computation.
     * @param min_N             Minimum size for a mser.
     * @param max_N             Maximum size for a mser.
     * @param min_stability     Minimum stability for a mser.
     * @param min_diversity     Minimum diversity
     *                          (\f$ = \frac{\text{current_size} - \text{last_size}}{\text{current_size}}\f$)
     * @param min_retrieval_stability  Minimum stability for mser retrieval.
     * @param max_retrieval_error      Maximum error for mser retrieval.
     */
    MatMser(unsigned int delta=5, unsigned int min_N=60, unsigned int max_N=14400,
            float min_stability = 20.f, float min_diversity=.2f,
            float min_retrieval_stability=.5f, float max_retrieval_error=1e4)
        : delta_(delta), min_N_(min_N), max_N_(max_N),
          min_stability_(min_stability), min_diversity_(min_diversity),
          min_retrieval_stability_{min_retrieval_stability}, max_retrieval_error_{max_retrieval_error}
        {}

    /**
     * @brief Detects msers in the a gray level image.
     * @param image A gray level image where msers are to be detected.
     * @param dir   The direction in which the component tree should be parsed.
     * @return A vector with ComponentStats describing the found msers.
     *         This can be used by MatMser::stats_to_points to find the points
     */
    std::vector<ComponentStats> detect_msers (const cv::Mat& image, int dir=upwards|downwards);

    /**
     * @brief Detects the mser in a gray level image as a vector of points.
     * @param image A gray level image where msers are to be detected.
     * @param dir   The direction in which the component tree should be parsed.
     * @return A vector of a vector of points describing the found msers.
     */
    std::vector<std::vector<cv::Point2i>> detect_msers_points (const cv::Mat& image, int dir=upwards|downwards);

    /**
     * @brief Retrieves a mser in a given image.
     * @param image         The gray level image in which the mser is to be retrieved.
     * @param target_stats  The ComponentStats describing the mser to be retrieved.
     * @param reverse       If the component tree shoud be parsed in reverse direction.
     * @return              The ComponentStats describing the retrieved mser.
     */
    ComponentStats retrieve_mser (const cv::Mat& image, const ComponentStats& target_stats, bool reverse);

    /**
     * @brief Retrieves multiple msers in a given image.
     * @param image         The gray level image in which the mser is to be retrieved.
     * @param target_stats  The ComponentStats describing the msers to be retrieved.
     * @param reverse       If the component tree shoud be parsed in reverse direction.
     * @return              The ComponentStats describing the retrieved mser.
     */
    std::vector<ComponentStats> retrieve_msers (const cv::Mat& image,
                                   const std::vector<ComponentStats>& target_stats,
                                   bool reverse);

    // Utility functions
    /**
     * @brief Find the points belonging to a certain component in an image.
     * @param stats    The ComponentStats describing the component.
     * @param im       The gray level image in which the component lies.
     * @return Vector of cv::Point2i describing the points of the component.
     */
    static std::vector<cv::Point2i> stats_to_points (const MatMserAnalyzer::ComponentStats& stats, const cv::Mat& im);

    /**
     * @brief Find the points belonging to multiple components in an image.
     * @param stats    The ComponentStats describing the components.
     * @param im       The gray level image in which the component lies.
     * @return Vector of cv::Point2i vector describing the points of the components.
     */
    static std::vector<std::vector<cv::Point2i>> mult_stats_to_points (const std::vector<MatMserAnalyzer::ComponentStats>& stats,
                                                                const cv::Mat& image);

    /**
     * @brief Extract the means of multiple components.
     * @param stats   A vector of ComponentStats describing the components.
     * @return A vector of cv::Point2f representing the means of those components.
     */
    static std::vector<cv::Point2f> extract_means (const std::vector<ComponentStats>& stats);

    /**
     * @brief Clamps a point to an image
     * @param p        The point to be clamped.
     * @param image    The image to which the point should be clamped.
     * @return  The clamped point.
     */
    static cv::Point2i clamp(cv::Point2i p, const cv::Mat& image  ) {
        int x = std::min(std::max(p.x, 0), image.cols-1);
        int y = std::min(std::max(p.y, 0), image.rows-1);

        return {x, y};
    }

    // Getter and setter functions

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

    float roi_factor_ = .2;
    int roi_min_ = 5;
};

#endif // MATMSER_HPP

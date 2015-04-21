#ifndef MATMSERANALYZER_HPP
#define MATMSERANALYZER_HPP

#include "opencv2/core.hpp"
#include <cmath>
#include "MatComponentStats.hpp"

// ------------------------------------------------------------------------------------------------
// ------------------------------- declarations ---------------------------------------------------
// ------------------------------------------------------------------------------------------------

inline float compute_stability(unsigned int pred_N, uchar pred_level, unsigned int N, uchar level, unsigned int succ_N, uchar succ_level) {
    float stability =  static_cast<float>(N * std::abs(succ_level - pred_level))/(succ_N - pred_N);
    assert(stability >= 0);
    return stability;
}


/* TODO: Put Mser extraction into derived class
 * TODO: Create derived class for tracking
 * TODO: Optimize history (less reallocation) by creating a (#minima * history_length) array
 * Uses CRTP, so that no runtime cost is inflicted.
 * This class is thus purely abstract. It is neccessary to implement the check_component_ member
 * function in a derived class.
*/

class MatMserAnalyzer {
public:
    using ComponentStats = MatComponentStats;

    struct Component {
        uchar level;
        uchar stability = 0;
        ComponentStats stats;
        unsigned int last_mser_N = 0;

        std::vector<ComponentStats> history;
        std::vector<uchar> level_history;

        Component(uchar value)
            : level{value} {}

        Component(cv::Point2i point, uchar value)
            : level{value}, stats{point, value} {}
    };


    using Result = std::vector<ComponentStats>;

    MatMserAnalyzer (unsigned int delta=5, unsigned int min_N=60, unsigned int max_N=14400,
                 float min_stability = 20.f, float min_diversity=.5f)
     : delta_(delta), min_N_(min_N), max_N_(max_N), min_stability_(min_stability), min_diversity_(min_diversity) {}

    void reset(){
        result_.clear();
    }

    void raise_level (Component& comp, uchar level) {
        assert(level != comp.level);
        extend_history_(comp);
        comp.level = level;
    }

    uchar get_level (const Component& comp) const{
        return comp.level;
    }

    void merge_component_into(Component& comp1, Component& comp2);


    Component add_component (cv::Point2i point, uchar level) {
        return Component(point, level);
    }

    Component add_component (uchar level) {
        return Component(level);
    }

    void add_node( typename cv::Point2i node, Component& component) {
        component.stats.merge(node, component.level);
    }

    // TODO: put into derived classes
    Result get_result() {
        return std::move(result_);
    }

private:
    Result result_;
    const unsigned int delta_ = 5;
    const unsigned int min_N_;
    const unsigned int max_N_;
    const float min_stability_;
    const float min_diversity_;

    void extend_history_(Component& component);

    void calculate_stability(Component& comp);


    void check_component_(Component &comp);

};

/* FindMserAnalyzer
 *
 *
 */


class MatFindMserAnalyzer {
public:
    using ComponentStats = MatComponentStats;

    struct Component {
        uchar level;
        ComponentStats stats;

        std::vector<ComponentStats> history;
        std::vector<uchar> level_history;

        Component(uchar value)
            : level{value} {}

        Component(cv::Point2i point, uchar value)
            : level{value}, stats{point, value} {}
    };


    using Result = ComponentStats;

    MatFindMserAnalyzer (ComponentStats target_stats, unsigned int delta=5, float max_error=1000, float min_stability=0.)
     : target_stats_{target_stats}, delta_{delta}, max_error_{max_error}, min_stability_{min_stability} {}

    void reset(){

    }

    void raise_level (Component& comp, uchar level) {
        assert(level != comp.level);
        extend_history_(comp);
        comp.level = level;
    }

    uchar get_level (const Component& comp) const{
        return comp.level;
    }

    void merge_component_into(Component& comp1, Component& comp2);


    Component add_component (cv::Point2i point, uchar level) {
        return Component(point, level);
    }

    Component add_component (uchar level) {
        return Component(level);
    }

    void add_node( typename cv::Point2i node, Component& component) {
        component.stats.merge(node, component.level);
    }

    Result get_result() {
        return result_;
    }

private:
    Result result_;
    ComponentStats target_stats_;
    float current_optimal_ = -1;

    unsigned int delta_ = 5;
    float max_error_ = 1.e3;
    float min_stability_ = 0.;

    float weight_mean_ = 1.e0;
    float weight_boundingbox_ = 0.e1;
    float weight_mean_val_ = 1.e0;
    float weight_interval_ = 1.e0;
    float weight_N_ = 1.e0;
    float weight_cov_ = 1.e0;
    float weight_dstability_= 0.e0;

    void extend_history_(Component& component);


    void check_component_(Component &comp);

    void calculate_stability(Component& comp);

    float compute_error_(float val, float target, float norm=1.f) {
        float err = (val - target)/norm;
        return err*err;
    }

    float compute_pos_error_(float val, float target) {
        float err = std::max((val - target), 0.f);
        return err*err;
    }

    float compute_neg_error_(float val, float target) {
        float err = std::min((val - target), 0.f);
        return err*err;
    }

    float compute_rel_error_(float val, float target) {
        float err = (val - target)/target;
        return err*err;
    }

};


#endif // MATMSERANALYZER_HPP

#ifndef MATMSERANALYZER_HPP
#define MATMSERANALYZER_HPP

#include "opencv2/core.hpp"
#include <cmath>

// ------------------------------------------------------------------------------------------------
// ------------------------------- declarations ---------------------------------------------------
// ------------------------------------------------------------------------------------------------

struct MatComponentStats {
    unsigned int N = 0;
    float stability = 0.;
    cv::Point2f mean = cv::Point2f(0., 0.);
    cv::Matx22f cov = cv::Matx22f(0., 0., 0., 0.);
    cv::Point2i min_point = cv::Point2i(0., 0.);
    cv::Point2i max_point = cv::Point2i(0., 0.);
    uchar min_val = 255;
    uchar max_val = 0;
    float mean_val = 0;
    cv::Point2i source = cv::Point2i(0., 0.);

    // remove?
    MatComponentStats() = default;

    MatComponentStats(cv::Point2i point, uchar value)
        : N{1}, mean{point}, min_point{point}, max_point{point},
          min_val{value}, max_val{value}, mean_val{static_cast<float>(value)},
          source{point} {}

    void merge(const MatComponentStats& comp1) {
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

    void merge(cv::Point2i point, uchar value) {
        merge(MatComponentStats{point, value});
    }
};



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

    void merge_component_into(Component& comp1, Component& comp2) {
        assert(comp1.level != comp2.level);

        // take the history of the winner
        if (comp1.stats.N > comp2.stats.N) {
            extend_history_(comp1);
            comp2.history = std::move(comp1.history);
            comp2.level_history = std::move(comp1.level_history);
            comp2.last_mser_N = comp1.last_mser_N;
         }

        comp2.stats.merge(comp1.stats);
    }


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

    void extend_history_(Component& component) {
        component.history.push_back(component.stats);
        component.level_history.push_back(component.level);

        assert(component.history.size() == component.level_history.size());

        calculate_stability(component);
        check_component_(component);

    }

    void calculate_stability(Component& comp) {
        if(comp.history.size() >= 2*delta_ + 1) {
            auto& pred = comp.history.rbegin()[2*delta_];
            uchar pred_level = comp.level_history.rbegin()[2*delta_];
            auto& examinee = comp.history.rbegin()[delta_];
            uchar level = comp.level_history.rbegin()[delta_];
            auto& succ = comp.history.rbegin()[0];
            uchar succ_level = comp.level_history.rbegin()[0];

            assert(pred.N < examinee.N && examinee.N < succ.N);

            examinee.stability = compute_stability(pred.N, pred_level, examinee.N, level, succ.N, succ_level);
        }
    }


    void check_component_(Component &comp) {
        if (comp.history.size() >= 2*delta_ + 1 && comp.history.size() >= delta_ + 3) {
            auto& succ = comp.history.rbegin()[delta_];
            auto& examinee = comp.history.rbegin()[delta_+1];
            auto& pred = comp.history.rbegin()[delta_+2];

            auto diversity = static_cast<float> (examinee.N - comp.last_mser_N) / examinee.N;
            if (examinee.stability > pred.stability && examinee.stability > succ.stability
                    && min_N_ <= examinee.N && examinee.N <= max_N_
                    && examinee.stability >= min_stability_
                    && diversity >= min_diversity_) {
                result_.push_back(examinee);
                comp.last_mser_N = examinee.N;
            }
        }
    }

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

    void merge_component_into(Component& comp1, Component& comp2) {
        assert(comp1.level != comp2.level);

        // take the history of the winner
        if (comp1.stats.N > comp2.stats.N) {
            extend_history_(comp1);
            comp2.history = std::move(comp1.history);
            comp2.level_history = std::move(comp1.level_history);
         }

        comp2.stats.merge(comp1.stats);
    }


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

    float weight_mean_ = 1.e6;
    float weight_boundingbox_ = 1.e2;
    float weight_mean_val_ = 1.;
    float weight_interval_ = 1.e0;
    float weight_N_ = 1.e4;
    float weight_cov_ = 1.e4;
    float weight_dstability_= 1.e2;

    void extend_history_(Component& component) {
        component.history.push_back(component.stats);
        component.level_history.push_back(component.level);

        assert(component.history.size() == component.level_history.size());

        calculate_stability(component);
        check_component_(component);
    }


    void check_component_(Component &comp) {
        assert(comp.history.size() == comp.level_history.size());


        if (comp.history.size() >= 2*delta_ + 1 && comp.history.size() >= delta_ + 3) {
            auto& pred = comp.history.rbegin()[delta_+2];
            uchar pred_level = comp.level_history.rbegin()[delta_+2];
            auto& examinee = comp.history.rbegin()[delta_+1];
            uchar level = comp.level_history.rbegin()[delta_+1];
            auto& succ = comp.history.rbegin()[delta_];
            uchar succ_level = comp.level_history.rbegin()[delta_];

            float dstability1 = (examinee.stability - pred.stability)/std::abs(level - pred_level);
            float dstability2 = (succ.stability - examinee.stability)/std::abs(succ_level - level);


            // compute cost function
            float cost = 0;


            cost += .5*weight_mean_ * compute_error_(examinee.mean.x, target_stats_.mean.x,
                                                     target_stats_.max_point.x - target_stats_.min_point.x);
            cost += .5*weight_mean_ * compute_error_(examinee.mean.y, target_stats_.mean.y,
                                                     target_stats_.max_point.y - target_stats_.min_point.y);
            cost += .5*weight_boundingbox_* compute_rel_error_(examinee.max_point.x - examinee.min_point.x,
                                                           target_stats_.max_point.x - target_stats_.min_point.x);
            cost += .5*weight_boundingbox_ * compute_rel_error_(examinee.max_point.y - examinee.min_point.y,
                                                            target_stats_.max_point.y - target_stats_.min_point.y);

            cost += weight_N_ * compute_rel_error_(examinee.N, target_stats_.N);
            cost += weight_mean_val_ * compute_error_(examinee.mean_val, target_stats_.mean_val);
            //cost += .5*weight_interval_ * compute_neg_error_(examinee.min_val,  target_stats_.min_val);
            //cost += .5*weight_interval_ * compute_pos_error_(examinee.max_val,  target_stats_.max_val );

            cost += weight_dstability_ * compute_neg_error_(dstability1, 0.);
            cost += weight_dstability_ * compute_pos_error_(dstability2, 0.);


            for (auto i : {0, 1})
                for (auto j : {0, 1})
                    cost +=  .25*weight_cov_ * compute_rel_error_(examinee.cov(i, j), target_stats_.cov(i, j));

            //cost += 1.e4/(examinee.stability * examinee.stability);

            if ((cost < max_error_) && (current_optimal_ < 0 || cost < current_optimal_)
                    && examinee.stability >= min_stability_) {
                result_ = examinee;
                current_optimal_ = cost;
            }
        }
    }

    void calculate_stability(Component& comp) {
        if(comp.history.size() >= 2*delta_ + 1) {
            auto& pred = comp.history.rbegin()[2*delta_];
            uchar pred_level = comp.level_history.rbegin()[2*delta_];
            auto& examinee = comp.history.rbegin()[delta_];
            uchar level = comp.level_history.rbegin()[delta_];
            auto& succ = comp.history.rbegin()[0];
            uchar succ_level = comp.level_history.rbegin()[0];

            assert(pred.N < examinee.N && examinee.N < succ.N);

            examinee.stability = compute_stability(pred.N, pred_level, examinee.N, level, succ.N, succ_level);
        }
    }

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

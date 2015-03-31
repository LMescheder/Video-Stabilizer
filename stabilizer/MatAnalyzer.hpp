#ifndef MATMSERANALYZER_HPP
#define MATMSERANALYZER_HPP

#include "opencv2/core.hpp"

// ------------------------------------------------------------------------------------------------
// ------------------------------- declarations ---------------------------------------------------
// ------------------------------------------------------------------------------------------------

/* TODO: Put Mser extraction into derived class
 * TODO: Create derived class for tracking
 * TODO: Optimize history (less reallocation) by creating a (#minima * history_length) array
 * Uses CRTP, so that no runtime cost is inflicted.
 * This class is thus purely abstract. It is neccessary to implement the check_component_ member
 * function in a derived class.
*/
template <typename Child>
class MatAnalyzer {
public:
    struct ComponentStats {
        unsigned int N = 0;
        cv::Vec2f mean = cv::Vec2f(0., 0.);
        cv::Matx22f cov = cv::Matx22f(0., 0., 0., 0.);
        cv::Point2i min_point = cv::Point2i(0., 0.);
        cv::Point2i max_point = cv::Point2i(0., 0.);
        float stability = 0.;
    };

    struct Component {
        uchar level;
        ComponentStats stats;
        std::vector<ComponentStats> history;
        //std::vector<uchar> history_levels;

        Component (uchar value)
            : level(value) {}
    };


    using Result = std::vector<ComponentStats>;

    MatAnalyzer (unsigned int delta) : delta_(delta) {}

    void raise_level (Component& comp, uchar level) {
        assert(level > comp.level);
        extend_history_(comp,  level);
        comp.level = level;
    }

    uchar get_level (const Component& comp) const{
        return comp.level;
    }

    void add_node( typename cv::Point2i node, uchar level, Component& component);

    void merge_component_into (Component& comp1, Component& comp2, uchar level);

    Component add_component (cv::Point2i point, uchar level) {
        Component new_comp = Component(level);
        new_comp.stats.N = 1;
        new_comp.stats.mean = cv::Vec2f(point);
        return new_comp;
    }

    // TODO: still makes a deep copy -> has to be optimized (would calling std::move be save?
    Result get_result() { return result_; }

protected:
    Result result_;
    const unsigned int delta_ = 5;


    void extend_history_(Component& comp, uchar level);
    void merge_componentstats_into_(const ComponentStats& comp1, ComponentStats& comp2);
    void calculate_stability(Component& comp);

    void check_component_ (Component& comp) {
        static_cast<Child*>(this)->check_component_(static_cast<typename Child::Component&> (comp));
    }
};

/* Analyzes the component tree to find msers.
 *
 */
class MatMserAnalyzer : public MatAnalyzer<MatMserAnalyzer> {
public:
    MatMserAnalyzer (unsigned int delta=5, unsigned int min_N=60, unsigned int max_N=14400,
                     float min_stability = 20.f, float min_diversity=.5f)
        : MatAnalyzer<MatMserAnalyzer>(delta),
         min_N_(min_N), max_N_(max_N), min_stability_(min_stability), min_diversity_(min_diversity) {}

    struct Component : public MatAnalyzer<MatMserAnalyzer>::Component {
        Component (uchar level) : MatAnalyzer<MatMserAnalyzer>::Component(level) {}

        unsigned int last_mser_N = 0;
    };

    Component add_component (uchar level) {
        return Component(level);
    }

    Component add_component (cv::Point2i point, uchar level) {
        Component new_comp = Component(level);
        new_comp.stats.N = 1;
        new_comp.stats.mean = cv::Vec2f(point.x, point.y);
        new_comp.stats.min_point = point;
        new_comp.stats.max_point = point;
        return new_comp;
    }

    void merge_component_into (Component& comp1, Component& comp2, uchar level) {
        if (comp1.stats.N > comp2.stats.N )
            comp2.last_mser_N = comp1.last_mser_N;
        MatAnalyzer<MatMserAnalyzer>::merge_component_into(comp1, comp2, level);
    }

    void check_component_(Component &comp) {
        if (comp.history.size() >= 2*delta_ + 1) {
            auto& succ = comp.history.rbegin()[delta_-1];
            auto& examinee = comp.history.rbegin()[delta_];
            auto& pred = comp.history.rbegin()[delta_ + 1];

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

private:
    const unsigned int min_N_;
    const unsigned int max_N_;
    const float min_stability_;
    const float min_diversity_;
};

// ------------------------------------------------------------------------------------------------
// --------------------------------------- definitions --------------------------------------------
// ------------------------------------------------------------------------------------------------

// TODO: Remove level from parameter list
template <typename Child>
void MatAnalyzer<Child>::add_node(cv::Point2i node, uchar level, MatAnalyzer<Child>::Component &component) {
    assert(level == component.level);
    ComponentStats node_comp;
    node_comp.mean = cv::Vec2f(node.x, node.y);
    node_comp.N = 1;

    merge_componentstats_into_(node_comp, component.stats);


}

// TODO: Remove level from parameter list
// TODO: call extend history function instead of own function
template <typename Child>
void MatAnalyzer<Child>::merge_component_into(MatAnalyzer<Child>::Component &comp1,
                                              MatAnalyzer<Child>::Component &comp2,
                                              uchar level) {
    // take the history of the winner
    assert(comp1.level < comp2.level);
    assert(comp2.level <= level);

    // update history
    if (comp1.stats.N > comp2.stats.N) {
        extend_history_(comp1, comp2.level);
        comp2.history = std::move(comp1.history);
     }

    merge_componentstats_into_(comp1.stats, comp2.stats);
}
// TODO: remove level parameter
// TODO: put check mser into subclass
template <typename Child>
void MatAnalyzer<Child>::extend_history_(MatAnalyzer<Child>::Component &component, uchar level) {
    assert(component.level < level);
    component.history.push_back(component.stats);
    //component.history_levels.push_back(component.level);
    calculate_stability(component);
    check_component_(component);

}

template <typename Child>
void MatAnalyzer<Child>::merge_componentstats_into_(const MatAnalyzer<Child>::ComponentStats &comp1,
                                                    MatAnalyzer<Child>::ComponentStats &comp2) {
    auto p = float(comp1.N) / float(comp1.N + comp2.N);
    auto q = float(comp2.N) / float(comp1.N + comp2.N);

    for (auto i : {0, 1})
        for (auto j : {0, 1})
            comp2.cov(i, j) = p * comp1.cov(i, j) + q * comp2.cov(i, j)
                    + p*q*(comp2.mean(i) - comp1.mean(i)) * (comp2.mean(j) - comp1.mean(j));

    comp2.N = comp1.N + comp2.N;
    comp2.mean = p * comp1.mean + q * comp2.mean;
}

template <typename Child>
void MatAnalyzer<Child>::calculate_stability(MatAnalyzer<Child>::Component &comp) {
    if(comp.history.size() >= 2*delta_ + 1) {
        auto& comp0 = comp.history.rbegin()[2*delta_];
        auto& comp1 = comp.history.rbegin()[delta_];
        auto& comp2 = comp.history.rbegin()[0];
        comp1.stability = static_cast<float>(2*delta_ * comp0.N)/(comp2.N - comp0.N);
    }
    /*
    if (comp.history.size() < delta_) {
        comp.stats.stability = 0;
    } else {
        auto old_N = comp.history.rbegin()[delta_-1].N;
        comp.stats.stability = static_cast<float>(delta_ * old_N)/(comp.stats.N - old_N);
    }
    */
}


#endif // MATMSERANALYZER_HPP

#ifndef OPENCVMATMSERANALYZER_HPP
#define OPENCVMATMSERANALYZER_HPP

#include "opencv2/core.hpp"

class OpenCVMatMserAnalyzer {
public:
    struct ComponentStats {
        unsigned int age = 0;
        unsigned int N = 0;
        cv::Vec2f mean = cv::Vec2f(0., 0.);
        cv::Matx22f cov = cv::Matx22f(0., 0., 0., 0.);
        float stability = 0.;
    };

    struct Component {
        uchar level;
        ComponentStats stats;
        std::vector<ComponentStats> history;

        Component (uchar value) : level(value) {
        }
    };


    using Result = std::vector<ComponentStats>;

    uchar get_level (Component& comp) {
        return comp.level;
    }

    void add_node( typename cv::Point2i node, uchar level, Component& component);

    void merge_component_into (Component& comp1, Component& comp2, uchar level);

    Component add_component (uchar level) {
        return Component{level};
    }

    // TODO: still makes a deep copy -> has to be optimized (would calling std::move be save?
    Result get_result() { return result_; }

    bool is_finished() {
        return finished_;
    }

private:
    Result result_;
    bool finished_ = false;
    const unsigned int min_N_ = 200;
    const unsigned int max_N_ = 14400;
    const uchar delta_ = 5;
    const float min_stability_ = 20.;

    void extend_history_(Component& comp, const ComponentStats& stats, uchar level);


    void merge_componentstats_into_(const ComponentStats& comp1, ComponentStats& comp2);
    void calculate_stability(Component& comp);
    void check_mser_ (Component& comp);
};

void OpenCVMatMserAnalyzer::add_node(cv::Point2i node, uchar level, OpenCVMatMserAnalyzer::Component &component) {
    ComponentStats node_comp;
    node_comp.mean = cv::Vec2f(node.x, node.y);
    node_comp.N = 1;

    extend_history_(component, component.stats, level);
    merge_componentstats_into_(node_comp, component.stats);


}

void OpenCVMatMserAnalyzer::merge_component_into(OpenCVMatMserAnalyzer::Component &comp1, OpenCVMatMserAnalyzer::Component &comp2, uchar level) {
    // take the history of the winner
    Component* winner;
    if (comp1.stats.N > comp2.stats.N) {
        winner = &comp1;
        comp2.history = std::move(comp1.history);
    } else {
        winner = &comp2;
    }

    // update history
    if (level > winner->level) {
        for (int i=0; i<level - winner->level; ++i)
            comp2.history.push_back(winner->stats);
        comp2.level = level;
        check_mser_(comp2);
    }

    merge_componentstats_into_(comp1.stats, comp2.stats);
    calculate_stability(comp2);
}

void OpenCVMatMserAnalyzer::extend_history_(OpenCVMatMserAnalyzer::Component &component, const OpenCVMatMserAnalyzer::ComponentStats& stats, uchar level) {

    if (level > component.level) {
        for (int i=0; i<level - component.level; ++i) {
            component.history.push_back(stats);
            calculate_stability(component);
            check_mser_(component);
         }
    }
    component.level = level;

}

void OpenCVMatMserAnalyzer::merge_componentstats_into_(const OpenCVMatMserAnalyzer::ComponentStats &comp1, OpenCVMatMserAnalyzer::ComponentStats &comp2) {
    auto p = float(comp1.N) / float(comp1.N + comp2.N);
    auto q = float(comp2.N) / float(comp1.N + comp2.N);

    for (auto i : {0, 1})
        for (auto j : {0, 1})
            comp2.cov(i, j) = p * comp1.cov(i, j) + q * comp2.cov(i, j)
                    + p*q*(comp2.mean(i) - comp1.mean(i)) * (comp2.mean(j) - comp1.mean(j));

    comp2.N = comp1.N + comp2.N;
    comp2.mean = p * comp1.mean + q * comp2.mean;
}

void OpenCVMatMserAnalyzer::calculate_stability(OpenCVMatMserAnalyzer::Component &comp) {
    if (comp.history.size() < delta_) {
        comp.stats.stability = 0;
    } else {
        auto old_N = comp.history.rbegin()[delta_-1].N;
        comp.stats.stability = static_cast<float>(delta_ * old_N)/(comp.stats.N - old_N);
    }

}

void OpenCVMatMserAnalyzer::check_mser_(OpenCVMatMserAnalyzer::Component &comp) {
    if (comp.history.size() >= 3) {
        auto& succ = comp.history.rbegin()[0];
        auto& examinee = comp.history.rbegin()[1];
        auto& pred = comp.history.rbegin()[2];

        if (examinee.stability > pred.stability && examinee.stability > succ.stability
             && min_N_ <= examinee.N && examinee.N <= max_N_
             && examinee.stability >= min_stability_)
                result_.push_back(examinee);
    }
}
#endif // OPENCVMATMSERANALYZER_HPP

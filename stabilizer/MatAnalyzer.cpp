#include "MatAnalyzer.hpp"


void MatComponentStats::merge(const MatComponentStats &comp1) {
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


void MatMserAnalyzer::merge_component_into(MatMserAnalyzer::Component &comp1, MatMserAnalyzer::Component &comp2) {
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

void MatMserAnalyzer::extend_history_(MatMserAnalyzer::Component &component) {
    component.history.push_back(component.stats);
    component.level_history.push_back(component.level);

    assert(component.history.size() == component.level_history.size());

    calculate_stability(component);
    check_component_(component);

}

void MatMserAnalyzer::calculate_stability(MatMserAnalyzer::Component &comp) {
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

void MatMserAnalyzer::check_component_(MatMserAnalyzer::Component &comp) {
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


void MatFindMserAnalyzer::merge_component_into(MatFindMserAnalyzer::Component &comp1, MatFindMserAnalyzer::Component &comp2) {
    assert(comp1.level != comp2.level);

    // take the history of the winner
    if (comp1.stats.N > comp2.stats.N) {
        extend_history_(comp1);
        comp2.history = std::move(comp1.history);
        comp2.level_history = std::move(comp1.level_history);
    }

    comp2.stats.merge(comp1.stats);
}

void MatFindMserAnalyzer::extend_history_(MatFindMserAnalyzer::Component &component) {
    component.history.push_back(component.stats);
    component.level_history.push_back(component.level);

    assert(component.history.size() == component.level_history.size());

    calculate_stability(component);
    check_component_(component);
}

void MatFindMserAnalyzer::check_component_(MatFindMserAnalyzer::Component &comp) {
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
        cost += weight_mean_val_ * compute_error_(examinee.mean_val, target_stats_.mean_val, 255);
        cost += .5*weight_interval_ * compute_error_(examinee.min_val,  target_stats_.min_val, 255);
        cost += .5*weight_interval_ * compute_error_(examinee.max_val,  target_stats_.max_val, 255);

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

void MatFindMserAnalyzer::calculate_stability(MatFindMserAnalyzer::Component &comp) {
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

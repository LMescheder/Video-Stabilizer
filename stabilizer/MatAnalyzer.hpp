#ifndef MATMSERANALYZER_HPP
#define MATMSERANALYZER_HPP

#include "opencv2/core.hpp"

// ------------------------------------------------------------------------------------------------
// ------------------------------- declarations ---------------------------------------------------
// ------------------------------------------------------------------------------------------------

struct MatComponentStats {
	unsigned int N = 0;
	cv::Point2f mean = cv::Vec2f(0., 0.);
	cv::Matx22f cov = cv::Matx22f(0., 0., 0., 0.);
	cv::Point2i min_point = cv::Point2i(0., 0.);
	cv::Point2i max_point = cv::Point2i(0., 0.);
	uchar min_val = 255;
	uchar max_val = 0;
	float stability = 0.;
	cv::Point2i source = cv::Point2i(0., 0.);

	MatComponentStats() = default;

	MatComponentStats(cv::Point2i point, uchar value)
		: N{1}, mean{point}, min_point{point}, max_point{point},
		  min_val{value}, max_val{value}, source{point} {}

	void merge(const MatComponentStats& comp1) {
		auto& comp2 = *this;
		auto p = float(comp1.N) / float(N + comp2.N);
		auto q = float(comp2.N) / float(N + comp2.N);

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
	}
	void merge(cv::Point2i point, uchar value) {
		merge(MatComponentStats{point, value});
	}
};

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
	using ComponentStats = MatComponentStats;

    struct Component {
        uchar level;
        ComponentStats stats;
        std::vector<ComponentStats> history;
		std::vector<uchar> history_levels;

		Component(uchar value)
			: level{value} {}

		Component(cv::Point2i point, uchar value)
			: level{value}, stats{point, value} {}
    };


    using Result = std::vector<ComponentStats>;

    MatAnalyzer (unsigned int delta) : delta_(delta) {}

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
			comp2.history_levels = std::move(comp1.history_levels);
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

protected:
    Result result_;
    const unsigned int delta_ = 5;


	void extend_history_(Component& component) {
		component.history.push_back(component.stats);
		component.history_levels.push_back(component.level);
		calculate_stability(component);
		check_component_(component);
		assert(component.history.size() == component.history_levels.size());
	}

	void calculate_stability(Component& comp) {
		if(comp.history.size() >= 2*delta_ + 1) {
			auto& comp0 = comp.history.rbegin()[2*delta_];
			auto& comp1 = comp.history.rbegin()[delta_];
			auto& comp2 = comp.history.rbegin()[0];
			auto& level0 = comp.history_levels.rbegin()[2*delta_];
//			auto& level1 = comp.history_levels.rbegin()[delta_];
			auto& level2 = comp.history_levels.rbegin()[0];

			assert(comp0.N < comp1.N && comp1.N < comp2.N);
			comp1.stability = static_cast<float>(comp1.N * std::abs(level2 - level0))/(comp2.N - comp0.N);
			//comp1.stability = static_cast<float>(comp1.N * 2 * delta_)/(comp2.N - comp0.N);
		}
	}


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
		Component (cv::Point2i point, uchar level) : MatAnalyzer<MatMserAnalyzer>::Component(point, level) {}

        unsigned int last_mser_N = 0;
    };

	Component add_component (uchar level) {
		return Component{level};
	}

	Component add_component (cv::Point2i point, uchar level) {
		return Component(point, level);
    }

	void add_node( typename cv::Point2i node, Component& component) {
		component.stats.merge(node, component.level);
	}


	void merge_component_into (Component& comp1, Component& comp2) {
        if (comp1.stats.N > comp2.stats.N )
            comp2.last_mser_N = comp1.last_mser_N;
		MatAnalyzer<MatMserAnalyzer>::merge_component_into(comp1, comp2);
	}

	void check_component_(Component &comp) {
		if (comp.history.size() >= 2*delta_ + 1) {
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

private:
    const unsigned int min_N_;
    const unsigned int max_N_;
    const float min_stability_;
    const float min_diversity_;
};

#endif // MATMSERANALYZER_HPP

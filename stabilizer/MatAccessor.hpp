#ifndef MATACCESSOR_HPP
#define MATACCESSOR_HPP

#include "opencv2/core.hpp"
#include "boost/optional.hpp"
#include <array>

// declarations

// TODO: optimize by using pointers instead of pointers for indexes
// TODO: create function for a size hint (for optimizations in parser and analyzer)
// TODO: use more efficient way to store the mask
// TODO: do more computations in advance (e.g. next index computations)
// TODO: create a reset function to reuse the objects allocations
// TODO: create constructor, getter and setter for member variables
class MatAccessor
{
public:
    using NodeIndex = cv::Point2i;
    using Node =cv::Point2i;
    using Value = uchar;
    using Data = cv::Mat;

    class PriorityQueue;



    MatAccessor(Data data, bool inverted=false)
        : data_(data), inverted_(inverted) {
        mask_ = cv::Mat::zeros(data.rows, data.cols, CV_8U);
    }

	void reset() {
		mask_ = 0;
	}

    Value inf () const {
        if (!inverted_)
            return 255;
        else
            return 0;
    }

    bool less(Value val1, Value val2) const {
        if (!inverted_)
            return val1 < val2;
        else
            return val2 < val1;
    }

    NodeIndex get_source() {
        auto new_node = NodeIndex{0, 0};
        mask_.at<uchar>(new_node) = 1;
        return NodeIndex{0, 0};
    }

    Node node(NodeIndex node_idx) {
        return node_idx;
    }

    Value value (NodeIndex node_idx) {
        return data_.at<uchar>(node_idx);
    }


    NodeIndex get_index (const Node& node){
        return node;
    }

    boost::optional<NodeIndex> get_next_neighbor (NodeIndex node);

private:
    Data data_;
    cv::Mat mask_;
    bool inverted_;

};

// TODO: store priority queue contiguously (more efficient?)
// TODO: better way to pop?

class MatAccessor::PriorityQueue {
public:
    PriorityQueue (bool inv=false)
        : inverted_(inv) {}

	void reset_() {
		minimum_ = 255;
		for (auto& stack : points_)
			stack.clear();
	}

    void push(cv::Point2i point, uchar value) {
        if (inverted_)
            value = 255 - value;
        points_[value].push_back(point);
        minimum_ = std::min(minimum_, value);

    }

    boost::optional<cv::Point2i> pop();

private:
    std::array<std::vector<cv::Point2i>, 256> points_;
    uchar minimum_ = 255;
    bool inverted_ = false;
};

// definitions
boost::optional<MatAccessor::NodeIndex> MatAccessor::get_next_neighbor(MatAccessor::NodeIndex node) {
    uchar& mask_value = mask_.at<uchar>(node);

    while (mask_value < 5) {
        NodeIndex next_node;

        switch (mask_value) {
        case 0:
            next_node = node;
        case 1:
            next_node = NodeIndex{node.x + 1, node.y};
            break;
        case 2:
            next_node = NodeIndex{node.x, node.y + 1};
            break;
        case 3:
            next_node = NodeIndex{node.x - 1, node.y};
            break;
        case 4:
            next_node = NodeIndex{node.x, node.y - 1};
            break;
        }
        ++mask_value;

        if (   0 <= next_node.x  && next_node.x < data_.cols
               && 0 <= next_node.y  && next_node.y < data_.rows ) {
            uchar& next_mask_value = mask_.at<uchar>(next_node);
            if (next_mask_value == 0) {
                ++next_mask_value;
                return next_node;
            }
        }
    }
    return boost::none;
}


boost::optional<cv::Point2i> MatAccessor::PriorityQueue::pop() {
    if (points_[minimum_].empty())
        return boost::none;
    else {
        auto next_point = points_[minimum_].back();
        points_[minimum_].pop_back();
        while (minimum_ < 255 && points_[minimum_].empty())
            ++minimum_;
        return next_point;
    }
}

#endif // MATACCESSOR_HPP






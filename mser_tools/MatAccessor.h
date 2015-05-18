#ifndef MATACCESSOR_HPP
#define MATACCESSOR_HPP

#include "opencv2/opencv.hpp"
#include "boost/optional.hpp"
#include <array>

// declarations



/** \brief Implementation of GraphAccessor concept to access an opencv Mat object.
 *
 * This class can be used to parametrize the ComponentTreeParser class to access
 * a gray level image given as an opencv Mat object. It assumes, that the image has one
 * channel and its datatype is uchar.
 *
 * \todo optimize by using pointers instead of points for indexes
 * \todo create function for a size hint (for optimizations in parser and analyzer)
 * \todo use more efficient way to store the mask
 * \todo do more computations in advance (e.g. next index computations)
 * \todo create a reset function to reuse the objects allocations
 * \todo create constructor, getter and setter for member variables
 */
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
        assert(data.type() == CV_8U);
        mask_ = cv::Mat::zeros(data.rows, data.cols, CV_8U);
    }

    MatAccessor(Data data, cv::Point2i offset, bool inverted=false)
        : MatAccessor(data, inverted) {
        offset_ = offset;
    }

    void reset() {
        mask_.setTo(0.);
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
        return new_node;
    }

    Node node(NodeIndex node_idx) const {
        return offset_ + node_idx;
    }

    Value value (NodeIndex node_idx) const {
        return data_.at<uchar>(node_idx);
    }


    NodeIndex get_source (const Node& node) {
        NodeIndex new_node = node - offset_;
        mask_.at<uchar>(new_node) = 1;
        return new_node;
    }

    boost::optional<NodeIndex> get_next_neighbor (NodeIndex node);

private:
    Data data_;
    cv::Mat mask_;
    bool inverted_;
    cv::Point2i offset_;

};



/** \brief A priority queue to efficiently get the next pixel with the lowest
 *         gray value.
 *  \todo store priority queue contiguously (more efficient?)
*   \todo better way to pop?
 */
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

#endif // MATACCESSOR_HPP






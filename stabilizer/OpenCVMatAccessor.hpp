#ifndef OPENCVMATACCESSOR_HPP_
#define OPENCVMATACCESSOR_HPP_

#include "opencv2/core.hpp"
#include "boost/optional.hpp"
#include <array>

class OpenCVMatAccessor
{
public:
    using NodeIndex = cv::Point2i;
    using Value = uchar;
    using Data = cv::Mat;

    static const Value inf = 255;

    OpenCVMatAccessor(Data data) {
        data_ = data;
        mask_ = cv::Mat::zeros(data.rows, data.cols, CV_8U);
    }

    NodeIndex get_source() {
        auto new_node = NodeIndex{0, 0};
        mask_.at<uchar>(new_node) = 1;
        return NodeIndex{0, 0};
    }

    Value value (NodeIndex node) {
        uchar result  = data_.at<uchar>(node);
        return result;
    }


    boost::optional<NodeIndex> get_next_neighbor (NodeIndex node) {
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

private:
    Data data_;
    cv::Mat mask_;
};

class OpenCVMatMserAnalyzer {
public:
    struct Component {
        unsigned int age = 0;
        unsigned int N = 0;
        cv::Point2f mean{0., 0.};
    };

    using Result = std::vector<Component>;

    void add_node( typename cv::Point2i node, Component& component) {
        //component.points.push_back(node);
        auto area = float(component.N);
        auto factor = area / (area + 1);
        component.mean.x = node.x/(area + 1) + factor * component.mean.x;
        component.mean.y = node.y/(area + 1) + factor * component.mean.y;
        ++component.age;

        // todo check maximal stability
    }

    Component add_component () {
        return Component{};
    }

    void merge_component_into(const Component& comp1, Component& comp2) {
        auto area1 = float(comp1.N);
        auto area2 = float(comp2.N);
        auto area12 = area1 + area2;
        comp2.mean.x = area1/area12 * comp1.mean.x + area2/area12 * comp2.mean.x;
        comp2.mean.y = area1/area12 * comp1.mean.y + area2/area12 * comp2.mean.y;
        comp2.N += comp1.N;
    }

    // TODO: still makes a deep copy -> has to be optimized (would calling std::move be save?
    Result get_result() { return result_; }

private:
    Result result_;
};


class OpenCVMatPriorityQueue {
public:
    void push(cv::Point2i point, uchar value) {
        points_[value].push_back(point);
        //minimum_ = std::min(minimum_, value);
    }

    boost::optional<cv::Point2i> pop() {
        for (auto& vec : points_)
            if (!vec.empty()) {
                cv::Point2i next_point = vec.back();
                vec.pop_back();
                return next_point;
            }
        return boost::none;
    }

private:
    std::array<std::vector<cv::Point2i>, 255> points_;
    //uchar minimum_ = 0;

};

#endif // OPENCVMATACCESSOR_HPP_

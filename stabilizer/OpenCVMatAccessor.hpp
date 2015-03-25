#ifndef OPENCVMATACCESSOR_HPP_
#define OPENCVMATACCESSOR_HPP_

#include "opencv2/core.hpp"

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

    bool has_more_neighbors (NodeIndex node) {
        // point has status 5 if accessible and all 4 neighbors explored
        return mask_.at<uchar>(node) < 5;
    }
    bool get_next_neighbor (NodeIndex node, NodeIndex& neighor_node) {
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
                    neighor_node = next_node;
                    return true;
                 }
            }
        }
        return false;
    }

private:
    Data data_;
    cv::Mat mask_;
};

class OpenCVMatMserAnalyzer {
public:
    struct Component {
        uchar level_;
        uchar level() {return level_;}
        void set_level(uchar level) {level_ = level;}
    };

    using Result = std::vector<std::vector<cv::Point2i>>;

    void add_node( typename cv::Point2i node, Component component) { }
    Component add_component (uchar level) { return Component{level}; }
    Component merge_components (Component comp1, Component comp2) {return Component{std::max(comp1.level_, comp2.level_)};}


    Result get_result() {return {}; }
};

#endif // OPENCVMATACCESSOR_HPP_

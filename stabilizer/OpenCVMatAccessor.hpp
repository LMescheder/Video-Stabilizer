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

    OpenCVMatAccessor(Data data) : data_{data}{
        mask_ = cv::Mat::zeros(data.rows, data.cols, CV_8U);
    }

    NodeIndex get_source() {
        return NodeIndex{0, 0};
    }

    Value value (NodeIndex node) {
        return data_.at<uchar>(node);
    }

    bool has_more_neighbors (NodeIndex node) {
        // point has status 5 if accessible and all 4 neighbors explored
        return mask_.at<uchar>(node) < 5;
    }
    NodeIndex get_next_neighbor (NodeIndex node) {
        uchar& mask_value = mask_.at<uchar>(node);

        CV_Assert(1 < mask_value && mask_value < 5 );

        while (mask_value < 5) {
            int dx = dx_[mask_value-1];
            int dy = dy_[mask_value-1];

            auto next_node = NodeIndex{node.y + dy, node.x + dx};
            ++mask_value;

            if (   0 <= next_node.x  && next_node.x < data_.cols
                && 0 <= next_node.y  && next_node.y < data_.rows )
                continue;

            uchar& next_mask_value = mask_.at<uchar>(next_node);
            if (next_mask_value == 0) {
                ++next_mask_value;
                return next_node;
             }
        }
        // TODO: throw exception instead of return node?
        return node;
    }

private:
    const int dx_[4] = {1, -1, 0, 0};
    const int dy_[4] = {0,  0, 1, -1};
    const Data& data_;
    cv::Mat mask_;
};


#endif // OPENCVMATACCESSOR_HPP_

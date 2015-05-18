#include "MatAccessor.h"


boost::optional<MatAccessor::NodeIndex> MatAccessor::get_next_neighbor(MatAccessor::NodeIndex node) {
    uchar& mask_value = mask_.at<uchar>(node);

    assert(1 <= mask_value);
    assert(mask_value <= 5);

    while (mask_value < 5) {
        NodeIndex next_node;

        switch (mask_value) {
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
                next_mask_value = 1;
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

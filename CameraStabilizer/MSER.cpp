#include "MSER.hpp"
#include "Tree.hpp"
#include "opencv2/core.hpp"
#include <queue>
#include <stack>
#include <vector>
#include <tuple>

// usings
using cv::Mat; using cv::Point2i;
using std::vector; using std::priority_queue; using std::stack;
using std::tuple;

// declarations
Point2i next_pixel(Point2i pixel, int height, int width);

struct PointLevelPair {
    Point2i point;
    int level;
};

bool operator< (PointLevelPair pl1, PointLevelPair pl2) {
    return (pl1.level < pl2.level);
}

// class implementation
Tree<Component> ComponentTreeBuilder::operator() (Mat image) const {
    // data structures
    Mat accessible_mask = Mat::zeros(image.size(), CV_8U);
    priority_queue<PointLevelPair> boundary_pixels;
    stack<Tree<Component>> components;

    unsigned int current_level = 0;

    return Tree<Component>{};
}

// utility functions
tuple<Point2i, int> next_pixel (Point2i pixel, int state, int height, int width) {
    switch (state) {
    case 0:
    break;
    case 1:
    break;
    case 2:
    break;
    case 3:
    break;
    default:
    break;
    }
}

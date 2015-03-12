#ifndef CAMERASTABILIZER_MSER_H_
#define CAMERASTABILIZER_MSER_H_

#include <vector>
#include "opencv2/core.hpp"
#include "Tree.hpp"


struct ComponentStatus {
    unsigned int level;
    unsigned int area;
    cv::Point2f mean;
    cv::Matx22f covariance;
};

struct Component {
    std::vector<cv::Point2i> points;
    unsigned int minlevel, maxlevel;
    std::vector<ComponentStatus> history;
};


class ComponentTreeBuilder {
    public:
    ComponentTreeBuilder ();
    
    Tree<Component> operator() (cv::Mat image) const;
};



#endif // CAMERASTABILIZER_MSER_H_

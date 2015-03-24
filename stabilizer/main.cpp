#include "ComponentTreeParser.hpp"
#include "OpenCVMatAccessor.hpp"
#include "opencv2/core.hpp"
#include <iostream>

struct G {
    using Value = int;
    using Data = int;
    struct NodeIndex {
        Value value = 0;
    };
    bool has_more_neighbors(NodeIndex node) { return false; }
    NodeIndex get_next_neighbor(NodeIndex node) { return node; }
    Value value(NodeIndex node) {return node.value;}
    static const Value inf = 1000;

    G (const Data&) {}


    NodeIndex get_source () { return NodeIndex{}; }

};


template <typename G>
struct A {
    struct Component {
        typename G::Value level_;
        typename G::Value level() {return level_;}
        void set_level(typename G::Value level) {level_ = level;}
    };
    using Result = int;

    void add_node( typename G::NodeIndex node, Component component) { }
    Component add_component () { return Component{}; }
    Component merge_components (Component comp1, Component comp2) {return Component{};}


    Result get_result() {return Result{}; }
};

int main() {
    cv::Mat data(100, 100, CV_8U);
    ComponentTreeParser<OpenCVMatAccessor, A<OpenCVMatAccessor>> test;
    test(data);
}




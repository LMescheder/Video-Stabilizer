#include "ComponentTreeParser.hpp"
#include "OpenCVMatAccessor.hpp"

/*
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
*/
#include "opencv2/opencv.hpp"

#include <iostream>
#include <chrono>
#include <vector>

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
    Component add_component (typename G::Value level) { return Component{level}; }
    Component merge_components (Component comp1, Component comp2) {return Component{std::max(comp1.level_, comp2.level_)};}


    Result get_result() {return Result{}; }
};

int main() {
    const char* path = "/home/lars/Education/University/Semester_10_Lausanne/CV_Project/work/build/data/Lena.png";
    cv::Mat data = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    ComponentTreeParser<OpenCVMatAccessor, A<OpenCVMatAccessor>> test;


    auto start = std::chrono::high_resolution_clock::now();
    test(data);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Operations took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    //start = std::chrono::high_resolution_clock::now();

    auto mser = cv::MSER::create();
    std::vector<std::vector<cv::Point>> points;
    std::vector<cv::Rect> boxes;

    start = std::chrono::high_resolution_clock::now();
    mser->detectRegions(data, points, boxes) ;
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Operations took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

}




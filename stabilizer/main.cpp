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


int main() {
    const char* path = "/home/lars/Education/University/Semester_10_Lausanne/CV_Project/work/build/data/Lena.png";
    cv::Mat data = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
    ComponentTreeParser<OpenCVMatAccessor, OpenCVMatMserAnalyzer, OpenCVMatPriorityQueue> test;


    auto start = std::chrono::high_resolution_clock::now();
    test(data);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Operations took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    //start = std::chrono::high_resolution_clock::now();

    /*
    auto mser = cv::MSER::create();
    std::vector<std::vector<cv::Point>> points;
    std::vector<cv::Rect> boxes;

    start = std::chrono::high_resolution_clock::now();
    mser->detectRegions(data, points, boxes) ;
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Operations took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    */

}




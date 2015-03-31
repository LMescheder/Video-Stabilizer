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

#include "ComponentTreeParser.hpp"
#include "MatAnalyzer.hpp"
#include "MatAccessor.hpp"

int main() {
    const char* path = "/home/lars/Education/University/Semester_10_Lausanne/CV_Project/work/build/data/Lena.png";
    cv::Mat im = cv::imread(path, CV_LOAD_IMAGE_COLOR);

    cv::Mat data;
    cv::cvtColor(im, data, CV_BGR2GRAY);


    ComponentTreeParser<MatAccessor, MatMserAnalyzer, MatPriorityQueue> test;


    auto start = std::chrono::high_resolution_clock::now();
    auto result = test(data);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Operations took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    //start = std::chrono::high_resolution_clock::now();



    auto mser = cv::MSER::create();
    std::vector<std::vector<cv::Point>> cv_msers;
    std::vector<cv::Rect> boxes;

    start = std::chrono::high_resolution_clock::now();
    mser->detectRegions(data, cv_msers, boxes) ;
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Operations took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    cv::Mat output_im = data.clone();
    for (auto& mser : result)
        cv::circle(output_im, cv::Point(mser.mean), 2, cv::Scalar(255, 0, 0));



    for (auto& mser : cv_msers) {
        std::vector<cv::Point> hull;
        cv::convexHull(mser, hull);
        cv::polylines(output_im, hull, true, cv::Scalar(0, 255, 0));
    }


    cv::imshow("MSER", output_im);

    cv::waitKey(0);

}




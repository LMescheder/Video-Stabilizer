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
#include "MatMser.hpp"

void test0 ();
void test1 ();
void test2 ();
void test3 ();
void test4();
void test5();

int main() {
    //test0();
    //test1();
    //test2();
    //test3();
    //test4();
    test5();
}


void test0 () {
    const char* path = "/home/lars/Education/University/Semester_10_Lausanne/CV_Project/work/build/data/Lena.png";
    cv::Mat im = cv::imread(path, CV_LOAD_IMAGE_COLOR);

    cv::Mat data;
    cv::cvtColor(im, data, CV_BGR2GRAY);


    MatMser mymser;


    auto start = std::chrono::high_resolution_clock::now();
    auto result = mymser.detect_msers(data);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Operations took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
}


void test1 () {
    const char* path = "../../data/Lena.png";
    cv::Mat im = cv::imread(path, CV_LOAD_IMAGE_COLOR);

    cv::Mat data;
    cv::cvtColor(im, data, CV_BGR2GRAY);


    MatMser  mser_detector(2, 60, 14400, 30.f, .2f, 40.f, 5e2);


    auto start = std::chrono::high_resolution_clock::now();
    auto result = mser_detector.detect_msers_points(data);
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

    cv::Mat output_im1 = im.clone();
    cv::Mat output_im2 = im.clone();
    for (auto& mser : result) {
        //cv::circle(output_im, cv::Point(mser.mean), 2, cv::Scalar(255, 0, 0));

        std::vector<cv::Point> hull;
        cv::convexHull(mser, hull);
        cv::polylines(output_im1, hull, true, cv::Scalar(0, 255, 0));
        }


    for (auto& mser : cv_msers) {
        std::vector<cv::Point> hull;
        cv::convexHull(mser, hull);
        cv::polylines(output_im2, hull, true, cv::Scalar(0, 255, 0));
    }


    cv::imshow("my MSER", output_im1);
    cv::imshow("opencv MSER", output_im2);
    cv::waitKey(0);
}

void test2()
{
    std::string filename = "../../data/shop.avi";
    cv::VideoCapture cap(filename);

    //if(!cap.isOpened())
    //   return -1;

    cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );

    MatMser  mser_detector(2, 60, 14400, 30.f, .2f, 40.f, 5e2);

    while (true) {
        cv::Mat frame;
        cap >> frame;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        auto result = mser_detector.detect_msers(gray);

        for (auto& mser : result) {
            cv::circle(frame, cv::Point(mser.mean), 2, cv::Scalar(255, 0, 0));
            cv::rectangle(frame, mser.min_point, mser.max_point, cv::Scalar(255, 0, 0));
        }

        cv::imshow("Video", frame);

        if (cv::waitKey(1) >= 0) {
            break;
        }
    }

}

void test3()
{
    std::string filename = "../../data/basket.avi";
    cv::VideoCapture cap(filename);

    //if(!cap.isOpened())
    //   return -1;

    cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );

    MatMser mymser(10, 200, 14400, 10.f, .2f);

    while (true) {
        cv::Mat frame;
        cap >> frame;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        auto result = mymser.detect_msers_points(gray);

        for (auto& mser : result) {
            std::vector<cv::Point> hull;
            cv::convexHull(mser, hull);
            cv::polylines(frame, hull, true, cv::Scalar(0, 255, 0));
        }

        cv::imshow("Video", frame);

        if (cv::waitKey(1) >= 0) {
            break;
        }
    }

}

void test4()
{
   auto filename = "../../data/basket.avi";
   cv::VideoCapture cap(filename);
   //cv::VideoCapture cap(0);

    //if(!cap.isOpened())
    //   return -1;

    cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );

    MatMser  mser_detector(2, 10, 7500, 30.f, .2f, 30.f, 1e3);
    cv::Mat frame;
    cv::Mat gray;
    cap >> frame;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    auto up_msers = mser_detector.detect_msers(gray, MatMser::upwards);
    auto down_msers = mser_detector.detect_msers(gray, MatMser::downwards);

    bool recompute = false;

    for (int i=1; true; i = (i+1) % 10) {
        cap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        up_msers = mser_detector.retrieve_msers(gray, up_msers, false);
        down_msers = mser_detector.retrieve_msers(gray, down_msers, true);

        cv::Mat out_frame = frame.clone();

        for (auto& mser : up_msers) {
            if (mser.N > 0) {
                auto points = mser_detector.stats_to_points(mser, gray);
                std::vector<cv::Point> hull;
                cv::convexHull(points, hull);
                cv::polylines(out_frame, hull, true, cv::Scalar(0, 255, 0));
                cv::circle(out_frame, mser.mean, 3, cv::Scalar(255, 255, 0));
            }
        }

        for (auto& mser : down_msers) {
            if (mser.N > 0) {
                auto points = mser_detector.stats_to_points(mser, gray);
                std::vector<cv::Point> hull;
                cv::convexHull(points, hull);
                cv::polylines(out_frame, hull, true, cv::Scalar(0, 255, 0));
                cv::circle(out_frame, mser.mean, 3, cv::Scalar(255, 255, 0));
            }
        }

        // recompute
        if (recompute && i == 0) {
            up_msers = mser_detector.detect_msers(gray, MatMser::upwards);
            down_msers = mser_detector.detect_msers(gray, MatMser::downwards);
        }
        cv::imshow("Video", out_frame);

        if (cv::waitKey(1) >= 0) {
            break;
        }
    }

}

void test5()
{
    std::string filename = "../../data/path.mp4";
    cv::VideoCapture cap(filename);

    //if(!cap.isOpened())
    //   return -1;

    cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );

    MatMser mser_detector(10, 200, 14400, 10.f, .2f);
    cv::Mat frame;
    cap >> frame;
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    auto msers = mser_detector.detect_msers(gray);

    std::vector<cv::Point2f> p0 = mser_detector.extract_means(msers);
    std::vector<cv::Point2f> p1 = p0;
    std::vector<float> err(p1.size(), 0.f);
    std::vector<bool> status(p1.size(), true);

    std::vector<cv::Point2f> p2(p1.size());

    while (true) {
        auto oldgray = gray.clone();

        cap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::calcOpticalFlowPyrLK(oldgray, gray, p1, p2, status, err);

        for (auto& point : p2)
            cv::circle(frame, point, 2., cv::Scalar(0, 255, 0));

        cv::imshow("Video", frame);

        if (cv::waitKey(1) >= 0) {
            break;
        }
        p1 = p2;
    }

}

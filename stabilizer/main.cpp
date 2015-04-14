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
#include "MatMserTracker.hpp"
#include "VideoStabilizer.hpp"
void test0 (std::string input);
void test1 (std::string input);
void test2 (std::string input);
void test3 (std::string input);
void test4 (std::string input);
void test5 (std::string input);
void test6 (std::string input);

int main(int argc, char** argv) {
    std::string filename;
    if (argc < 2) {
        std::cout << "Usage: stabilizer [filename]\n";
        return 1;
    }
    filename = std::string("../../data/") + argv[1];
    //test0(filename);
    //test1(filename);
    //test2(filename);
    //test3(filename);
    //test4(filename);
    //test5(filename);
    test6(filename);
}


void test0 (std::string input) {
    cv::Mat im = cv::imread(input, CV_LOAD_IMAGE_COLOR);

    cv::Mat data;
    cv::cvtColor(im, data, CV_BGR2GRAY);


    MatMser mymser;


    auto start = std::chrono::high_resolution_clock::now();
    auto result = mymser.detect_msers(data);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Operations took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
}


void test1 (std::string input) {
    cv::Mat im = cv::imread(input, CV_LOAD_IMAGE_COLOR);

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

void test2(std::string input)
{
    cv::VideoCapture cap(input);

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

void test3(std::string input)
{
    cv::VideoCapture cap(input);

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

void test4(std::string input)
{
   cv::VideoCapture cap(input);
   //cv::VideoCapture cap(0);

    //if(!cap.isOpened())
    //   return -1;

    cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );

    MatMser mser_detector(2, 50, 3000, 150.f, .1f, 100.f, 1e2);;
    MatMserTracker  tracker(mser_detector);

    cv::Mat frame;
    cv::Mat gray;
    while (true) {
        cap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        tracker.update(gray);

        cv::Mat out_frame = frame.clone();

        for (auto& mser : tracker.up_msers()) {
            if (mser.N > 0) {
                auto points = MatMser::stats_to_points(mser, gray);
                std::vector<cv::Point> hull;
                cv::convexHull(points, hull);
                cv::polylines(out_frame, hull, true, cv::Scalar(0, 255, 0), 1.5);
                cv::circle(out_frame, mser.mean, 3, cv::Scalar(255, 255, 0));
            }
        }


        for (auto& mser : tracker.down_msers()) {
            if (mser.N > 0) {
                auto points = MatMser::stats_to_points(mser, gray);
                std::vector<cv::Point> hull;
                cv::convexHull(points, hull);
                cv::polylines(out_frame, hull, true, cv::Scalar(0, 255, 0), 1.5);
                cv::circle(out_frame, mser.mean, 3, cv::Scalar(255, 255, 0));
            }
        }
        /*
        for (auto& mean : tracker.up_means())
            cv::circle(out_frame, mean, 3, cv::Scalar(0, 0, 255));

        for (auto& mean : tracker.down_means())
            cv::circle(out_frame, mean, 3, cv::Scalar(0, 0, 255));
        */
        cv::imshow("Video", out_frame);

        if (cv::waitKey(1) >= 0) {
            break;
        }
    }

}

void test5(std::string input)
{
    cv::VideoCapture cap(input);

    //if(!cap.isOpened())
    //   return -1;

    cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );

    MatMser mser_detector(2, 50, 3000, 75.f, .1f, 50.f, 1e3);;
    cv::Mat frame;
    cap >> frame;
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    auto msers = mser_detector.detect_msers(gray);

    std::vector<cv::Point2f> p0 = mser_detector.extract_means(msers);
    std::vector<cv::Point2f> p1, p2;
    cv::Mat err, status;

    p1 = p0;

    while (true) {
        auto oldgray = gray.clone();

        cap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::calcOpticalFlowPyrLK(oldgray, gray, p1, p2, status, err, cv::Size(50, 50), 2);

        for (auto& point : p2)
            cv::circle(frame, point, 2., cv::Scalar(0, 255, 0));

        cv::imshow("Video", frame);

        if (cv::waitKey(1) >= 0) {
            break;
        }
        p1 = p2;
    }

}

void test6(std::string input)
{
   bool show = true;
    MatMser mser_detector(5, 50, 3000, 25.f, .1f, 15.f, 5.e0);;

   cv::VideoCapture cap(input);

   int frame_width=   cap.get(CV_CAP_PROP_FRAME_WIDTH);
   int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);
   int N = cap.get(CV_CAP_PROP_FRAME_COUNT);
   cv::VideoWriter vout("../../output/output.avi",
                           CV_FOURCC('M','J','P','G'),
                           20,
                           cv::Size(frame_width,frame_height),
                           true);
   cv::VideoWriter voutr("../../output/output_regions.avi",
                           CV_FOURCC('M','J','P','G'),
                           20,
                           cv::Size(frame_width,frame_height),
                           true);
   //cv::VideoCapture cap(0);

    //if(!cap.isOpened())
    //   return -1;

    if (show) {
        cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );
        cv::namedWindow( "Stabilized", CV_WINDOW_AUTOSIZE );
    }

    cv::Mat frame;
    cv::Mat gray;
    if (!cap.read(frame))
        return;

    MatMserTracker  tracker(mser_detector);
    VideoStabilizer stabilizer(tracker, frame);

    int i = 0;
    while (cap.isOpened()) {
        auto start = std::chrono::high_resolution_clock::now();
        if (!cap.read(frame))
            break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat stabilized = stabilizer.stabilze_next(frame);

        cv::Mat out_frame = frame.clone();

        for (auto& mser : stabilizer.msers()) {
            if (mser.N > 0) {
                auto points = MatMser::stats_to_points(mser, gray);
                std::vector<cv::Point> hull;
                cv::convexHull(points, hull);
                cv::polylines(out_frame, hull, true, cv::Scalar(0, 255, 0), 1.5);
                cv::circle(out_frame, mser.mean, 3, cv::Scalar(255, 255, 0));
            }
        }

        vout.write(stabilized);
        voutr.write(out_frame);

        auto end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>
                            (end - start).count();

        ++i;
        std::cout << i << "/" << N << " done! ( " << time << "ms )" << std::endl;


        if (show) {


            cv::imshow("Video", out_frame);
            cv::imshow("Stabilized", stabilized);


            if (cv::waitKey(1) >= 0) {
                break;
            }
        }

    }

    cap.release();
    vout.release();
    voutr.release();
    if (show)
        cv::destroyAllWindows();
}

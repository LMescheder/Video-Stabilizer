#include "opencv2/opencv.hpp"

#include <iostream>
#include <chrono>
#include <vector>
#include <memory>

#include "mser_tools/MatMser.hpp"
#include "stabilizer/Stabilizer.hpp"
#include "stabilizer/MserStabilizer.hpp"
#include "stabilizer/PointStabilizer.hpp"

void run_stabilizer (std::string input, std::string output, std::string output_regions);

void visualize_points (cv::Mat& image, const std::vector<MatComponentStats>& msers, bool orientation=true);
void visualize_stabilization_points (cv::Mat&  image, const MserStabilizer& stabilizer, bool lines=true);
void visualize_regions_hulls (cv::Mat& image, const std::vector<MatComponentStats>& msers, const cv::Mat& gray);
void visualize_regions_cov (cv::Mat& image, const std::vector<MatComponentStats>& msers);
void visualize_regions_box (cv::Mat& image, const std::vector<MatComponentStats>& msers);


int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "Usage: stabilizer [input] [output] [output_regions]\n";
        return 1;
    }

    std::string input = argv[1];
    std::string output = argv[2];
    std::string output_regions = argv[3];
    run_stabilizer(input, output, output_regions);
}

void run_stabilizer(std::string input, std::string output, std::string output_regions)
{
   bool show = true;
   MatMser mser_detector(5, 50, 3000, 50.f, .1f, 25.f, 1.e0);;

   cv::VideoCapture cap(input);

   int frame_width=   cap.get(CV_CAP_PROP_FRAME_WIDTH);
   int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);
   int N = cap.get(CV_CAP_PROP_FRAME_COUNT);
   cv::VideoWriter vout(output,
                           CV_FOURCC('M','J','P','G'),
                           20,
                           cv::Size(frame_width,frame_height),
                           true);
   cv::VideoWriter voutr(output_regions,
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

    //std::unique_ptr<Stabilizer> stabilizer  (new MserStabilizer(mser_detector, frame));
    std::unique_ptr<Stabilizer> stabilizer  (new PointStabilizer(frame, Stabilizer::homography));

    int i = 0;
    while (cap.isOpened()) {
        auto start = std::chrono::high_resolution_clock::now();
        if (!cap.read(frame))
            break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat stabilized = stabilizer->stabilize_next(frame);

        cv::Mat out_frame = stabilizer->visualization();

        //auto msers = stabilizer.msers();
        //visualize_points(out_frame, msers);
        //visualize_stabilization_points(out_frame, stabilizer);
        //visualize_regions_hulls(out_frame, msers, gray);
        //visualize_regions_box(out_frame, msers);
        //visualize_regions_cov(out_frame, msers);


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

void visualize_points (cv::Mat& image, const std::vector<MatComponentStats>& msers, bool orientation) {
    for (auto& mser : msers)
        if (mser.N > 0) {
            cv::circle(image, mser.mean, 3, cv::Scalar(255, 255, 0));
            if (orientation) {
                cv::Point2f dir(std::cos(mser.angle), std::sin(mser.angle));
                cv::line(image, mser.mean, mser.mean + 10 * dir, cv::Scalar(255, 255, 0), 1.);
            }
        }
}

void visualize_stabilization_points (cv::Mat&  image, const MserStabilizer& stabilizer, bool lines) {
    for (std::size_t i=0; i<stabilizer.points().size(); ++i) {
        cv::Point2f point = stabilizer.points()[i];
        cv::Point2f point0 =  stabilizer.points0()[i];
        cv::circle(image, point, 3, cv::Scalar(255, 0, 255));
        if (lines)
            cv::line(image, point, point0, cv::Scalar(255, 0, 255), 1.);
    }
}

void visualize_regions_hulls (cv::Mat& image, const std::vector<MatComponentStats>& msers, const cv::Mat& gray){
    for (auto& mser : msers)
        if (mser.N > 0) {
            auto points = MatMser::stats_to_points(mser, gray);
            std::vector<cv::Point> hull;
            cv::convexHull(points, hull);
            cv::polylines(image, hull, true, cv::Scalar(0, 255, 0), 1.5);
        }
}

void visualize_regions_cov (cv::Mat& image, const std::vector<MatComponentStats>& msers){
    for (auto& mser : msers)
        if (mser.N > 0) {
            // compute eigenvalue and eigenvectors
            cv::Mat eigenvals, eigenvecs;
            cv::eigen(mser.cov, true, eigenvals, eigenvecs);

            // get angle of first eigenvector
            float angle = atan2(eigenvecs.at<float>(0,1), eigenvecs.at<float>(0,0));

            // convert angle to degrees
            angle = 180*angle/3.14159265359;

            // calculate axis length
            float axis1_length = 2*sqrt(eigenvals.at<float>(0));
            float axis2_length = 2*sqrt(eigenvals.at<float>(1));

            // draw ellipse
            cv::ellipse(image, mser.mean, cv::Size(axis1_length, axis2_length), angle, 0, 360, cv::Scalar(255, 0, 255), 1.5);

        }
}

void visualize_regions_box (cv::Mat& image, const std::vector<MatComponentStats>& msers){
    for (auto& mser : msers)
        if (mser.N > 0)
            cv::rectangle(image, mser.min_point, mser.max_point, cv::Scalar(0, 165, 255), 1.5);
}



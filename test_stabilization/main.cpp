#include "opencv2/opencv.hpp"

#include <iostream>
#include <chrono>
#include <vector>
#include <memory>

#include "mser_tools/MatMser.hpp"
#include "stabilizer/Stabilizer.hpp"
#include "stabilizer/MserStabilizer.hpp"
#include "stabilizer/PointStabilizer.hpp"
#include "stabilizer/utilities.hpp"

void run_stabilizer (std::string input, std::string output, std::string output_regions);

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
   MatMser mser_detector(5, 50, 3000, 50.f, .1f, 25.f, 1.e-2);;

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

    std::unique_ptr<Stabilizer> stabilizer;
    stabilizer.reset(new MserStabilizer(mser_detector, frame,
                                        WarpingGroup::homography,
                                        MserStabilizer::visualize_means | MserStabilizer::visualize_hulls));

    //std::unique_ptr<Stabilizer> stabilizer  (new PointStabilizer(frame, WarpingGroup::homography));

    int i = 0;
    while (cap.isOpened()) {
        auto start = std::chrono::high_resolution_clock::now();
        if (!cap.read(frame))
            break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat stabilized = stabilizer->stabilize_next(frame);

        cv::Mat out_frame = stabilizer->visualization();

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



#include "opencv2/opencv.hpp"

#include <iostream>
#include <chrono>
#include <vector>
#include <memory>

#include "mser_tools/MatMser.h"
#include "stabilizer/Stabilizer.h"
#include "stabilizer/MserStabilizer.h"
#include "stabilizer/PointStabilizer.h"

#include "AccuracyEvaluator.h"

enum class StabilizerType {
    POINT,
    MSER
};

constexpr StabilizerType TYPE = StabilizerType::MSER;
constexpr Stabilizer::Mode MODE = Stabilizer::Mode::WARP_BACK;
constexpr Stabilizer::Warping WARPING = Stabilizer::Warping::HOMOGRAPHY;

int run_stabilizer (std::string input, std::string output, std::string output_regions,
                    StabilizerType type, bool show=true);

/**
 * Expected arguments:
 *   argv[1] : path to the input video sequence
 *   argv[2] : path to where the stabilized video sequence should be written
 *   argv[3] : path to where the visualization video sequence should be written
 */
int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "Usage: stabilizer <path to input video> <path to output video> <path to visualization video>\n";
        return 1;
    }

    std::string input = argv[1];
    std::string output = argv[2];
    std::string output_regions = argv[3];
    return run_stabilizer(input, output, output_regions, TYPE);
}

int run_stabilizer(std::string input, std::string output, std::string output_regions, StabilizerType type, bool show)
{
   MatMser mser_detector(5, 50, 3000, 50.f, .1f, 25.f, 1.e-1);

   cv::VideoCapture cap(input);

   if(!cap.isOpened()) {
      std::cerr << "Could not open input video!" << std::endl;
      return -1;
  }
   /// Get video parameters
   int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
   int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
   int N = cap.get(CV_CAP_PROP_FRAME_COUNT);

   /*
   std::cout << "Frame width: " << frame_width << "\n";
   std::cout << "Frame height = " << frame_width << "\n";
   std::cout << "N = " << frame_width << "\n";
   std::cout << "-- Press Enter to continue -- " << std::endl;
   std::getchar();
   */

   /// Create output video writers
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




    /// If required, create windows for visualization
    if (show) {
        cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );
        cv::namedWindow( "Stabilized", CV_WINDOW_AUTOSIZE );
    }

    /// Read first frame
    cv::Mat frame, frame0;
    cv::Mat gray;
    if (!cap.read(frame0))
        return -1;

    /// Select stabilizer
    std::unique_ptr<Stabilizer> stabilizer;

    if (type == StabilizerType::MSER)
        stabilizer.reset(new MserStabilizer(mser_detector, frame0,
                                            WARPING, MODE,
                                            MserStabilizer::VIS_MEANS | MserStabilizer::VIS_HULLS));
    else if (type == StabilizerType::POINT)
        stabilizer.reset(new PointStabilizer(frame0, WARPING, MODE));
    else
        throw std::logic_error("Stabilizer type not supported!");

    /// Initialize AccuracyEvaluator
    AccuracyEvaluator accuracy_unstabilized (frame0);
    AccuracyEvaluator accuracy_stabilized (frame0);


    /// Do the stabilization
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
        accuracy_unstabilized.evaluate_next(frame);
        accuracy_stabilized.evaluate_next(stabilized);

        if (show) {


            cv::imshow("Video", out_frame);
            cv::imshow("Stabilized", stabilized);
            cv::imshow("diff", stabilized - frame0);

            if (cv::waitKey(1) >= 0) {
                break;
            }
        }

    }

    std::cout << "Accuracy (unstabilized): \n";
    std::cout << "  Minimum mssim = " << accuracy_unstabilized.mssim_stats().min << "\n";
    std::cout << "  Average mssim = " << accuracy_unstabilized.mssim_stats().average << "\n";
    std::cout << "  Maximum mssim = " << accuracy_unstabilized.mssim_stats().max << "\n\n";

    std::cout << "Accuracy (stabilized): \n";
    std::cout << "  Minimum mssim = " << accuracy_stabilized.mssim_stats().min << "\n";
    std::cout << "  Average mssim = " << accuracy_stabilized.mssim_stats().average << "\n";
    std::cout << "  Maximum mssim = " << accuracy_stabilized.mssim_stats().max << "\n\n";

    std::cout << "Accuracy (unstabilized): \n";
    std::cout << "  Minimum psnr = " << accuracy_unstabilized.psnr_stats().min << "\n";
    std::cout << "  Average psnr = " << accuracy_unstabilized.psnr_stats().average << "\n";
    std::cout << "  Maximum psnr = " << accuracy_unstabilized.psnr_stats().max << "\n\n";

    std::cout << "Accuracy (stabilized): \n";
    std::cout << "  Minimum psnr = " << accuracy_stabilized.psnr_stats().min << "\n";
    std::cout << "  Average psnr = " << accuracy_stabilized.psnr_stats().average << "\n";
    std::cout << "  Maximum psnr = " << accuracy_stabilized.psnr_stats().max << "\n\n";

    /// Release resources
    cap.release();
    vout.release();
    voutr.release();
    if (show)
        cv::destroyAllWindows();
}



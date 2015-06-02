//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

#include "opencv2/opencv.hpp"

#include <iostream>
#include <chrono>
#include <vector>
#include <memory>
#include <boost/assign.hpp>

#include "mser_tools/MatMser.h"
#include "stabilizer/Stabilizer.h"
#include "stabilizer/MserStabilizer.h"
#include "stabilizer/PointStabilizer.h"
#include "stabilizer/PixelStabilizer.h"
#include "stabilizer/PatchStabilizer.h"

#include "AccuracyEvaluator.h"
#include "ConfigFileReader.h"



// how to map strings in config to properties
std::map<std::string, Stabilizer::Mode> stabilizer_mode_list = boost::assign::map_list_of
                            ("direct", Stabilizer::Mode::DIRECT)
                            ("track_ref", Stabilizer::Mode::TRACK_REF)
                            ("warp_back", Stabilizer::Mode::WARP_BACK);

std::map<std::string, Stabilizer::Warping> stabilizer_warping_list = boost::assign::map_list_of
                            ("affine", Stabilizer::Warping::AFFINE)
                            ("homography",Stabilizer::Warping::HOMOGRAPHY)
                            ("rigid", Stabilizer::Warping::RIGID)
                            ("rot_homography", Stabilizer::Warping::ROT_HOMOGRAPHY)
                            ("translation", Stabilizer::Warping::TRANSLATION);


// function declarations
int run_stabilizer (std::string input, std::string config_file, std::string output, std::string output_vis, std::string output_log, bool show=true);

std::unique_ptr<Stabilizer> get_stabilizer_from_config(const std::string& config_file, const cv::Mat& frame0);
std::unique_ptr<PointStabilizer> configure_point_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0);
std::unique_ptr<PixelStabilizer> configure_pixel_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0);
std::unique_ptr<PatchStabilizer> configure_patch_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0);
std::unique_ptr<MserStabilizer> configure_mser_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0);

/**
 * Expected arguments:
 *   argv[1] : path to the input video sequence
 *   argv[2] : path to config file
 *   argv[3] : path to output folder
 */
int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "Usage: stabilizer <path to input video> <path to config file> <path to output folder>\n";
        return 1;
    }

    // parse input arguments
    std::string input_file = argv[1];
    std::string config_file = argv[2];
    std::string output_folder = argv[3];

    size_t file_root_start = input_file.find_last_of('/');
    size_t file_root_end = input_file.find_last_of('.');
    std::string file_root = input_file.substr(file_root_start, file_root_end - file_root_start);

    std::string output_file = output_folder + "/" + file_root + "_stab.avi";
    std::string output_vis_file = output_folder + "/" + file_root + "_vis.avi";
    std::string output_log_file = output_folder + "/" + file_root + "_log";

    return run_stabilizer(input_file, config_file, output_file, output_vis_file, output_log_file, true);
}

int run_stabilizer(std::string input, std::string config_file, std::string output, std::string output_vis, std::string output_log, bool show)
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
   int fps = cap.get(CV_CAP_PROP_FPS);

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
   cv::VideoWriter voutr(output_vis,
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
    std::unique_ptr<Stabilizer> stabilizer = get_stabilizer_from_config(config_file, frame0);

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

        cv::Mat stabilized_vis, stabilized_mask;
        cv::cvtColor(frame0, stabilized_vis, CV_BGR2GRAY);
        cv::cvtColor(stabilized_vis, stabilized_vis, CV_GRAY2BGR);
        if (!stabilized.empty()) {
            cv::cvtColor(stabilized, stabilized_mask, CV_BGR2GRAY);
            stabilized.copyTo(stabilized_vis, stabilized_mask);
        }
        vout.write(stabilized_vis);
        voutr.write(out_frame);

        auto end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>
                            (end - start).count();

        ++i;
        std::cout << i << "/" << N << " done! ( " << time << "ms )" << std::endl;
        if (!stabilized.empty()){
            accuracy_unstabilized.evaluate_next(frame);
            accuracy_stabilized.evaluate_next(stabilized);
        }

        if (show) {


            cv::imshow("Video", out_frame);
            cv::imshow("Stabilized", stabilized_vis);
            //cv::imshow("diff", stabilized - frame0);

            if (cv::waitKey(1) >= 0) {
                break;
            }
        }

    }

    std::cout << "Accuracy (unstabilized): \n";
    std::cout << "  psnr = " << accuracy_unstabilized.psnr_stats().to_text() << "\n";
    std::cout << "  mssim = " << accuracy_unstabilized.mssim_stats().to_text() << "\n";

    std::cout << "Accuracy (stabilized): \n";
    std::cout << "  psnr = " << accuracy_stabilized.psnr_stats().to_text() << "\n";
    std::cout << "  mssim = " << accuracy_stabilized.mssim_stats().to_text() << "\n";




    /// Release resources
    cap.release();
    vout.release();
    voutr.release();
    if (show)
        cv::destroyAllWindows();
}


std::unique_ptr<Stabilizer> get_stabilizer_from_config(const std::string& config_file, const cv::Mat& frame0)
{
    ConfigFileReader reader(config_file);

    std::string parameter, value;

    if (!reader.get_next(parameter, value) || parameter!="TYPE") {
        throw std::runtime_error("Configuration file does not contain stabilizer type as the first parameter!");
    }

    if (value == "point")
        return configure_point_stabilizer(reader, frame0);
    else if (value == "pixel")
        return configure_pixel_stabilizer(reader, frame0);
    else if (value == "patch")
        return configure_patch_stabilizer(reader, frame0);
    else if (value == "mser")
        return configure_mser_stabilizer(reader, frame0);
    else
        throw std::runtime_error("Stabilizer type not supported!");

}

std::unique_ptr<PointStabilizer> configure_point_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0) {
    std::string parameter, value;
    Stabilizer::Warping warping;
    Stabilizer::Mode mode;
    PointStabilizer::FeatureExtractionParameters feature_params;
    PointStabilizer::OpticalFlowParameters flow_params;
    PointStabilizer::OpticalFlowParameters flow_params_retrieve;
    PointStabilizer::HomographyEstimationParameters homography_params;

    while (reader.get_next(parameter, value)) {
        if (parameter == "mode")
            if (stabilizer_mode_list.count(value) == 0)
                throw std::runtime_error("Invalid parameter value!");
            else
                mode = stabilizer_mode_list[value];
        else if (parameter == "warping")
            if (stabilizer_warping_list.count(value) == 0)
                throw std::runtime_error("Invalid parameter value!");
            else
                warping = stabilizer_warping_list[value];

        else if (parameter == "feature_params.maxN")
            feature_params.maxN = std::stoi(value);
        else if (parameter == "feature_params.quality")
            feature_params.quality = std::stod(value);
        else if (parameter == "feature_params.mindist")
            feature_params.mindist = std::stod(value);

        else if (parameter == "flow_params.lk_levels")
            flow_params.lk_levels = std::stoi(value);
        else if (parameter == "flow_params.max_err")
            flow_params.max_err = std::stod(value);
        else if (parameter == "flow_params.use_checked_optical_flow")
            flow_params.use_checked_optical_flow = std::stoi(value);

        else if (parameter == "flow_params_retrieve.lk_levels")
            flow_params_retrieve.lk_levels = std::stoi(value);
        else if (parameter == "flow_params_retrieve.max_err")
            flow_params_retrieve.max_err = std::stod(value);
        else if (parameter == "flow_params_retrieve.use_checked_optical_flow")
            flow_params_retrieve.use_checked_optical_flow = std::stoi(value);

        else if (parameter == "homography_params.min_points")
            homography_params.min_points = std::stoi(value);
        else if (parameter == "homography_params.use_ransac")
            homography_params.use_ransac = std::stoi(value);
        else
            throw std::runtime_error("Parameter value unknown!");
    }

    std::unique_ptr<PointStabilizer> stabilizer(new PointStabilizer(frame_0, warping, mode, feature_params, flow_params, flow_params_retrieve, homography_params));
    return stabilizer;
}

std::unique_ptr<PixelStabilizer> configure_pixel_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0) {
    // todo: put this to config
    Stabilizer::Warping warping = Stabilizer::Warping::HOMOGRAPHY;
    std::unique_ptr<PixelStabilizer> stabilizer(new PixelStabilizer(frame_0, warping));
    return stabilizer;
}

std::unique_ptr<PatchStabilizer> configure_patch_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0) {
    // todo: put this to config
    Stabilizer::Warping warping = Stabilizer::Warping::HOMOGRAPHY;
    std::unique_ptr<PatchStabilizer> stabilizer(new PatchStabilizer(frame_0, warping));
    return stabilizer;
}

std::unique_ptr<MserStabilizer> configure_mser_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0) {
    // todo: put this to config
    MatMser mser_detector(5, 50, 3000, 50.f, .1f, 25.f, 1.e-1);
    Stabilizer::Warping warping = Stabilizer::Warping::HOMOGRAPHY;
    Stabilizer::Mode mode = Stabilizer::Mode::TRACK_REF;
    int vis_flags = MserStabilizer::VIS_HULLS | MserStabilizer::VIS_MEANS;

    std::unique_ptr<MserStabilizer> stabilizer(new MserStabilizer(mser_detector, frame_0, warping, mode, vis_flags));
    return stabilizer;
}

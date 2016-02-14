//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

#include "opencv2/opencv.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <memory>
#include <boost/assign.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "libmsertools/MatMser.h"
#include "libstabilizer/Stabilizer.h"
#include "libstabilizer/MserStabilizer.h"
#include "libstabilizer/PointStabilizer.h"
#include "libstabilizer/PixelStabilizer.h"
#include "libstabilizer/PatchStabilizer.h"

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
    boost::filesystem::path input_file(argv[1]);
    boost::filesystem::path config_file(argv[2]);
    boost::filesystem::path output_folder(argv[3]);

    std::string file_root = input_file.stem().string();

    boost::filesystem::path output_file, output_vis_file, output_log_file;
    for (int i=0; i<100; ++i) {
        output_file = output_folder / (boost::format("%s_%02d_stab.avi") % file_root % i).str();
        output_vis_file = output_folder / (boost::format("%s_%02d_vis.avi") % file_root % i).str();
        output_log_file = output_folder / (boost::format("%s_%02d_log") % file_root % i).str();
        if (!boost::filesystem::exists(output_file) && !boost::filesystem::exists(output_vis_file) && !boost::filesystem::exists(output_log_file))
            break;
        else if (i == 99)
            throw std::runtime_error("Maximum number of output files exceeded!");
    }

    return run_stabilizer(input_file.string(), config_file.string(), output_file.string(), output_vis_file.string(), output_log_file.string(), true);
}

int run_stabilizer(std::string input, std::string config_file, std::string output, std::string output_vis, std::string output_log, bool show)
{
   cv::VideoCapture cap(input);

   if(!cap.isOpened()) {
      std::cerr << "Could not open input video!" << std::endl;
      return -1;
  }
   /// Get video parameters
   int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
   int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
   int N = cap.get(CV_CAP_PROP_FRAME_COUNT);
   double fps = cap.get(CV_CAP_PROP_FPS);

   // check fps
   if (!(fps >= 1))
       fps = 30;

   /*
   std::cout << "Frame width: " << frame_width << "\n";
   std::cout << "Frame height = " << frame_width << "\n";
   std::cout << "N = " << frame_width << "\n";
   std::cout << "-- Press Enter to continue -- " << std::endl;
   std::getchar();
   */

   /// Create output video writers
   cv::VideoWriter vout(output,
                           CV_FOURCC('D', 'I', 'V', 'X'),
                           fps,
                           cv::Size(frame_width,frame_height),
                           true);
   cv::VideoWriter voutr(output_vis,
                           CV_FOURCC('D', 'I', 'V', 'X'),
                           fps,
                           cv::Size(frame_width,frame_height),
                           true);

    /// If required, create windows for visualization
    if (show) {
        cv::namedWindow( "Video", CV_WINDOW_AUTOSIZE );
        cv::namedWindow( "Stabilized", CV_WINDOW_AUTOSIZE );
    }

    /// Read first frame
    cv::Mat frame, frame0;
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
        if (!cap.read(frame))
            break;

        auto start = std::chrono::high_resolution_clock::now();
        cv::Mat stabilized = stabilizer->stabilize_next(frame);
        auto end = std::chrono::high_resolution_clock::now();

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

            // break when ESC is pressed
            if (cv::waitKey(1) % 256 == 27) {
                break;
            }
        }

    }

    // Write stats
    std::string output_str = (boost::format("Stabilized %d/%d frames\n\n") % (i+1) % N).str()
                          +  "Accuracy (unstabilized): \n"
                          +  "  psnr = " + accuracy_unstabilized.psnr_stats().to_text() + "\n"
                          +  "  mssim = " + accuracy_unstabilized.mssim_stats().to_text() + "\n"
                          +  "Accuracy (stabilized): \n"
                          +  "  psnr = " + accuracy_stabilized.psnr_stats().to_text() + "\n"
                          +  "  mssim = " + accuracy_stabilized.mssim_stats().to_text() + "\n"
                          +  "\n";

    std::cout << output_str;

    std::ofstream logfile_stream;
    logfile_stream.open(output_log);
    if (!logfile_stream.is_open())
        std::cout << "Could not write log file!" << std::endl;

    else {
        logfile_stream << output_str;
        logfile_stream << "Configuration:\n";
        std::ifstream configfile_stream;
        configfile_stream.open(config_file);
        if (configfile_stream.is_open())
            logfile_stream << configfile_stream.rdbuf();
        else
            logfile_stream << "-- not available --\n";
        configfile_stream.close();
    }
    logfile_stream.close();

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
        throw std::runtime_error((boost::format("Stabilizer of type %s unknown!") % value).str());

}

std::unique_ptr<PointStabilizer> configure_point_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0) {
    Stabilizer::Warping warping = Stabilizer::Warping::HOMOGRAPHY;
    Stabilizer::Mode mode;
    PointStabilizer::FeatureExtractionParameters feature_params;
    PointStabilizer::OpticalFlowParameters flow_params;
    PointStabilizer::OpticalFlowParameters flow_params_retrieve;
    PointStabilizer::HomographyEstimationParameters homography_params;

    std::string parameter, value;
    while (reader.get_next(parameter, value)) {
        if (parameter == "mode")
            if (stabilizer_mode_list.count(value) == 0)
                throw std::runtime_error((boost::format("Invalid value %s for parameter %s!") % value % parameter).str());
            else
                mode = stabilizer_mode_list[value];
        else if (parameter == "warping")
            if (stabilizer_warping_list.count(value) == 0)
                throw std::runtime_error((boost::format("Invalid value %s for parameter %s!") % value % parameter).str());
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
            throw std::runtime_error((boost::format("Parameter %s unknown!") % parameter).str());
    }

    std::unique_ptr<PointStabilizer> stabilizer(new PointStabilizer(frame_0, warping, mode, feature_params, flow_params, flow_params_retrieve, homography_params));
    return stabilizer;
}

std::unique_ptr<PixelStabilizer> configure_pixel_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0) {
    Stabilizer::Warping warping = Stabilizer::Warping::HOMOGRAPHY;

    std::string parameter, value;
    while (reader.get_next(parameter, value)) {
        if (parameter == "warping")
            if (stabilizer_warping_list.count(value) == 0)
                throw std::runtime_error((boost::format("Invalid value %s for parameter %s!") % value % parameter).str());
            else
                warping = stabilizer_warping_list[value];
        else
            throw std::runtime_error((boost::format("Parameter %s unknown!") % parameter).str());
    }
    std::unique_ptr<PixelStabilizer> stabilizer(new PixelStabilizer(frame_0, warping));
    return stabilizer;
}

std::unique_ptr<PatchStabilizer> configure_patch_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0) {
    Stabilizer::Warping warping = Stabilizer::Warping::HOMOGRAPHY;
    PatchStabilizer::OpticalFlowParameters flow_params;
    PatchStabilizer::OpticalFlowParameters flow_params_retrieve;
    PatchStabilizer::HomographyEstimationParameters homography_params;
    PatchStabilizer::PatchParameters patch_params;

    std::string parameter, value;
    while (reader.get_next(parameter, value)) {
        if (parameter == "warping")
            if (stabilizer_warping_list.count(value) == 0)
                throw std::runtime_error((boost::format("Invalid value %s for parameter %s!") % value % parameter).str());
            else
                warping = stabilizer_warping_list[value];

        else if (parameter == "flow_params.lk_levels")
            flow_params.lk_levels = std::stoi(value);
        else if (parameter == "flow_params.max_err")
            flow_params.max_err = std::stod(value);
        else if (parameter == "flow_params.max_err_weighted")
            flow_params.max_err_weighted = std::stod(value);
        else if (parameter == "flow_params.use_checked_optical_flow")
            flow_params.use_checked_optical_flow = std::stoi(value);
        else if (parameter == "flow_params.minEigThreshold")
            flow_params.minEigThreshold = std::stod(value);
        else if (parameter == "flow_params.lk_window")
            flow_params.lk_window = std::stoi(value);

        else if (parameter == "flow_params_retrieve.lk_levels")
            flow_params_retrieve.lk_levels = std::stoi(value);
        else if (parameter == "flow_params_retrieve.max_err")
            flow_params_retrieve.max_err = std::stod(value);
        else if (parameter == "flow_params_retrieve.max_err_weighted")
            flow_params_retrieve.max_err_weighted = std::stod(value);
        else if (parameter == "flow_params_retrieve.use_checked_optical_flow")
            flow_params_retrieve.use_checked_optical_flow = std::stoi(value);

        else if (parameter == "homography_params.maxiter")
            homography_params.maxiter = std::stoi(value);
        else if (parameter == "homography_params.regularize")
            homography_params.regularize = std::stod(value);
        else if (parameter == "homography_params.reweight_lambda")
            homography_params.reweight_lambda = std::stod(value);

        else if (parameter == "patch_params.Nx")
            patch_params.Nx  = std::stoi(value);
        else if (parameter == "patch_params.Ny")
            patch_params.Ny  = std::stoi(value);
        else if (parameter == "patch_params.sigma_grad")
            patch_params.sigma_grad  = std::stod(value);
        else if (parameter == "patch_params.sigma_integrate")
            patch_params.sigma_integrate  = std::stod(value);

        else
            throw std::runtime_error((boost::format("Parameter %s unknown!") % parameter).str());

    }

    std::unique_ptr<PatchStabilizer> stabilizer(new PatchStabilizer(frame_0, patch_params, homography_params,
                                                                    flow_params, flow_params_retrieve));
    return stabilizer;
}

std::unique_ptr<MserStabilizer> configure_mser_stabilizer(ConfigFileReader& reader, const cv::Mat& frame_0) {
    // todo: put this to config
    MatMser mser_detector(5, 50, 3000, 50.f, .1f, 25.f, 1.e-1);

    Stabilizer::Warping warping = Stabilizer::Warping::HOMOGRAPHY;
    Stabilizer::Mode mode = Stabilizer::Mode::TRACK_REF;
    int vis_flags = MserStabilizer::VIS_HULLS | MserStabilizer::VIS_MEANS;

    std::string parameter, value;
    while (reader.get_next(parameter, value)) {
        if (parameter == "mode")
            if (stabilizer_mode_list.count(value) == 0)
                throw std::runtime_error((boost::format("Invalid value %s for parameter %s!") % value % parameter).str());
            else
                mode = stabilizer_mode_list[value];
        else if (parameter == "warping")
            if (stabilizer_warping_list.count(value) == 0)
                throw std::runtime_error((boost::format("Invalid value %s for parameter %s!") % value % parameter).str());
            else
                warping = stabilizer_warping_list[value];
        else
            throw std::runtime_error((boost::format("Parameter %s unknown!") % parameter).str());
    }

    std::unique_ptr<MserStabilizer> stabilizer(new MserStabilizer(mser_detector, frame_0, warping, mode, vis_flags));
    return stabilizer;
}

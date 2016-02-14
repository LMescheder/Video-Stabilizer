//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

#ifndef STABILZATIONACCURACYEVALUATOR_H
#define STABILZATIONACCURACYEVALUATOR_H

#include "opencv2/opencv.hpp"
#include <limits>
#include "boost/format.hpp"

class AccuracyEvaluator
{
public:
    struct Stats {
        double val = 0.;
        double min = std::numeric_limits<double>::infinity();
        double max = 0.;
        double average = 0.;
        int N = 0;

        void update (double new_val);

        std::string to_text() const {
            return (boost::format( "%.3f -- %.3f (%.3f)") % min % max % average).str();
        }
    };

    AccuracyEvaluator(const cv::Mat& reference_frame);

    void evaluate_next(const cv::Mat& frame);

    const Stats& psnr_stats () {
        return psnr_stats_;
    }

    const Stats& mssim_stats () {
        return mssim_stats_;
    }

    static double compute_PSNR(const cv::Mat&mat1, const cv::Mat&mat2);
    static double compute_MSSIM(const cv::Mat&mat1, const cv::Mat&mat2);

private:
    cv::Mat crop(const cv::Mat& mat);

    cv::Mat ref_frame_;
    bool first_crop_ = true;

    Stats psnr_stats_;
    Stats mssim_stats_;
};

#endif // STABILZATIONACCURACYEVALUATOR_H

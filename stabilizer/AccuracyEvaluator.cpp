//=======================================================================
// Copyright Lars Mescheder 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at
//  http://opensource.org/licenses/MIT)
//=======================================================================

#include "AccuracyEvaluator.h"
#include <exception>

AccuracyEvaluator::AccuracyEvaluator(const cv::Mat &reference_frame)
{
    ref_frame_ = reference_frame.clone();
}

void AccuracyEvaluator::evaluate_next(const cv::Mat& frame)
{
    double psnr, mssim;
    if (first_crop_) {
        cv::Mat frame_crop = crop(frame);
        cv::Mat ref_frame_crop = crop(ref_frame_);
        psnr = compute_PSNR(frame_crop, ref_frame_crop);
        mssim = compute_MSSIM(frame_crop, ref_frame_crop);
    } else {
        psnr = compute_PSNR(frame, ref_frame_);
        mssim = compute_MSSIM(frame, ref_frame_);
    }

    psnr_stats_.update(psnr);
    mssim_stats_.update(mssim);
}

double AccuracyEvaluator::compute_PSNR(const cv::Mat& mat1, const cv::Mat& mat2)
{
    if (mat1.rows != mat2.rows)
        throw std::runtime_error("Matrices must have same number of rows!");
    if (mat1.cols != mat2.cols)
        throw std::runtime_error("Matrices must have same number of columns!");
    if (mat1.channels() != mat2.channels())
        throw std::runtime_error("Matrices must have same number of channels!");

    // Some abbreviations
    int N_rows = mat1.rows;
    int N_cols = mat1.cols;
    int N_channels = mat1.channels();

    // Compute mean squared error.
    double mser = 0;
    int N_values = 0;

    // Iterate over all pixels.
    for (int i = 0; i < N_rows; ++i) {
        const uchar* p1 = mat1.ptr<uchar>(i);
        const uchar* p2 = mat2.ptr<uchar>(i);
        for (int j = 0; j < N_cols; ++j) {
            bool black1 = true;
            bool black2 = true;
            double dmser = 0;
            for (int k =0; k < N_channels; ++k) {
                int idx = N_channels*j + k;
                black1 &= (p1[idx] == 0);
                black2 &= (p2[idx] == 0);
                dmser += (p2[idx] - p1[idx])*(p2[idx] - p1[idx]);
            }
            if (!(black1 || black2)) {
                mser += dmser/N_channels;
                ++N_values;
            }
        }
    }
    mser /= N_values;


    if( mser <= 1e-10)
        return 0;
    else
        return 10.0*log10((255*255)/mser);
}


// taken from http://docs.opencv.org/doc/tutorials/highgui/video-input-psnr-ssim/video-input-psnr-ssim.html
double AccuracyEvaluator::compute_MSSIM(const cv::Mat& mat1, const cv::Mat & mat2)
{
    using namespace cv;

    const double C1 = 6.5025, C2 = 58.5225;

    // intitializations
    int d     = CV_32F;

    Mat I1, I2;
    mat1.convertTo(I1, d);           // cannot calculate on one byte large values
    mat2.convertTo(I2, d);

    Mat I2_2   = I2.mul(I2);        // I2^2
    Mat I1_2   = I1.mul(I1);        // I1^2
    Mat I1_I2  = I1.mul(I2);        // I1 * I2

    // preliminary computations

    Mat mu1, mu2;
    GaussianBlur(I1, mu1, Size(11, 11), 1.5);
    GaussianBlur(I2, mu2, Size(11, 11), 1.5);

    Mat mu1_2   =   mu1.mul(mu1);
    Mat mu2_2   =   mu2.mul(mu2);
    Mat mu1_mu2 =   mu1.mul(mu2);

    Mat sigma1_2, sigma2_2, sigma12;

    GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;

    GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;

    GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;

    // the actual formula
    Mat t1, t2, t3;

    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

    Mat ssim_map;
    divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;

    Scalar mssim = mean(ssim_map); // mssim = average of ssim map

    return (mssim[0] + mssim[1] + mssim[2])/3;
}

cv::Mat AccuracyEvaluator::crop(const cv::Mat& frame)
{
    int xMarg = frame.rows / 6;
    int yMarg = frame.cols / 6;

    cv::Rect roi(xMarg, yMarg, frame.size().width - 2*xMarg, frame.size().height - 2*yMarg);
    cv::Mat image_roi = frame(roi);

    return image_roi;
}



void AccuracyEvaluator::Stats::update(double new_val) {
    val = new_val;
    min = std::min(min, val);
    max = std::max(max, val);
    double alpha = 1. / (N+1);
    average = (1 - alpha) * average + alpha * val;
    ++N;
}

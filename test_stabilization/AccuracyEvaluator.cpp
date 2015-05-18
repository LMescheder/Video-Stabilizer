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

double AccuracyEvaluator::compute_MSSIM(const cv::Mat& mat1, const cv::Mat & mat2)
{


}

cv::Mat AccuracyEvaluator::crop(const cv::Mat& frame)
{
    int xMarg = frame.rows / 6;
    int yMarg = frame.cols / 6;

    cv::Rect roi(xMarg, yMarg, frame.size().width - 2*xMarg, frame.size().height - 2*yMarg);
    cv::Mat image_roi = frame(roi);
    resize(image_roi, image_roi, cv::Size(640, 480));

    return image_roi;
}


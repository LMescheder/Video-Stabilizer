#include "Stabilizer.h"

cv::Mat Stabilizer::stabilize_next(const cv::Mat& next_frame)
{
    cv::cvtColor(next_frame, frame_gray_, CV_BGR2GRAY);

    // first warp with previous homography to make direct tracking from template possible
    if (warping_back_)
        cv::warpPerspective(frame_gray_, H_frame_gray_, H_, cv::Size(frame_gray_.cols, frame_gray_.rows));
    else
        H_frame_gray_ = frame_gray_;

    // compute the new homography
    cv::Mat new_H = get_next_homography(H_frame_gray_);

    // visualize if required
    if (visualize_)
        create_visualization(next_frame);
    else
        visualization_.release();


    // compose new homography with previous one (undoing the initial back warping)
    H_ = new_H * H_;

    // compute and return stabilized frame
    cv::Mat stabilized_frame;
    cv::warpPerspective(next_frame, stabilized_frame, H_, cv::Size(next_frame.cols, next_frame.rows));

    // reset reference frame
    if (!warping_back_) {
        frame_gray_0_ = frame_gray_.clone();
    }
    return stabilized_frame;
}

#include "Stabilizer.h"

cv::Mat Stabilizer::stabilize_next(const cv::Mat& next_frame)
{
    cv::cvtColor(next_frame, frame_gray_, CV_BGR2GRAY);

    // first warp with previous homography to make direct tracking from template possible
    if (mode_ == Mode::WARP_BACK)
        cv::warpPerspective(frame_gray_, frame_gray_, H_, cv::Size(frame_gray_.cols, frame_gray_.rows));

    // compute the new homography
    cv::Mat new_H = get_next_homography(frame_gray_);

    // visualize if required
    if (visualize_) {
        visualization_ = next_frame.clone();
        create_visualization();
    } else {
        visualization_.release();
    }

    // compute new homography
    if (new_H.empty()) {
        H_ = cv::Mat::eye(3, 3, CV_64F);
        std::cout << "-- stabilization not possible --!." << std::endl;
        return cv::Mat();
    }

    if (mode_ == Mode::WARP_BACK)
        H_ = new_H * H_;            // undo warping back
    else
        H_ = new_H;

    // compute and return stabilized frame
    cv::Mat stabilized_frame;
    cv::warpPerspective(next_frame, stabilized_frame, H_, cv::Size(next_frame.cols, next_frame.rows));

    // reset reference frame
    if (mode_ == Mode::TRACK_REF)
        track_ref();

    return stabilized_frame;
}

void Stabilizer::track_ref()
{
    ref_frame_gray_ = frame_gray_.clone();
}

cv::Mat Stabilizer::find_homography(const cv::vector<cv::Point2f> &points0, const cv::vector<cv::Point2f> &points1, Warping mode, bool use_ransac)
{
    cv::Mat H, A;

    switch (mode) {
    case Warping::HOMOGRAPHY :
        if (use_ransac)
            H = cv::findHomography(points0, points1, CV_RANSAC);
        else
            H = cv::findHomography(points0, points1);
        break;
    case Warping::AFFINE :
        A = cv::estimateRigidTransform(points0, points1, true);
        H = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H(cv::Range(0, 2), cv::Range(0, 3)));
        break;
    case Warping::RIGID :
        A = cv::estimateRigidTransform(points0, points1, false);
        H = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H(cv::Range(0, 2), cv::Range(0, 3)));
        break;
    default:
        throw std::logic_error("Requested warping not implemented!");
    }
    return H;
}



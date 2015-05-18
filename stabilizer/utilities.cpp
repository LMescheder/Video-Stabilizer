#include <vector>
#include "opencv2/opencv.hpp"
#include "utilities.h"

cv::Mat find_homography(const cv::vector<cv::Point2f> &points0, const cv::vector<cv::Point2f> &points1, WarpingGroup mode)
{
    cv::Mat H, A;

    switch (mode) {
    case WarpingGroup::homography :
        H = cv::findHomography(points0, points1, CV_RANSAC);
        break;
    case WarpingGroup::affine :
        A = cv::estimateRigidTransform(points0, points1, true);
        H = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H(cv::Range(0, 2), cv::Range(0, 3)));
        break;
    case WarpingGroup::rigid :
        A = cv::estimateRigidTransform(points0, points1, false);
        H = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H(cv::Range(0, 2), cv::Range(0, 3)));
        break;
    }
    return H;
}

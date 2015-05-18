#ifndef UTILITIES_HPP
#define UTILITIES_HPP


enum class WarpingGroup {
    homography,
    affine,
    rigid
};

cv::Mat find_homography(const cv::vector<cv::Point2f>& points0, const cv::vector<cv::Point2f>& points1, WarpingGroup mode=WarpingGroup::homography);

#endif // UTILITIES_HPP

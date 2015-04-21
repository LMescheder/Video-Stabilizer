#include "VideoStabilizer.hpp"

cv::Mat VideoStabilizer::stabilze_next(cv::Mat next_image) {
    cv::Mat gray;
    cv::cvtColor(next_image, gray, CV_BGR2GRAY);

    tracker_.update(gray);

    std::vector<cv::Point2f> means, means0;\
    auto msers = tracker_.msers();
    assert(msers.size() == msers_0_.size());
    means.reserve(msers.size());
    means0.reserve(msers.size());
    for (std::size_t i=0; i<msers.size(); ++i)
        if (msers[i].N > 0) {
            means.push_back(msers[i].mean);
            means0.push_back(msers_0_[i].mean);
        }

    cv::Mat H = cv::findHomography(means, means0);

    H = H0_ * H;
    cv::Mat stabilized;
    cv::warpPerspective(next_image, stabilized, H, cv::Size(next_image.cols, next_image.rows));

    ++count_;
    if (count_ % recompute_T_ == 0) {
        recompute_msers_(gray);
        H0_ = H.clone();
    }

    return stabilized;
}

void VideoStabilizer::recompute_msers_(cv::Mat image) {
    tracker_.reset();
    // TODO: merge this into reset
    tracker_.update(image);
    msers_0_ = tracker_.msers();
}

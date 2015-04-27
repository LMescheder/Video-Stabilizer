#include <iostream>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

#include "stabilizer.hpp"

void run_stabilizer (const std::string& filename);
void visualize (cv::Mat image, const std::vector<cv::Point2f>& points, const std::vector<cv::Point2f>& points0);

int main (int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: stabilizer [filename]" << std::endl;
        return 0;
    }

    std::string filename = argv[1];

    run_stabilizer(filename);
}

void run_stabilizer(const std::string &filename) {
    cv::VideoCapture video_in (filename);

    if (!video_in.isOpened()) {
        std::cout << "Could not open video!" << std::endl;
        return;
    }

    cv::Mat frame;
    video_in.read(frame);

    Stabilizer stabilizer(frame);

    while (video_in.isOpened() && video_in.read(frame)) {
        cv::Mat stabilized_frame = stabilizer.stabilize_next(frame);


        visualize(frame, stabilizer.points(), stabilizer.points0());

        cv::imshow("Normal", frame);
        cv::imshow("Stabilized", stabilized_frame);

        if (cv::waitKey(1) >= 0)
            break;
    }
}

void visualize (cv::Mat image, const std::vector<cv::Point2f>& points, const std::vector<cv::Point2f>& points0) {
    assert(points.size() == points0.size());

    for (std::size_t i=0; i<points.size(); ++i) {
            cv::circle(image, points[i], 3, cv::Scalar(0, 255, 0));
            cv::line(image, points[i], points0[i], cv::Scalar(0, 255, 0));
    }
}

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Tree.hpp"

using namespace cv;


int main(int argc, char *argv[])
{
    VideoCapture cap(0);

    if(!cap.isOpened())
       return -1;

    namedWindow( "Video", CV_WINDOW_AUTOSIZE );


    while (true) {
        Mat frame;
        cap >> frame;
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        imshow("Video", frame);

        if (waitKey(1) >= 0) {
            break;
        }
    }

    return 0;
}




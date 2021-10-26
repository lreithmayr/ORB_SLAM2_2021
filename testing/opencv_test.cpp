#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;

int main(int argc, char **argv) {
    cv::Mat img;
    string vid = "http://192.168.0.94:8080/video";
    cv::VideoCapture cap(vid);

    string window_name = "Window";
    namedWindow(window_name, cv::WINDOW_NORMAL);

    while (true) {
        // Read frame
        bool bSuccess = cap.read(img);

        if (!bSuccess) {
            cerr << endl << "Webcam input failed " << endl;
            break;
        }

        imshow(window_name, img);

        if (cv::waitKey(1) == 27) {
            cout << "ESC pressed." << endl;
            break;
        }
    }
}
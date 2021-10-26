#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;

int main(int argc, char **argv) {
    cv::Mat img;
    string cam_url = "http://192.168.0.94:8080/";
    cv::VideoCapture cap(cam_url);

    string window_name = "My First Video";
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
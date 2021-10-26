/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

// void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
//                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_ipWebcam path_to_vocabulary path_to_settings path_to_video" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Main loop
    cv::Mat img;
    string cam_url = argv[3];
    cv::VideoCapture cap(cam_url);

    while(true)
    {
        // Read frame
        bool bSuccess = cap.read(img);

        if(!bSuccess)
        {
            cerr << endl << "Webcam input failed " << endl;
            break;
        }

        double tframe = cap.get(cv::CAP_PROP_POS_MSEC);

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(img,tframe);

        if (cv::waitKey(1) == 27)
        {
            cout << "ESC pressed." << endl;
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


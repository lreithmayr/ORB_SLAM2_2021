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
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<pangolin/pangolin.h>

#include<System.h>

using namespace std;


void LoadImages(const string &strPathToSequence, vector<string> &vstrImages, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./arducam_images path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImages;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImages, vTimestamps);

    const int nImages = vstrImages.size();

    int nImages_var;
    if (string(argv[4]) == "reduced") {
        nImages_var = 500;
    } else {
        nImages_var = nImages;
    }

    string strSettingsFile = string(argv[2]);
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat img, imLeft, imRight;
    for(int i=0; i<nImages_var; i++)
    {
        // Read left and right images from file
        img = cv::imread(vstrImages[i], CV_LOAD_IMAGE_UNCHANGED);
	    imLeft = img(cv::Rect(0, 0, (width), height));
	    imRight = img(cv::Rect((width), 0, (width), height)); 
        double tframe = vTimestamps[i];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImages[i]) << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[i]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(i<nImages-1)
            T = vTimestamps[i+1]-tframe;
        else if(i>0)
            T = tframe-vTimestamps[i-1];

        if(ttrack<T)
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T-ttrack)*1e6)));

        cv::imshow("Window", img);
        if (cv::waitKey(1) == 27)
            break;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int i=0; i<nImages; i++)
    {
        totaltime+=vTimesTrack[i];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImages, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    const int nTimes = vTimestamps.size();
    vstrImages.resize(nTimes);

    int ctr = 10000;
    for(int i=0; i<nTimes; i++)
    {
        stringstream str_stream;
        str_stream << ctr;
        vstrImages[i] = strPathToSequence + str_stream.str() + ".jpg";
        ctr++;
    }
}

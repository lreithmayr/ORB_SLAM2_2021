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
**/

#include<iostream>
#include<algorithm>
#include<chrono>
#include<tuple>

#include<opencv2/core/core.hpp>
#include<pangolin/pangolin.h>
#include<tf/tf.h>

#include<System.h>

using namespace std;

struct RectMats
{
	cv::Mat M1l;
	cv::Mat M1r;
	cv::Mat M2l;
	cv::Mat M2r;
};

void LoadImages(const string& strPathToSequence, vector<string>& vstrImages, vector<double>& vTimestamps);

RectMats rectification(cv::FileStorage& fsSettings);

void PublishCameraPose(cv::Mat& Tcw, ros::Publisher& pose_pub);

int main(int argc, char** argv)
{
	if (argc != 5)
	{
		cerr << endl << "Usage: ./arducam_images path_to_vocabulary path_to_settings path_to_sequence" << endl;
		return 1;
	}

	// Retrieve paths to images
	vector<string> vstrImages;
	vector<double> vTimestamps;
	LoadImages(string(argv[3]), vstrImages, vTimestamps);

	const uint32_t nImages = vstrImages.size();

	uint32_t nImages_var;
	if (string(argv[4]) == "reduced")
	{
		nImages_var = 500;
	}
	else
	{
		nImages_var = nImages;
	}

	string strSettingsFile = string(argv[2]);
	cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		cerr << "ERROR: Wrong path to settings" << endl;
		return -1;
	}

	int width = fsSettings["Camera.width"];
	int height = fsSettings["Camera.height"];

	auto rect_mats = rectification(fsSettings);

	// Initialize ROS node
	ros::init(argc, argv, "os2");
	ros::NodeHandle nh;
	ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("os2_pose_fromMain", 10);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	bool mapping = false;
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, false, mapping, nh);

	// Vector for tracking time statistics
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);

	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;

	// Main loop
	uint32_t height_top_cropped;
	uint32_t height_bottom_cropped;
	bool crop = true;
	if (crop)
	{
		height_top_cropped = 0;
		height_bottom_cropped = 400;
	}
	else
	{
		height_top_cropped = 0;
		height_bottom_cropped = height;
	}

	cv::Mat img, imLeft, imRight, imLeftRect, imRightRect, imLeftRectCropped, imRightRectCropped;
	for (uint32_t i = 0; i < nImages_var; i++)
	{
		// Read left and right images from file
		img = cv::imread(vstrImages[i], CV_LOAD_IMAGE_UNCHANGED);
		imLeft = img(cv::Rect(0, 0, width, height));
		imRight = img(cv::Rect(width, 0, width, height));

		cv::remap(imLeft, imLeftRect, rect_mats.M1l, rect_mats.M2l, cv::INTER_LINEAR);
		cv::remap(imRight, imRightRect, rect_mats.M1r, rect_mats.M2r, cv::INTER_LINEAR);

		imLeftRectCropped =
			imLeftRect(cv::Rect(0, height_top_cropped, width, height_bottom_cropped - height_top_cropped));
		imRightRectCropped =
			imRightRect(cv::Rect(0, height_top_cropped, width, height_bottom_cropped - height_top_cropped));

		double tframe = vTimestamps[i];

		if (imLeft.empty())
		{
			cerr << endl << "Failed to load image at: "
				 << string(vstrImages[i]) << endl;
			return 1;
		}

		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		// Pass the images to the SLAM system
		// TrackStereo returns the determined camera pose in a 4x4 matrix in the form (Rcw | tcw):
		/*
			 Rcw11 Rcw12 Rcw13 | tcw1
			 Rcw21 Rcw22 Rcw23 | tcw2
			 Rcw31 Rcw32 Rcw33 | tcw3
		*/
		cv::Mat Tcw = SLAM.TrackStereo(imLeftRectCropped, imRightRectCropped, tframe);

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		vTimesTrack[i] = ttrack;

		// Publish the camera pose via ROS on the topic os2_pose
		PublishCameraPose(Tcw, pub_pose);

		// Wait to load the next frame
		double T = 0;
		if (i < nImages - 1)
			T = vTimestamps[i + 1] - tframe;
		else if (i > 0)
			T = tframe - vTimestamps[i - 1];

		if (ttrack < T)
			std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T - ttrack) * 1e6)));
	}

	// Stop all threads
	SLAM.Shutdown();

	// Tracking time statistics
	sort(vTimesTrack.begin(), vTimesTrack.end());
	float totaltime = 0;
	for (uint32_t i = 0; i < nImages; i++)
	{
		totaltime += vTimesTrack[i];
	}
	cout << "-------" << endl << endl;
	cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
	cout << "mean tracking time: " << totaltime / nImages << endl;

	// Save camera trajectory
	// SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

	ros::shutdown();

	return 0;
}

void LoadImages(const string& strPathToSequence, vector<string>& vstrImages, vector<double>& vTimestamps)
{
	ifstream fTimes;
	string strPathTimeFile = strPathToSequence + "times.txt";
	fTimes.open(strPathTimeFile.c_str());
	while (!fTimes.eof())
	{
		string s;
		getline(fTimes, s);
		if (!s.empty())
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
	for (int i = 0; i < nTimes; i++)
	{
		stringstream str_stream;
		str_stream << ctr;
		vstrImages[i] = strPathToSequence + str_stream.str() + ".jpg";
		ctr++;
	}
}

RectMats rectification(cv::FileStorage& fsSettings)
{
	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
	fsSettings["LEFT.K"] >> K_l;
	fsSettings["RIGHT.K"] >> K_r;

	fsSettings["LEFT.P"] >> P_l;
	fsSettings["RIGHT.P"] >> P_r;

	fsSettings["LEFT.R"] >> R_l;
	fsSettings["RIGHT.R"] >> R_r;

	fsSettings["LEFT.D"] >> D_l;
	fsSettings["RIGHT.D"] >> D_r;

	int rows_l = fsSettings["LEFT.height"];
	int cols_l = fsSettings["LEFT.width"];
	int rows_r = fsSettings["RIGHT.height"];
	int cols_r = fsSettings["RIGHT.width"];

	if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty()
		|| D_r.empty() ||
		rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
	{
		cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
	}

	RectMats rectification_matrices;
	cv::initUndistortRectifyMap(K_l,
		D_l,
		R_l,
		P_l.rowRange(0, 3).colRange(0, 3),
		cv::Size(cols_l, rows_l),
		CV_32F,
		rectification_matrices.M1l,
		rectification_matrices.M2l);
	cv::initUndistortRectifyMap(K_r,
		D_r,
		R_r,
		P_r.rowRange(0, 3).colRange(0, 3),
		cv::Size(cols_r, rows_r),
		CV_32F,
		rectification_matrices.M1r,
		rectification_matrices.M2r);

	return rectification_matrices;
}

void PublishCameraPose(cv::Mat& Tcw, ros::Publisher& pose_pub)
{
	//Rotation Matrix
	cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
	// Convert to camera coordinates
	cv::Mat Rwc = Rcw.t();


	// Conversion to Quaternion: q[0] = q.x(), q[1] = q.y(), q[2] = q.z(), q[3] = q.w()
	// geometry_msgs::Pose::Orientation is a quaternion representation
	vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

	// geometry_msgs::Pose::position in (x, y, z)
	cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
	// Convert to camera coordinates
	cv::Mat twc = -Rwc*tcw;

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "map";

	// Define R and t via tf, convert to Pose message and publish
	tf::Transform new_transform;
	tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);

	new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));
	new_transform.setRotation(quaternion);

	tf::poseTFToMsg(new_transform, pose.pose);
	pose_pub.publish(pose);
}
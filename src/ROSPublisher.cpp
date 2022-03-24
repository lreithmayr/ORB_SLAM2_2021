//
// Created by lorenz on 14.03.22.
//

#include "ROSPublisher.h"

namespace ORB_SLAM2
{
	ROSPublisher::ROSPublisher(Map* map, ros::NodeHandle& nh) :
		Map_(map),
		nh_(nh),
		queue_size_(1),
		finished_(false),
		stopped_(false),
		finish_requested_(false)
	{
	}

	void ROSPublisher::SetTracker(Tracking* Tracker)
	{
		Tracker_ = Tracker;
	}

	void ROSPublisher::SetLoopCloser(LoopClosing* LoopCloser)
	{
		LoopCloser_ = LoopCloser;
	}

	void ROSPublisher::SetLocalMapper(LocalMapping* LocalMapper)
	{
		LocalMapper_ = LocalMapper;
	}

	void ROSPublisher::Run()
	{
		finished_ = false;
		ros::Rate rate(10);

		while (true)
		{
			if (Tracker_->mCurrentFrame.is_keyframe_ && !LoopCloser_->loop_closed_)
			{
				// TODO: Get KF pose and map points in KF and run grid map algorithm

				CameraPose pose = GetKFPose();
				set<MapPoint*> map_points = GetKFMapPoints();
			}
			else if (LoopCloser_->loop_closed_)
			{
				// TODO: Republish all updated KF poses and MP locations and update grid map

				std::cout << "Loop detected!" << endl;
			}

			rate.sleep();

			if (CheckFinish())
				break;
		}

		SetFinish();
	}

	std::vector<MapPoint*> ROSPublisher::GetAllMPs()
	{
		return Map_->GetAllMapPoints();
	}

	set<MapPoint*> ROSPublisher::GetKFMapPoints()
	{
		KeyFrame* current_KF = Tracker_->mCurrentFrame.mpReferenceKF;
		set<MapPoint*> mps_in_KF = current_KF->GetMapPoints();
		return mps_in_KF;
	}

	ROSPublisher::CameraPose ROSPublisher::GetKFPose()
	{
		KeyFrame* current_KF = Tracker_->mCurrentFrame.mpReferenceKF;
		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

		vector<ORB_SLAM2::KeyFrame*> all_KFs = Map_->GetAllKeyFrames();
		sort(all_KFs.begin(), all_KFs.end(), ORB_SLAM2::KeyFrame::lId);

		cv::Mat Two = all_KFs[0]->GetPoseInverse();

		Trw = Trw * current_KF->GetPose()*Two;
		cv::Mat curr_rel_framepose = Tracker_->mlRelativeFramePoses.back();
		cv::Mat Tcw = curr_rel_framepose * Trw;
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

		ROSPublisher::CameraPose camera_pose{};
		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		return camera_pose;
	}

	template<typename T>
	pcl::PointCloud<pcl::PointXYZ> ROSPublisher::ConvertToPCL(T mps)
	{
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl_cloud.width = mps.size();
		pcl_cloud.height = 1;
		pcl_cloud.is_dense = true;
		pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

		for (auto mp : mps)
		{
			float x = mp->GetWorldPos().template at<float>(0);
			float y = mp->GetWorldPos().template at<float>(1);
			float z = mp->GetWorldPos().template at<float>(2);

			pcl_cloud.emplace_back(pcl::PointXYZ(x, y, z));
		}

		return pcl_cloud;
	}

	void ROSPublisher::PublishPC(pcl::PointCloud<pcl::PointXYZ>& pub_cld, ros::Publisher& pub)
	{
		sensor_msgs::PointCloud2 pc2_cld;
		pcl::toROSMsg(pub_cld, pc2_cld);

		pc2_cld.header.frame_id = "map";
		pc2_cld.header.stamp = ros::Time::now();
		pc2_cld.row_step = (pc2_cld.point_step * pc2_cld.width);
		pc2_cld.data.resize(pc2_cld.height * pc2_cld.row_step);

		pub.publish(pc2_cld);
	}

	void ROSPublisher::PublishKFPose(cv::Mat& Tcw, ros::Publisher& pub)
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
		cv::Mat twc = -Rwc * tcw;

		// TODO: Synchronize pose time stamp and pc time stamp
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "map";

		// Define R and t via tf, convert to Pose message and publish
		tf::Transform new_transform;
		tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);

		new_transform.setOrigin(tf::Vector3(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2)));
		new_transform.setRotation(quaternion);

		tf::poseTFToMsg(new_transform, pose.pose);
		pub.publish(pose);
	}

	void ROSPublisher::RequestFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		finish_requested_ = true;
	}

	bool ROSPublisher::CheckFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		return finish_requested_;
	}

	void ROSPublisher::SetFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		finished_ = true;
		unique_lock<mutex> lock2(mtx_stop_);
		stopped_ = true;
	}

	bool ROSPublisher::IsFinished()
	{
		unique_lock<mutex> lock(mtx_finish_);
		return finished_;
	}
}
//
// Created by lorenz on 14.03.22.
//

#include "GridMapping.h"

namespace ORB_SLAM2
{
	GridMapping::GridMapping(Map* map, ros::NodeHandle& nh) :
		Map_(map),
		nh_(nh),
		queue_size_(1000),
		finished_(false),
		stopped_(false),
		finish_requested_(false)
	{
	}

	void GridMapping::Run()
	{
		finished_ = false;
		ros::Publisher gridmap_pub = nh_.advertise<nav_msgs::OccupancyGrid>("os2_gridMap", queue_size_);
		ros::Rate rate(10);

		while (true)
		{
			if (Tracker_ && LoopCloser_)
			{
				if (Tracker_->mCurrentFrame.is_keyframe_ && !LoopCloser_->loop_closed_)
				{
					// TODO: Implement grid mapping algorithm

					GetKFPose();
					GetKFMapPoints();
					UpdateGridMap();
					PublishGridMap();
				}
				else if (LoopCloser_->loop_closed_)
				{
					// TODO: Republish all updated KF poses and MP locations and update grid map

					std::cout << "Loop detected!" << endl;
				}
			}

			rate.sleep();

			if (CheckFinish())
				break;
		}

		SetFinish();
	}

	void GridMapping::UpdateGridMap()
	{
		float kf_pose_x = pose_.position.x;
		float kf_pose_z = pose_.position.z;
		int kf_pose_grid_x = int(floor(kf_pose_x - grid_min_x));
		int kf_pose_grid_z = int(floor(kf_pose_z - grid_min_z));

		CastBeam();
	}

	void GridMapping::PublishGridMap()
	{
		// TODO
	}

	void GridMapping::GetKFMapPoints()
	{
		KeyFrame* current_KF = Tracker_->mCurrentFrame.mpReferenceKF;
		kf_mps_ = current_KF->GetMPs();
	}

	void GridMapping::GetKFPose()
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

		pose_.position.x = twc.at<float>(0);
		pose_.position.y = twc.at<float>(1);
		pose_.position.z = twc.at<float>(2);

		pose_.orientation.x = q[0];
		pose_.orientation.y = q[1];
		pose_.orientation.z = q[2];
		pose_.orientation.w = q[3];
	}

	void GridMapping::CastBeam()
	{
		// TODO
		std::cout << "CastBeam." << endl;
	}

	template<typename T>
	pcl::PointCloud<pcl::PointXYZ> GridMapping::ConvertToPCL(T& mps)
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

	void GridMapping::PublishPC(pcl::PointCloud<pcl::PointXYZ>& pub_cld, ros::Publisher& pub)
	{
		sensor_msgs::PointCloud2 pc2_cld;
		pcl::toROSMsg(pub_cld, pc2_cld);

		pc2_cld.header.frame_id = "map";
		pc2_cld.header.stamp = ros::Time::now();
		pc2_cld.row_step = (pc2_cld.point_step * pc2_cld.width);
		pc2_cld.data.resize(pc2_cld.height * pc2_cld.row_step);

		pub.publish(pc2_cld);
	}

	void GridMapping::PublishKFPose(cv::Mat& Tcw, ros::Publisher& pub)
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

	void GridMapping::SetTracker(Tracking* Tracker)
	{
		Tracker_ = Tracker;
	}

	void GridMapping::SetLoopCloser(LoopClosing* LoopCloser)
	{
		LoopCloser_ = LoopCloser;
	}

	void GridMapping::SetLocalMapper(LocalMapping* LocalMapper)
	{
		LocalMapper_ = LocalMapper;
	}

	void GridMapping::RequestFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		finish_requested_ = true;
	}

	bool GridMapping::CheckFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		return finish_requested_;
	}

	void GridMapping::SetFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		finished_ = true;
		unique_lock<mutex> lock2(mtx_stop_);
		stopped_ = true;
	}

	bool GridMapping::IsFinished()
	{
		unique_lock<mutex> lock(mtx_finish_);
		return finished_;
	}
}
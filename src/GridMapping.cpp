//
// Created by lorenz on 14.03.22.
//

#include "GridMapping.h"

namespace ORB_SLAM2
{
	GridMapping::GridMapping(Map* map, ros::NodeHandle& nh) :
		Map_(map),
		queue_size_(1000),
		gridmap_pub_(nh.advertise<nav_msgs::OccupancyGrid>("os2_gm", queue_size_)),
		pose_pub_(nh.advertise<geometry_msgs::PoseStamped>("os2_pose", queue_size_)),
		pc_pub_(nh.advertise<sensor_msgs::PointCloud2>("os2_pointcloud", queue_size_)),
		finished_(false),
		stopped_(false),
		finish_requested_(false)
	{
	}

	void GridMapping::Run()
	{
		finished_ = false;
		ros::Rate rate(10);

		InitGridMap();

		while (true)
		{
			if (Tracker_ && LoopCloser_)
			{
				if (Tracker_->mCurrentFrame.is_keyframe_ && !LoopCloser_->loop_closed_)
				{
					counter_++;

					GetPose();
					GetMapPoints();
					UpdateGridMap();
					BuildOccupancyGridMsg();
					PublishGridMap();
					PublishGridMapPose();
				}
				else if (LoopCloser_->loop_closed_)
				{
					LoopCloser_->loop_closed_ = false;
					ResetGridMap();
					std::vector<KeyFrame*> all_KFs = Map_->GetAllKeyFrames();

					cout << "Counter_ = " << counter_ << endl;
					cout << "Size of all_KFs = " << all_KFs.size() << endl;

					for (auto kf: all_KFs)
					{
						GetPose(kf);
						GetMapPoints(kf);
						UpdateGridMap();
						BuildOccupancyGridMsg();
						PublishGridMap();
					}
				}
			}

			rate.sleep();

			if (CheckFinish())
				break;
		}

		SetFinish();
	}

	void GridMapping::InitGridMap()
	{
		// TODO: Parse grid map parameters from OS2 setting file

		// Initalize GridMap struct
		gmap_.scale_factor = 3;

		gmap_.max_x = 1000 * gmap_.scale_factor;
		gmap_.max_z = 1600 * gmap_.scale_factor;
		gmap_.min_x = -1000 * gmap_.scale_factor;
		gmap_.min_z = -500 * gmap_.scale_factor;

		gmap_.size_x = gmap_.max_x - gmap_.min_x;
		gmap_.size_z = gmap_.max_z - gmap_.min_z;

		gmap_.norm_factor_x = float(gmap_.size_x - 1) / float(gmap_.size_x);
		gmap_.norm_factor_z = float(gmap_.size_z - 1) / float(gmap_.size_z);

		gmap_.data.create(gmap_.size_z, gmap_.size_x, CV_32FC1);
		gmap_.occupied_counter.create(gmap_.size_z, gmap_.size_x, CV_32SC1);
		gmap_.visit_counter.create(gmap_.size_z, gmap_.size_x, CV_32SC1);
		gmap_.occupied_counter.setTo(cv::Scalar(0));
		gmap_.visit_counter.setTo(cv::Scalar(0));

		gmap_.visit_threshold = 0;
		gmap_.free_threshold = 0.7;
		gmap_.occ_threshold = 0.5;

		// Initialize ROS Occupancy Grid message
		grid_map_msg_.data.resize(gmap_.size_z * gmap_.size_x);
		grid_map_msg_.info.width = gmap_.size_x;
		grid_map_msg_.info.height = gmap_.size_z;
		grid_map_msg_.info.resolution = 1.0 / gmap_.scale_factor;

		grid_map_int_ = cv::Mat(gmap_.size_z, gmap_.size_x, CV_8SC1, grid_map_msg_.data.data());
	}

	void GridMapping::UpdateGridMap()
	{
		float kf_pose_x = pose_.position.x * gmap_.scale_factor;
		float kf_pose_z = pose_.position.z * gmap_.scale_factor;
		pose_.position.kf_pose_grid_x = int(floor((kf_pose_x - gmap_.min_x) * gmap_.norm_factor_x));
		pose_.position.kf_pose_grid_z = int(floor((kf_pose_z - gmap_.min_z) * gmap_.norm_factor_z));

		if (pose_.position.kf_pose_grid_x < 0 || pose_.position.kf_pose_grid_z < 0 || pose_.position.kf_pose_grid_x >= gmap_.size_x
			|| pose_.position.kf_pose_grid_z >= gmap_.size_z)
			return;

		for (auto mp : kf_mps_)
		{
			float mp_pos_x = mp->GetWorldPos().at<float>(0) * gmap_.scale_factor;
			float mp_pos_z = mp->GetWorldPos().at<float>(2) * gmap_.scale_factor;

			int mp_pos_grid_x = int(floor((mp_pos_x - gmap_.min_x) * gmap_.norm_factor_x));
			int mp_pos_grid_z = int(floor((mp_pos_z - gmap_.min_z) * gmap_.norm_factor_z));

			if (mp_pos_grid_x < 0 || mp_pos_grid_z < 0 || mp_pos_grid_x >= gmap_.size_x
				|| mp_pos_grid_z >= gmap_.size_z)
				return;

			++gmap_.occupied_counter.at<int>(mp_pos_grid_z, mp_pos_grid_x);
			CastLaserBeam(pose_.position.kf_pose_grid_x, pose_.position.kf_pose_grid_z, mp_pos_grid_x, mp_pos_grid_z);
		}
	}

	void GridMapping::BuildOccupancyGridMsg()
	{
		for (size_t i = 0; i < gmap_.size_z; i++)
		{
			for (size_t j = 0; j < gmap_.size_x; j++)
			{
				int vc = gmap_.visit_counter.at<int>(i, j);
				int oc = gmap_.occupied_counter.at<int>(i, j);

				if (vc <= gmap_.visit_threshold)
					gmap_.data.at<float>(i, j) = 0.5;
				else
					gmap_.data.at<float>(i, j) = 1 - (float(oc) / float(vc));

				grid_map_int_.at<int8_t>(i, j) = (1 - gmap_.data.at<float>(i, j)) * 100;
			}
		}
	}

	void GridMapping::PublishGridMap()
	{
		grid_map_msg_.info.map_load_time = ros::Time::now();
		grid_map_msg_.header.frame_id = "gridmap";

		if (counter_ == 1)
		{
			grid_map_msg_.info.origin.position.x = pose_.position.x * gmap_.scale_factor;
			grid_map_msg_.info.origin.position.y = pose_.position.z * gmap_.scale_factor;
			grid_map_msg_.info.origin.position.z = pose_.position.y * gmap_.scale_factor;

			grid_map_msg_.info.origin.orientation.x = pose_.orientation.x * gmap_.scale_factor;
			grid_map_msg_.info.origin.orientation.y = pose_.orientation.y * gmap_.scale_factor;
			grid_map_msg_.info.origin.orientation.z = pose_.orientation.z * gmap_.scale_factor;
			grid_map_msg_.info.origin.orientation.w = pose_.orientation.w * gmap_.scale_factor;
		}

		gridmap_pub_.publish(grid_map_msg_);
	}

	void GridMapping::ResetGridMap()
	{
		cout << "Loop detected. Resetting grid map!" << endl;
		gmap_.visit_counter.setTo(0);
		gmap_.occupied_counter.setTo(0);
	}

	void GridMapping::GetMapPoints()
	{
		KeyFrame* current_KF = Tracker_->mCurrentFrame.mpReferenceKF;
		kf_mps_ = current_KF->GetMPs();
	}

	void GridMapping::GetMapPoints(KeyFrame* kf)
	{
		kf_mps_ = kf->GetMPs();
	}

	void GridMapping::GetPose()
	{
		KeyFrame* current_KF = Tracker_->mCurrentFrame.mpReferenceKF;

		cv::Mat Tcw = current_KF->GetPose();
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

		pose_.position.x = twc.at<float>(0);
		pose_.position.y = twc.at<float>(1);
		pose_.position.z = twc.at<float>(2);

		pose_.orientation.x = q[0];
		pose_.orientation.y = q[1];
		pose_.orientation.z = q[2];
		pose_.orientation.w = q[3];
	}

	void GridMapping::GetPose(KeyFrame* kf)
	{
		cv::Mat Tcw = kf->GetPose();
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

		pose_.position.x = twc.at<float>(0);
		pose_.position.y = twc.at<float>(1);
		pose_.position.z = twc.at<float>(2);

		pose_.orientation.x = q[0];
		pose_.orientation.y = q[1];
		pose_.orientation.z = q[2];
		pose_.orientation.w = q[3];
	}

	void GridMapping::CastLaserBeam(int& x0, int& y0, int& x1, int& y1)
	{
		bool steep = (abs(y1 - y0) > abs(x1 - x0));
		if (steep)
		{
			swap(x0, y0);
			swap(x1, y1);
		}
		if (x0 > x1)
		{
			swap(x0, x1);
			swap(y0, y1);
		}
		int dx = x1 - x0;
		int dy = abs(y1 - y0);

		double error = 0;
		double deltaerr = ((double)dy) / ((double)dx);

		int y = y0;
		int ystep = (y0 < y1) ? 1 : -1;
		for (int x = x0; x <= x1; ++x)
		{
			if (steep)
			{
				++gmap_.visit_counter.at<int>(x, y);
			}
			else
			{
				++gmap_.visit_counter.at<int>(y, x);
			}
			error = error + deltaerr;
			if (error >= 0.5)
			{
				y = y + ystep;
				error = error - 1.0;
			}
		}
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

	void GridMapping::PublishPC()
	{
		pcl::PointCloud<pcl::PointXYZ> pub_cld = GridMapping::ConvertToPCL(kf_mps_);

		sensor_msgs::PointCloud2 pc2_cld;
		pcl::toROSMsg(pub_cld, pc2_cld);

		pc2_cld.header.frame_id = "gridmap";
		pc2_cld.header.stamp = ros::Time::now();
		pc2_cld.row_step = (pc2_cld.point_step * pc2_cld.width);
		pc2_cld.data.resize(pc2_cld.height * pc2_cld.row_step);

		pc_pub_.publish(pc2_cld);
	}

	void GridMapping::PublishGridMapPose()
	{
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.header.stamp = ros::Time::now();
		pose_msg.header.frame_id = "gridmap";

		pose_msg.pose.position.x = pose_.position.kf_pose_grid_x;
		pose_msg.pose.position.y = pose_.position.kf_pose_grid_z;
		pose_msg.pose.position.z = 0;

		pose_msg.pose.orientation.x = pose_.orientation.x * gmap_.scale_factor;
		pose_msg.pose.orientation.y = pose_.orientation.y * gmap_.scale_factor;
		pose_msg.pose.orientation.z = pose_.orientation.z * gmap_.scale_factor;
		pose_msg.pose.orientation.w = pose_.orientation.w * gmap_.scale_factor;

		pose_pub_.publish(pose_msg);
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
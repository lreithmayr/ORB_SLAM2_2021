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
		ros::Rate rate(10);

		InitGridMap();

		while (true)
		{
			if (Tracker_ && LoopCloser_)
			{
				if (Tracker_->mCurrentFrame.is_keyframe_ && !LoopCloser_->loop_closed_)
				{
					GetPose();
					GetMapPoints();
					UpdateGridMap();
					PublishGridMap();
				}
				else if (LoopCloser_->loop_closed_)
				{
					// TODO: Republish all updated KF poses and MP locations and update grid map
					std::cout << "Loop detected!" << endl;
					break;
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
		ros::Publisher gridmap_pub = nh_.advertise<nav_msgs::OccupancyGrid>("os2_gridMap", queue_size_);
		gmap_.scale_factor = 2;

		gmap_.max_x = 10 * gmap_.scale_factor;
		gmap_.max_z = 16 * gmap_.scale_factor;
		gmap_.min_x = -10 *gmap_.scale_factor;
		gmap_.min_z = -5 * gmap_.scale_factor;

		gmap_.res_x = gmap_.max_x - gmap_.min_x;
		gmap_.res_z = gmap_.max_z - gmap_.min_z;

		gmap_.data.create(gmap_.res_z, gmap_.res_x, CV_32FC1);
		gmap_.occupied_counter.create(gmap_.res_z, gmap_.res_x, CV_32SC1);
		gmap_.visit_counter.create(gmap_.res_z, gmap_.res_x, CV_32SC1);
		gmap_.occupied_counter.setTo(cv::Scalar(0));
		gmap_.visit_counter.setTo(cv::Scalar(0));
	}

	void GridMapping::UpdateGridMap()
	{
		float kf_pose_x = pose_.position.x;
		float kf_pose_z = pose_.position.z;
		int kf_pose_grid_x = int(floor(kf_pose_x - gmap_.min_x));
		int kf_pose_grid_z = int(floor(kf_pose_z - gmap_.min_z));

		if (kf_pose_grid_x < 0 || kf_pose_grid_z < 0 || kf_pose_grid_x  >= gmap_.res_x || kf_pose_grid_z >= gmap_.res_z)
			return;

		for (auto mp: kf_mps_)
		{
			float mp_pos_x = mp->GetWorldPos().at<float>(0);
			float mp_pos_z = mp->GetWorldPos().at<float>(2);

			int mp_pos_grid_x = int(floor(mp_pos_x - gmap_.min_x));
			int mp_pos_grid_z = int(floor(mp_pos_z - gmap_.min_z));

			if (mp_pos_grid_x < 0 || mp_pos_grid_z < 0 || mp_pos_grid_x  >= gmap_.res_x || mp_pos_grid_z >= gmap_.res_z)
				return;

			++gmap_.occupied_counter.at<int>(mp_pos_grid_z, mp_pos_grid_x);
			// TODO: CastBeam();
		}

	}


	void GridMapping::PublishGridMap()
	{
		// TODO
	}

	void GridMapping::GetMapPoints()
	{
		KeyFrame* current_KF = Tracker_->mCurrentFrame.mpReferenceKF;
		kf_mps_ = current_KF->GetMPs();
	}

	void GridMapping::GetPose()
	{
		KeyFrame* current_KF = Tracker_->mCurrentFrame.mpReferenceKF;
		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

		vector<ORB_SLAM2::KeyFrame*> all_KFs = Map_->GetAllKeyFrames();
		sort(all_KFs.begin(), all_KFs.end(), ORB_SLAM2::KeyFrame::lId);

		cv::Mat Two = all_KFs[0]->GetPoseInverse();

		Trw = Trw * current_KF->GetPose() * Two;
		cv::Mat curr_rel_framepose = Tracker_->mlRelativeFramePoses.back();
		cv::Mat Tcw = curr_rel_framepose * Trw;
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

	void GridMapping::CastBeam(int& x1, int& y1, int& x2, int& y2)
	{
		// Bresenham's line algorithm
		const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
		if(steep)
		{
			std::swap(x1, y1);
			std::swap(x2, y2);
		}

		if(x1 > x2)
		{
			std::swap(x1, x2);
			std::swap(y1, y2);
		}

		const int dx = x2 - x1;
		const int dy = abs(y2 - y1);

		auto error = dx / 2;
		const int y_step = (y1 < y2) ? 1 : -1;
		int y = y1;

		for(int x=x1; x<=x2; x++)
		{
			if(steep)
			{
				++gmap_.visit_counter.at<int>(x, y);
			}
			else
			{
				++gmap_.visit_counter.at<int>(y,x);
			}

			error -= dy;
			if(error < 0)
			{
				y += y_step;
				error += dx;
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
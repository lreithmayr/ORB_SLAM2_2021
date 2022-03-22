//
// Created by lorenz on 14.03.22.
//

#include "ROSPublisher.h"

namespace ORB_SLAM2
{
	ROSPublisher::ROSPublisher(Map* map, ros::NodeHandle& nh) :
		map_(map),
		nh_(nh),
		queue_size_(1),
		viewer_("PCL Viewer"),
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
		ros::Publisher pub_mps = nh_.advertise<sensor_msgs::PointCloud2>("pc_allMPs", queue_size_);
		ros::Publisher pub_KFmps = nh_.advertise<sensor_msgs::PointCloud2>("pc_KFmps", queue_size_);
		ros::Publisher pub_KFpose = nh_.advertise<geometry_msgs::PoseStamped>("kf_pose", queue_size_);

		while (true)
		{
			if (!map_->GetAllKeyFrames().empty() && !map_->GetAllMapPoints().empty())
			{
				std::vector<MapPoint*> mps = GetAllMPs();
				set<MapPoint*> mps_in_KF = GetKFMapPoints();
				cv::Mat KF_pose = GetKFPose();

				pcl::PointCloud<pcl::PointXYZ> mps_pcl = ConvertToPCL(mps);
				pcl::PointCloud<pcl::PointXYZ> mps_kf_pcl = ConvertToPCL(mps_in_KF);

				pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cld(new pcl::PointCloud<pcl::PointXYZ>);
				*ptr_cld = mps_kf_pcl;
				viewer_.showCloud(ptr_cld);

				PublishKFPose(KF_pose, pub_KFpose);
				PublishPC(mps_pcl, pub_mps);
				PublishPC(mps_kf_pcl, pub_KFmps);
			}

			rate.sleep();

			if (CheckFinish())
				break;
		}

		SetFinish();
	}

	std::vector<MapPoint*> ROSPublisher::GetAllMPs()
	{
		return map_->GetAllMapPoints();
	}

	set<MapPoint*> ROSPublisher::GetKFMapPoints()
	{
		vector<KeyFrame*> all_KFs = map_->GetAllKeyFrames();
		KeyFrame* most_recent_KF = all_KFs.back();
		set<MapPoint*> mps_in_KF = most_recent_KF->GetMapPoints();

		return mps_in_KF;
	}

	cv::Mat ROSPublisher::GetKFPose()
	{
		vector<KeyFrame*> all_KFs = map_->GetAllKeyFrames();
		KeyFrame* most_recent_KF = all_KFs.back();
		cv::Mat Tcw = most_recent_KF->GetPose();

		return Tcw;
	}

	pcl::PointCloud<pcl::PointXYZ> ROSPublisher::ConvertToPCL(std::vector<MapPoint*>& mps)
	{
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl_cloud.width = mps.size();
		pcl_cloud.height = 1;
		pcl_cloud.is_dense = true;
		pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

		for (uint32_t i = 0; i < pcl_cloud.width; i++)
		{
			pcl_cloud[i].x = mps[i]->GetWorldPos().at<float>(0);
			pcl_cloud[i].y = mps[i]->GetWorldPos().at<float>(1);
			pcl_cloud[i].z = mps[i]->GetWorldPos().at<float>(2);
		}

		return pcl_cloud;
	}

	pcl::PointCloud<pcl::PointXYZ> ROSPublisher::ConvertToPCL(set<MapPoint*>& mps)
	{
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl_cloud.width = mps.size();
		pcl_cloud.height = 1;
		pcl_cloud.is_dense = true;
		pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

		for (auto mp: mps)
		{
			float x = mp->GetWorldPos().at<float>(0);
			float y = mp->GetWorldPos().at<float>(1);
			float z = mp->GetWorldPos().at<float>(2);

			pcl_cloud.emplace_back(pcl::PointXYZ(x, y, z));
		}

		return pcl_cloud;
	}

	void ROSPublisher::PublishPC(pcl::PointCloud<pcl::PointXYZ>& pub_cld, ros::Publisher& pub)
	{
		//Convert ROS time stamp to PCL time stamp
		// pcl_conversions::toPCL(ros::Time::now(), pub_cld.header.stamp);

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
		cv::Mat twc = -Rwc*tcw;

		// TODO: Synchronize pose time stamp and pc time stamp
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "map";

		// Define R and t via tf, convert to Pose message and publish
		tf::Transform new_transform;
		tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);

		new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));
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
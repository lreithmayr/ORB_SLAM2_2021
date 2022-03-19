//
// Created by lorenz on 14.03.22.
//

#include "PointCloudPublisher.h"

namespace ORB_SLAM2
{
	PointCloudPublisher::PointCloudPublisher(Map* map, bool visualize_pc, ros::NodeHandle& nh):
		map_(map),
		nh_pc_(nh),
		topic_("os2_point_cloud"),
		queue_size_(10),
		pub_pc_(nh_pc_.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic_, queue_size_)),
		visualize_pc_(visualize_pc),
		finished_(false),
		stopped_(false),
		finish_requested_(false)
	{
	}

	void PointCloudPublisher::SetTracker(Tracking* Tracker)
	{
		Tracker_ = Tracker;
	}

	void PointCloudPublisher::SetLoopCloser(LoopClosing* LoopCloser)
	{
		LoopCloser_ = LoopCloser;
	}

	void PointCloudPublisher::SetLocalMapper(LocalMapping* LocalMapper)
	{
		LocalMapper_ = LocalMapper;
	}

	void PointCloudPublisher::Run()
	{
		finished_ = false;
		pcl::visualization::CloudViewer viewer("PCL Viewer");

		while (true)
		{
			std::vector<MapPoint*> mps = GetAllMPs();
			if (mps.empty())
				continue;
			else
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr mps_pcl = ConvertToPCL(mps);

				if (visualize_pc_)
				{
					viewer.showCloud(mps_pcl);
				}

				PublishPC(mps_pcl);

				// TestPublisher();
			}

			if (CheckFinish())
				break;
		}

		SetFinish();
	}

	std::vector<MapPoint*> PointCloudPublisher::GetAllMPs()
	{
		return map_->GetAllMapPoints();
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPublisher::ConvertToPCL(std::vector<MapPoint*>& mps)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl_cloud->width = mps.size();
		pcl_cloud->height = 1;
		pcl_cloud->is_dense = true;
		pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

		for(uint32_t i = 0; i < pcl_cloud->width; i++)
		{
			float x = mps[i]->GetWorldPos().at<float>(0);
			float y = mps[i]->GetWorldPos().at<float>(1);
			float z = mps[i]->GetWorldPos().at<float>(2);

			pcl_cloud->points.emplace_back(pcl::PointXYZ(x, y, z));
		}

		return pcl_cloud;
	}

	void PointCloudPublisher::PublishPC(pcl::PointCloud<pcl::PointXYZ>::Ptr& pub_cld)
	{
		ros::Rate rate(10);

		//Convert ROS time stamp to PCL time stamp
		pcl_conversions::toPCL(ros::Time::now(), pub_cld->header.stamp);

		// Convert PCL cloud "pub_cld" to sensor_msgs::PointCloud2 out_cld
		sensor_msgs::PointCloud2 out_cld;
		pcl::toROSMsg(*pub_cld, out_cld);

		// ROS_INFO("%s", out_cld.header.frame_id.c_str());

		pub_pc_.publish(out_cld);
		rate.sleep();
	}

	void PointCloudPublisher::TestPublisher()
	{
		ros::Publisher chatter_pub = nh_pc_.advertise<std_msgs::String>("chatter", 1000);
		ros::Rate loop_rate(10);

		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << "\n";
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		loop_rate.sleep();
	}

	void PointCloudPublisher::RequestFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		finish_requested_ = true;
	}

	bool PointCloudPublisher::CheckFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		return finish_requested_;
	}

	void PointCloudPublisher::SetFinish()
	{
		unique_lock<mutex> lock(mtx_finish_);
		finished_ = true;
		unique_lock<mutex> lock2(mtx_stop_);
		stopped_ = true;
	}

	bool PointCloudPublisher::IsFinished()
	{
		unique_lock<mutex> lock(mtx_finish_);
		return finished_;
	}
}
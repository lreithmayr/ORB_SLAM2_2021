//
// Created by lorenz on 14.03.22.
//

#include "PointCloudPublisher.h"

namespace ORB_SLAM2
{
	PointCloudPublisher::PointCloudPublisher(Map* map, bool visualize_pc, ros::NodeHandle& nh) :
		map_(map),
		nh_pc_(nh),
		topic_("os2_point_cloud"),
		queue_size_(1),
		pub_pc_(nh_pc_.advertise<sensor_msgs::PointCloud2>(topic_, queue_size_)),
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
		ros::Rate rate(10);

		while (true)
		{
			std::vector<MapPoint*> mps = GetAllMPs();
			if (mps.empty())
				continue;
			else
			{
				pcl::PointCloud<pcl::PointXYZ> mps_pcl = ConvertToPCL(mps);

				if (visualize_pc_)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cld(new pcl::PointCloud<pcl::PointXYZ>);
					*ptr_cld = mps_pcl;
					viewer.showCloud(ptr_cld);
				}

				PublishPC(mps_pcl);
				rate.sleep();
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

	pcl::PointCloud<pcl::PointXYZ> PointCloudPublisher::ConvertToPCL(std::vector<MapPoint*>& mps)
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

	void PointCloudPublisher::PublishPC(pcl::PointCloud<pcl::PointXYZ>& pub_cld)
	{
		//Convert ROS time stamp to PCL time stamp
		pcl_conversions::toPCL(ros::Time::now(), pub_cld.header.stamp);

		sensor_msgs::PointCloud2 pc2_cld;
		pcl::toROSMsg(pub_cld, pc2_cld);

		pc2_cld.header.frame_id = "map";
		pc2_cld.row_step = (pc2_cld.point_step * pc2_cld.width);
		pc2_cld.data.resize(pc2_cld.height * pc2_cld.row_step);

		pub_pc_.publish(pc2_cld);
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
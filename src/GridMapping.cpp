//
// Created by lorenz on 14.03.22.
//

#include "GridMapping.h"

namespace ORB_SLAM2
{
	GridMapping::GridMapping(Map* map, bool visualize_pc):
	map_(map),
	nh_(),
	topic_("point_cloud"),
	queue_size_(10),
	visualize_pc_(visualize_pc),
	finished_(false),
	stopped_(false),
	finish_requested_(false)
	{
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

	void GridMapping::Run()
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
				pcl::PointCloud<pcl::PointXYZ> mps_pcl = ConvertToPCL(mps);

				if (visualize_pc_)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
					*pc_ptr = mps_pcl;
					viewer.showCloud(pc_ptr);
				}

				PublishPC(mps_pcl);
			}

			if (CheckFinish())
				break;
		}

		SetFinish();
	}

	std::vector<MapPoint*> GridMapping::GetAllMPs()
	{
		return map_->GetAllMapPoints();
	}

	pcl::PointCloud<pcl::PointXYZ> GridMapping::ConvertToPCL(std::vector<MapPoint*>& mps)
	{
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl_cloud.width = mps.size();
		pcl_cloud.height = 1;
		pcl_cloud.is_dense = true;
		pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

		for(uint32_t i = 0; i < pcl_cloud.width; i++)
		{
			pcl_cloud[i].x = mps[i]->GetWorldPos().at<float>(0);
			pcl_cloud[i].y = mps[i]->GetWorldPos().at<float>(1);
			pcl_cloud[i].z = mps[i]->GetWorldPos().at<float>(2);
			// pcl_cloud[i].id =mps[i]->mnId;
		}

		return pcl_cloud;
	}

	void GridMapping::PublishPC(pcl::PointCloud<pcl::PointXYZ>& pub_cld)
	{
		ros::Rate rate(10);
		pcl_conversions::toPCL(ros::Time::now(), pub_cld.header.stamp);
		ros::Publisher pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>> (topic_, queue_size_);
		pub.publish(pub_cld);
		ros::spinOnce();
		rate.sleep();
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
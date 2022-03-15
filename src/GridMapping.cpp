//
// Created by lorenz on 14.03.22.
//

#include "GridMapping.h"

namespace ORB_SLAM2
{
	GridMapping::GridMapping(Map* map) : map_(map), nh_(),topic_("point_cloud"), queue_size_(1), finished_(false), stopped_(false), finish_requested_(false)
	{
		std::cout << "GridMapping constructed." << endl;
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

		while (true)
		{
			std::vector<MapPoint*> mps = GetAllMPs();
			if (mps.empty())
				continue;
			else
			{
				pcl::PointCloud<PointXYZid> mps_pcl = ConvertToPCL(mps);
				PublishPC(mps_pcl);
				//SubToPC(nh_);
			}

			if (CheckFinish())
				break;

			// std::this_thread::sleep_for(std::chrono::microseconds(3000));
		}

		SetFinish();
	}

	std::vector<MapPoint*> GridMapping::GetAllMPs()
	{
		return map_->GetAllMapPoints();
	}

	pcl::PointCloud<PointXYZid> GridMapping::ConvertToPCL(std::vector<MapPoint*>& mps)
	{
		pcl::PointCloud<PointXYZid> pcl_cloud;
		pcl_cloud.width = mps.size();
		pcl_cloud.height = 1;
		pcl_cloud.is_dense = true;
		pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

		for(uint32_t i = 0; i < pcl_cloud.width; i++)
		{
			pcl_cloud[i].x = mps[i]->GetWorldPos().at<float>(0);
			pcl_cloud[i].y = mps[i]->GetWorldPos().at<float>(1);
			pcl_cloud[i].z = mps[i]->GetWorldPos().at<float>(2);
			pcl_cloud[i].id =mps[i]->mnId;
		}

		return pcl_cloud;
	}

	void GridMapping::PublishPC(pcl::PointCloud<PointXYZid>& pub_cld)
	{
		ros::Publisher pub = nh_.advertise<pcl::PointCloud<PointXYZid>> (topic_, queue_size_);
		pub.publish(pub_cld);
		std::cout << "PC publishing" << endl;
	}

	// void GridMapping::SubToPC(ros::NodeHandle nh)
	// {
	// 	void callback(const sensor_msgs::PointCloud2ConstPtr&);
	// 	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (topic_, queue_size_, callback);
	// }

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
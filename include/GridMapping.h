// Grid Mapping class

#ifndef GRIDMAPPING_H
#define GRIDMAPPING_H

#include <System.h>
#include <Map.h>
#include <Tracking.h>
#include <LocalMapping.h>
#include <LoopClosing.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

#include <thread>
#include <mutex>
#include <optional>

namespace ORB_SLAM2
{
	class Tracking;
	class LocalMapping;
	class LoopClosing;

	class GridMapping
	{
	 public:
		GridMapping(Map* map, ros::NodeHandle& nh);

		// Main function
		void Run();

		// Camera pose containing position and orientation as quaternion
		struct CameraPose
		{
			struct Position
			{
				float x;
				float y;
				float z;
			};
			Position position;

			struct Orientation
			{
				float x;
				float y;
				float z;
				float w;
			};
			Orientation orientation;
		};

		void GetKFMapPoints();
		void GetKFPose();
		void CastBeam();
		void UpdateGridMap();

		// PCL conversion and ROS publishers
		template<typename T>
		pcl::PointCloud<pcl::PointXYZ> ConvertToPCL(T& mps);

		static void PublishPC(pcl::PointCloud<pcl::PointXYZ>& pub_cld, ros::Publisher& pub);
		static void PublishKFPose(cv::Mat& pose, ros::Publisher& pub);
		void PublishGridMap();

		// Set thread pointers
		void SetTracker(Tracking* Tracker);
		void SetLoopCloser(LoopClosing* LoopCloser);
		void SetLocalMapper(LocalMapping* LocalMapper);

		// Public thread sync stuff
		void RequestFinish();
		bool IsFinished();

	 private:
		// Class member variables
		Map* Map_;
		CameraPose pose_{};
		std::vector<MapPoint*> all_mps_{};
		std::vector<MapPoint*> kf_mps_{};
		nav_msgs::OccupancyGrid os2_gm_msg_;

		// ROS variables
		ros::NodeHandle nh_;
		uint32_t queue_size_;

		// Thread pointers
		Tracking* Tracker_{};
		LoopClosing* LoopCloser_{};
		LocalMapping* LocalMapper_{};

		// Private thread sync stuff
		bool CheckFinish();
		void SetFinish();

		bool finished_;
		bool stopped_;
		bool finish_requested_;

		// Mutexes
		std::mutex mtx_finish_;
		std::mutex mtx_stop_;
		std::mutex mtx_state_;
	};
}

#endif //GRIDMAPPING_H
// Grid Mapping class

#ifndef GRIDMAPPING_H
#define GRIDMAPPING_H

#include "Map.h"

#include <thread>
#include <mutex>

namespace ORB_SLAM2
{
	class Map;

	class GridMapping
	{
	 public:
		explicit GridMapping(Map* map);

		void Run();

		std::vector<MapPoint*> GetAllMPs();

		// Public thread sync stuff
		void RequestFinish();

		bool IsFinished();

	 private:
		Map* map_;

		// Private thread sync stuff
		bool CheckFinish();

		void SetFinish();

		std::mutex mtx_finish_;
		std::mutex mtx_stop_;
		bool finished_;
		bool stopped_;
		bool finish_requested_;
	};

}

#endif //GRIDMAPPING_H

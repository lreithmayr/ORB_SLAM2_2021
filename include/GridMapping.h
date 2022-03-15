// Grid Mapping class

#ifndef GRIDMAPPING_H
#define GRIDMAPPING_H

#include "Map.h"
#include "Tracking.h"
#include "LocalMapping.h"
#include "LoopClosing.h"

#include <thread>
#include <mutex>

namespace ORB_SLAM2
{
	class Tracking;
	class LocalMapping;
	class LoopClosing;

	class GridMapping
	{
	 public:
		explicit GridMapping(Map* map);

		// Set other thread pointers
		void SetTracker(Tracking* Tracker);
		void SetLoopCloser(LoopClosing* LoopCloser);
		void SetLocalMapper(LocalMapping* LocalMapper);

		// Main method
		void Run();

		std::vector<MapPoint*> GetAllMPs();

		// Public thread sync stuff
		void RequestFinish();

		bool IsFinished();

	 private:
		Map* map_;

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
	};

}

#endif //GRIDMAPPING_H

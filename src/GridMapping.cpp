//
// Created by lorenz on 14.03.22.
//

#include "GridMapping.h"

namespace ORB_SLAM2
{
	GridMapping::GridMapping(Map* map) : map_(map), finished_(false), stopped_(false), finish_requested_(false)
	{
		std::cout << "GridMapping constructed." << endl;
	}

	void GridMapping::Run()
	{
		finished_ = false;

		while (true)
		{
			std::vector<MapPoint*> all_mps = GetAllMPs();
			std::cout << all_mps.size() << endl;

			if (CheckFinish())
				break;

			std::this_thread::sleep_for(std::chrono::microseconds(3000));
		}

		SetFinish();
	}

	std::vector<MapPoint*> GridMapping::GetAllMPs()
	{
		return map_->GetAllMapPoints();
	}

	void GridMapping::RequestFinish()
	{
		unique_lock<mutex> lock(mutex_finish_);
		finish_requested_ = true;
	}

	bool GridMapping::CheckFinish()
	{
		unique_lock<mutex> lock(mutex_finish_);
		return finish_requested_;
	}

	void GridMapping::SetFinish()
	{
		unique_lock<mutex> lock(mutex_finish_);
		finished_ = true;
		unique_lock<mutex> lock2(mutex_stop_);
		stopped_ = true;
	}

	bool GridMapping::isFinished()
	{
		unique_lock<mutex> lock(mutex_finish_);
		return finished_;
	}
}
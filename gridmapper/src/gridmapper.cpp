#include "MapProcessor.h"

int main()
{
    // Initialize Map Processor and load map from binary file "map.bin"
    std::string map_path =  "../maps/map_07_mono.bin";
    ORB_SLAM2::MapProcessor map(map_path);

    // map.SaveTrajectoryKITTI("../trajectories/stereo_kitti_traj_Quaternions_03red.txt");
    // map.SavePointCloud("../point_clouds/stereo_kitti_mapPoints_03red.txt");

    map.OpenMapPangolin();

}
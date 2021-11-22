#include "MapProcessor.h"

int main()
{
    // Initialize Map Processor and load map from binary file "map.bin"
    ORB_SLAM2::MapProcessor map("../../scripts/map.bin");

    // Process the raw point cloud map into a grid map and open it
    const std::string gridMap_path = "../maps/GridMap_stereoKITTI.pgm";
    // map.SaveGridMapKITTI(gridMap_path);
    // ORB_SLAM2::MapProcessor::OpenMap(gridMap_path);

    // map.SaveTrajectoryKITTI("../trajectories/stereo_kitti_traj_Quaternions.txt");

    map.SavePointCloud("../point_clouds/stereo_kitti_mapPoints.txt");

}
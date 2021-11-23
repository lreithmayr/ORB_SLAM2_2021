#include "MapProcessor.h"

int main()
{
    std::string map_path =  "../../scripts/map_07.bin";

    // Initialize Map Processor and load map from binary file "map.bin"
    ORB_SLAM2::MapProcessor map(map_path);

    // Process the raw point cloud map into a grid map and open it
    /*
    const std::string gridMap_path = "../maps/GridMap_stereoKITTI_03red.pgm";
    map.SaveGridMapKITTI(gridMap_path);
    ORB_SLAM2::MapProcessor::OpenMap(gridMap_path);
    */

    // map.SaveTrajectoryKITTI("../trajectories/stereo_kitti_traj_Quaternions_03red.txt");

    // map.SavePointCloud("../point_clouds/stereo_kitti_mapPoints_03red.txt");

    map.OpenMapPangolin();

}
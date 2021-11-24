#include "MapProcessor.h"

int main()
{
    std::map<std::string, std::string> bin_map { {"Mono", "../maps/map_07_mono.bin"}, {"Stereo", "../maps/map_07_stereo.bin"} };
    std::string camera = "Stereo";

    // Initialize Map Processor and load map from binary file "map.bin"
    std::string map_path =  bin_map[camera];
    ORB_SLAM2::MapProcessor map(map_path);

    // map.SaveTrajectoryKITTI("../trajectories/stereo_kitti_traj_Quaternions_03red.txt");
    // map.SavePointCloud("../point_clouds/stereo_kitti_mapPoints_03red.txt");

    // map.OpenMapPangolin();

    map.ConvertMPsToPCL();
    ORB_SLAM2::MapProcessor::RemoveOutliers("../point_clouds/test_pcd.pcd", "../point_clouds/filtered_test_pcd.pcd");

    ORB_SLAM2::MapProcessor::ViewPC("../point_clouds/filtered_test_pcd.pcd");

    return 0;
}
#include "MapProcessor.h"

int main()
{
    std::map<std::string, std::string> bin_map { {"Mono", "map_07_mono.bin"}, {"Stereo", "map_07_stereo.bin"} };
    std::string camera = "Stereo";

    std::string pc_folder = "../point_clouds/";
    std::string map_folder = "../maps/";

    // Initialize Map Processor and load map from binary file "map.bin"
    std::string map_path =  map_folder + bin_map[camera];
    ORB_SLAM2::MapProcessor map(map_path);

    // Convert map to PCL, filter, and convert it back.
    map.FilterOutliers();
    map.SavePointCloud(pc_folder + "mps_and_kfs_kitti_stereo07.txt");
    map.SaveTrajectoryKITTI(pc_folder + "traj_kitti_stereo07.txt");

    map.OpenMapPangolin();

    return 0;
}
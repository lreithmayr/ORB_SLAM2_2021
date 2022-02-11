#include "MapProcessor.h"

int main()
{
    std::string map_name = "map_07_stereo";

    std::string pc_folder = "../point_clouds/";
    std::string map_folder = "../maps/";

    // Initialize Map Processor and load map from binary file "map.bin"
    std::string map_path = map_folder + map_name + ".bin";
    ORB_SLAM2::MapProcessor map(map_path);

    // Convert map to PCL, filter, and convert it back.
    map.FilterOutliers();
    map.SavePointCloud(pc_folder + "mps_and_kfs_" + map_name + ".txt");
    map.SaveTrajectoryKITTI(pc_folder + "traj_" + map_name + ".txt");

    map.OpenMapPangolin();

    return 0;
}
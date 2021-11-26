#include "MapProcessor.h"

int main()
{
    std::map<std::string, std::string> bin_map { {"Mono", "../maps/map_07_mono.bin"}, {"Stereo", "../maps/map_07_stereo.bin"} };
    std::string camera = "Stereo";

    std::string pc_path = "../point_clouds/";
    std::string filtered = "filtered_pc_stereo07.pcd";
    std::string outliers = "outliers.pcd";
    std::string unfiltered = "unfiltered.pcd";

    // Initialize Map Processor and load map from binary file "map.bin"
    std::string map_path =  bin_map[camera];
    ORB_SLAM2::MapProcessor map(map_path);

    map.FilterOutliers(pc_path + filtered);

    // ORB_SLAM2::MapProcessor::ViewPC(pc_path + filtered);

    map.OpenMapPangolin();

    return 0;
}
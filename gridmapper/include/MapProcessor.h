#define PCL_NO_PRECOMPILE
#ifndef ORB_SLAM2_MOD_MAPPROCESSOR_H
#define ORB_SLAM2_MOD_MAPPROCESSOR_H

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/memory.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include "System.h"
#include "Converter.h"

struct PointXYZid
{
    PCL_ADD_POINT4D;
    uint32_t id;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT
(
    PointXYZid,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, id, id)
)

namespace ORB_SLAM2
{
    class MapProcessor
    {
        public:
        // Constructor initializes binary map, and ORB vocabulary for relocalization
        explicit MapProcessor(const string& filename);

        // Generates occupancy grid map from KFs and Map Points. Bugged.
        void SaveGridMapKITTI(const string& filename);

        // Saves KF trajectory in txt file. Translation t and Quaternions.
        void SaveTrajectoryKITTI(const string& filename);

        // Extracts map points and the timestamps at which they are observed in different KFs
        void SavePointCloud(const string& filename);

        // Opens the generated grid map as a pgm file.
        static void OpenMap(const string& filename);

        void OpenMapPangolin();

        // Converts the MPs saved in the map to a PCL point cloud and saves it as .pcd
        void FilterOutliers(const string& outfn);

        static void ViewPC(const string& pcl_filename);

        private:
        Map* map;
        string mapfile;
        vector<KeyFrame*> KFs;
        KeyFrameDatabase* keyFrameDatabase;
        ORBVocabulary* vocabulary;
        string vocFile;
    };
}

#endif //ORB_SLAM2_MOD_MAPPROCESSOR_H

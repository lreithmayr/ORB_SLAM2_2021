#ifndef ORB_SLAM2_MOD_MAPPROCESSOR_H
#define ORB_SLAM2_MOD_MAPPROCESSOR_H

#endif //ORB_SLAM2_MOD_MAPPROCESSOR_H

#include "System.h"
#include "Converter.h"

namespace ORB_SLAM2
{

class MapProcessor
{
public:
    // Constructor initializes binary map, and ORB vocabulary for relocalization
    explicit MapProcessor(const string &filename);

    // Generates occupancy grid map from KFs and Map Points. Bugged.
    void SaveGridMapKITTI(const string &filename);

    // Saves KF trajectory in txt file. Translation t and Quaternions.
    void SaveTrajectoryKITTI(const string &filename);

    // Extracts map points and the timestamps at which they are observed in different KFs
    void SavePointCloud(const string &filename);

    // Opens the generated grid map as a pgm file.
    static void OpenMap(const string &filename);

private:
    Map* map;
    string mapfile;
    vector<KeyFrame*> KFs;
    KeyFrameDatabase* keyFrameDatabase;
    ORBVocabulary* vocabulary;
    string vocFile;
};

}
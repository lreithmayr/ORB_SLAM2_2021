#ifndef ORB_SLAM2_MOD_MAPPROCESSOR_H
#define ORB_SLAM2_MOD_MAPPROCESSOR_H

#endif //ORB_SLAM2_MOD_MAPPROCESSOR_H

#include "System.h"

namespace ORB_SLAM2
{

class MapProcessor
{
public:
    MapProcessor();
    void loadMap(const string &filename);
    vector<ORB_SLAM2::KeyFrame*> getAllKeyFrames();

private:
    Map* map;
    string mapfile;
    KeyFrameDatabase* keyFrameDatabase;
    ORBVocabulary* vocabulary;
    string vocFile;
};

}
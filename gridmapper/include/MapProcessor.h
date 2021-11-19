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

    vector<KeyFrame*> loadMap(const string &filename);

private:
    Map* map;
    string mapfile;
    KeyFrameDatabase* keyFrameDatabase;
    ORBVocabulary* vocabulary;
    string vocFile;
};

}
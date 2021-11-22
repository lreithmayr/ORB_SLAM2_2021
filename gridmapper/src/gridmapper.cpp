#include "MapProcessor.h"

int main()
{
    ORB_SLAM2::MapProcessor map("../../scripts/map.bin");

    vector<ORB_SLAM2::KeyFrame*> KFs = map.getAllKeyFrames();

    std::cout << KFs.size() << endl;
    return 0;
}
#include "MapProcessor.h"

int main()
{
    ORB_SLAM2::MapProcessor mapProcessor;
    vector<ORB_SLAM2::KeyFrame*> KFs = mapProcessor.loadMap("/home/lorenz/Projects/BA/01_Algorithms/ORB_SLAM2_MOD/map.bin");
    cv::Mat Two = KFs[0]->GetPoseInverse();
    std::cout << Two.at<float>(0, 0) << endl;
    return 0;
}
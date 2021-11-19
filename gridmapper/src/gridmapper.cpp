#include "MapProcessor.h"

int main()
{
    ORB_SLAM2::MapProcessor mapProcessor;
    vector<ORB_SLAM2::KeyFrame*> KFs = mapProcessor.loadMap("map.bin");
    cv::Mat Two = KFs[0]->GetPoseInverse();
    std::cout << Two.at<float>(0, 0) << endl;
    return 0;
}
#include "MapProcessor.h"

int main()
{
    ORB_SLAM2::MapProcessor mapProcessor;

    mapProcessor.loadMap("../../scripts/map.bin");
    vector<ORB_SLAM2::KeyFrame*> KFs = mapProcessor.getAllKeyFrames();

    cv::Mat Two = KFs[0]->GetPoseInverse();
    std::cout << Two.at<float>(0, 0) << endl;
    return 0;
}
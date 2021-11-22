#include "MapProcessor.h"

namespace ORB_SLAM2
{
    MapProcessor::MapProcessor()
    {
        map = new Map();
        keyFrameDatabase = new KeyFrameDatabase();

        // Load ORB Vocabulary
        vocFile = "../../Vocabulary/ORBvoc.bin";
        vocabulary = new ORBVocabulary();
        bool vocLoad = vocabulary->loadFromBinaryFile(vocFile);
        cout << "Vocabulary loaded!" << endl << endl;
    }

    void MapProcessor::loadMap(const string &filename)
    {
        mapfile = filename;
        // unique_lock<mutex>MapPointGlobal(MapPoint::mGlobalMutex);
        std::ifstream in(filename, std::ios_base::binary);
        if (!in)
        {
            cerr << "Cannot Open Mapfile: " << mapfile << " , You need create it first!" << std::endl;
        }
        cout << "Loading Mapfile: " << mapfile << endl;
        boost::archive::binary_iarchive ia(in, boost::archive::no_header);
        ia >> map;
        ia >> keyFrameDatabase;

        keyFrameDatabase->SetORBvocabulary(vocabulary);
        cout << " ...done" << std::endl;
        cout << "Map Reconstructing" << flush;
        vector<ORB_SLAM2::KeyFrame*> KFs = map->GetAllKeyFrames();
        unsigned long mnFrameId = 0;
        for (auto it:KFs) {
            it->SetORBvocabulary(vocabulary);
            it->ComputeBoW();
            if (it->mnFrameId > mnFrameId)
                mnFrameId = it->mnFrameId;
        }
        Frame::nNextId = mnFrameId;
        cout << " ...done" << endl;
        in.close();
    }

    vector<ORB_SLAM2::KeyFrame*> MapProcessor::getAllKeyFrames()
    {
        vector<ORB_SLAM2::KeyFrame*> KFs = map->GetAllKeyFrames();
        return KFs;
    }
}
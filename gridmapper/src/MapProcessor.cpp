#include "MapProcessor.h"

namespace ORB_SLAM2
{
    MapProcessor::MapProcessor(const string &filename): map(new Map), keyFrameDatabase(new KeyFrameDatabase), vocabulary(new ORBVocabulary)
    {
        // Load ORB Vocabulary
        vocFile = "../../Vocabulary/ORBvoc.bin";
        vocabulary->loadFromBinaryFile(vocFile);
        cout << "Vocabulary loaded!" << endl << endl;

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
        KFs = map->GetAllKeyFrames();
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
        return KFs;
    }
}
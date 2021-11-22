#include "MapProcessor.h"

namespace ORB_SLAM2
{
    MapProcessor::MapProcessor(const std::string &filename): map(new Map), keyFrameDatabase(new KeyFrameDatabase), vocabulary(new ORBVocabulary)
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

    /*
    vector<ORB_SLAM2::KeyFrame*> MapProcessor::getAllKeyFrames()
    {
        return KFs;
    }
    */

    void MapProcessor::SaveGridMapKITTI(const string &filename)
    {
        cout << endl << "Saving grid map to " << filename << " ..." << endl;
        vector<MapPoint *> MPs = map->GetAllMapPoints();

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        //f << fixed;

        f << "P2" << endl;

        // Init Grid Statictics
        // We set the resolution as 10 mm, all points and keyframes are in the range of (-3, -4) to (3, 1),
        // so the grid map size is 600x500
        const float upper_left_x = -1.5;
        const float upper_left_y = -2.5;
        const int resolution = 10;
        const int h = 300;
        const int w = 450;
        double grid_occup[w][h];
        double grid_visit[w][h];

        memset(grid_occup, 0, sizeof(grid_occup[0][0]) * w * h);
        memset(grid_visit, 0, sizeof(grid_visit[0][0]) * w * h);

        f << w << " " << h << endl;
        f << 255 << endl;

         for (auto mp : MPs)
        {
            if (mp->isBad())
                continue;
            // Get the grid position of the map point
            cv::Mat MPPositions = mp->GetWorldPos();
            float mp_pos_x = MPPositions.at<float>(0);
            float mp_pos_y = MPPositions.at<float>(1);

            int mp_pos_grid_x = ((int) ((mp_pos_x - upper_left_x) * 1000)) / resolution;
            int mp_pos_grid_y = ((int) ((mp_pos_y - upper_left_y) * 1000)) / resolution;

            if (mp_pos_grid_x < 0 || mp_pos_grid_x >= w)
                continue;

            if (mp_pos_grid_y < 0 || mp_pos_grid_y >= h)
                continue;

            // Increment the occupency account of the grid cell where map point is located
            grid_occup[mp_pos_grid_x][mp_pos_grid_y]++;

            // Get all KeyFrames that observes the map point
            std::map<KeyFrame *, size_t> obsFrames = mp->GetObservations();
            std::map<KeyFrame *, size_t>::iterator it;

            //cout << "----------------------" << endl;
            for (it = obsFrames.begin(); it != obsFrames.end(); it++)
            {
                KeyFrame *oKF = it->first;
                if (oKF->isBad())
                    continue;

                // Get the grid position of the KeyFrame
                cv::Mat t = oKF->GetCameraCenter();
                float okf_pos_x = t.at<float>(0);
                float okf_pos_y = t.at<float>(1);
                int okf_pos_grid_x = ((int) ((okf_pos_x - upper_left_x) * 1000)) / resolution;
                int okf_pos_grid_y = ((int) ((okf_pos_y - upper_left_y) * 1000)) / resolution;

                if (okf_pos_grid_x < 0 || okf_pos_grid_x >= w)
                    continue;

                if (okf_pos_grid_y < 0 || okf_pos_grid_y >= h)
                    continue;

                //cout << okf_pos_grid_x << " " << okf_pos_grid_y << endl;

                // Get all grid cell that the line between keyframe and map point pass through
                int x0 = okf_pos_grid_x;
                int y0 = okf_pos_grid_y;
                int x1 = mp_pos_grid_x;
                int y1 = mp_pos_grid_y;
                bool steep = (abs(y1 - y0) > abs(x1 - x0));
                if (steep)
                {
                    x0 = okf_pos_grid_y;
                    y0 = okf_pos_grid_x;
                    x1 = mp_pos_grid_y;
                    y1 = mp_pos_grid_x;
                }
                if (x0 > x1)
                {
                    x0 = mp_pos_grid_y;
                    x1 = okf_pos_grid_y;
                    y0 = mp_pos_grid_x;
                    y1 = okf_pos_grid_x;
                }
                int deltax = x1 - x0;
                int deltay = abs(y1 - y0);
                double error = 0;
                double deltaerr = ((double) deltay) / ((double) deltax);
                int y = y0;
                int ystep = (y0 < y1) ? 1 : -1;
                for (int x = x0; x <= x1; x++)
                {
                    if (steep)
                    {
                        grid_visit[y][x]++;
                    } else
                    {
                        grid_visit[x][y]++;
                    }
                    error = error + deltaerr;
                    if (error >= 0.5)
                    {
                        y = y + ystep;
                        error = error - 1.0;
                    }
                }
            }
        }

        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                if (grid_visit[j][i] == 0)
                {
                    f << 230 << " ";
                    continue;
                }
                double r = grid_occup[j][i] / grid_visit[j][i];
                int grey = (int) (r * 255);
                if (grey > 0)
                    grey += 100;
                if (grey > 255)
                    grey = 255;
                f << 255 - grey << " ";

            }
            f << endl;
        }
        f.close();
        cout << endl << "Grid map saved!" << endl;
    }

    void MapProcessor::OpenMap(const string &path)
    {
        cv::Mat gmap = cv::imread(path);
        while(true)
        {
           cv::imshow("Grid Map", gmap);
           if (cv::waitKey(0) == 27)
               break;
        }
        cv::destroyAllWindows();
    }
}
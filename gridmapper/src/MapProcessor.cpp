#include "MapProcessor.h"

namespace ORB_SLAM2
{
    MapProcessor::MapProcessor(const std::string &filename): map(), keyFrameDatabase(), vocabulary(new ORBVocabulary())
    {
        // Load ORB Vocabulary
        vocFile = "../../Vocabulary/ORBvoc.bin";
        vocabulary->loadFromBinaryFile(vocFile);
        cout << "Vocabulary loaded!" << endl << endl;

        mapfile = filename;
        std::ifstream in(filename, std::ios_base::binary);
        if (!in)
        {
            cerr << "Cannot Open Mapfile: " << mapfile << " , You need create it first!" << std::endl;
        }
        cout << "Loading Mapfile: " << mapfile << endl;
        boost::archive::binary_iarchive ia(in, boost::archive::no_header);

        // Retrieve Map and KF Database from stored map
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

    void MapProcessor::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

        KFs = map->GetAllKeyFrames();
        sort(KFs.begin(), KFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = KFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        for(auto KF : KFs)
        {
            cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

            while(KF->isBad())
            {
                //  cout << "bad parent" << endl;
                Trw = Trw*KF->mTcp;
                KF = KF->GetParent();
            }

            // Trw = Trw*KF->GetPose()*Two;

            cv::Mat Tcw = KF->GetPose();
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            f << setprecision(6) << KF->mTimeStamp << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

           /*
            f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
              Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
              Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
           */
        }
        f.close();
        cout << endl << "trajectory saved!" << endl;
    }

    void MapProcessor::SavePointCloud(const string &filename)
    {
        cout << "Saving map points along with keyframe pose to " << filename << endl;
        ofstream fout(filename.c_str(), ios::out);
        vector<MapPoint *> MPs = map->GetAllMapPoints();
        cout << "  writing " << MPs.size() << " map points" << endl;

        fout << fixed;
        for (auto mp: MPs)
        {
            cv::Mat wp = mp->GetWorldPos();
            fout << wp.at<float>(0) << " "; // pos x: float
            fout << wp.at<float>(1) << " "; // pos y: float
            fout << wp.at<float>(2) << " "; // pos z: float

            std::map<KeyFrame *, size_t> observations = mp->GetObservations();
            for (auto obs: observations)
                fout << setprecision(6) << " " << obs.first->mTimeStamp;

            fout << endl;
        }
        fout.close();
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

    void MapProcessor::OpenMapPangolin()
    {
        std::string settings_path = "Examples/Stereo/KITTI03.yaml";

        MapDrawer mapDrawer(map, settings_path);

        cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
                pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, 1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        while(!pangolin::ShouldQuit())
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f,1.0f,1.0f,1.0f);

            d_cam.Activate(s_cam);
            mapDrawer.DrawKeyFrames(true, true);
            mapDrawer.DrawMapPoints();

            pangolin::FinishFrame();
        }
    }

    void MapProcessor::ConvertMPsToPCL()
    {
        const vector<MapPoint*> &MPs = map->GetAllMapPoints();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = map->GetAllMapPoints().size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.points.resize (cloud.width * cloud.height);

        for (uint32_t i = 0; i < cloud.width; i++)
        {
            cloud[i].x = MPs[i]->GetWorldPos().at<float>(0);
            cloud[i].y = MPs[i]->GetWorldPos().at<float>(1);
            cloud[i].z = MPs[i]->GetWorldPos().at<float>(2);
        }

        pcl::io::savePCDFileASCII ("../point_clouds/test_pcd.pcd", cloud);
        std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;
    }

    void MapProcessor::RemoveOutliers(const string& pcl_filename, const string& pcl_outfn)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

         // Fill in the cloud data
        pcl::PCDReader reader;
        // Replace the path below with the path where you saved your file
        reader.read<pcl::PointXYZ> (pcl_filename, *cloud);

        std::cerr << "Cloud before filtering: " << std::endl;
        std::cerr << *cloud << std::endl;

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered);

        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << *cloud_filtered << std::endl;

        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ> (pcl_outfn, *cloud_filtered, false);

        // sor.setNegative (true);
        // sor.filter (*cloud_filtered);
        // writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
    }

    void MapProcessor::ViewPC(const string& filename)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        // Fill in the cloud data
        pcl::PCDReader reader;
        // Replace the path below with the path where you saved your file
        reader.read<pcl::PointXYZ> (filename, *cloud);

        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        viewer.showCloud (cloud);

        while (!viewer.wasStopped())
        {
            // Stuff
        }
    }

}
#include "arducam_mipicamera.h"
#include "System.h"

#include <opencv2/core/core.hpp>  

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#define VCOS_ALIGN_DOWN(p,n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p,n) VCOS_ALIGN_DOWN((ptrdiff_t)(p)+(n)-1,(n))

#define LOG(fmt, args...) fprintf(stderr, fmt "\n", ##args)

// template <class T>
// class circular_buffer {
// public:
// 	explicit circular_buffer(size_t size) :
// 		buf_(std::unique_ptr<T[]>(new T[size])),
// 		max_size_(size)
// 
// 	void put(T item);
// 	T get();
// 	void reset();
// 	bool empty() const;
// 	bool full() const;
// 	size_t capacity() const;
// 	size_t size() const;
// 
// private:
// 	std::mutex mutex_;
// 	std::unique_ptr<T[]> buf_;
// 	size_t head_ = 0;
// 	size_t tail_ = 0;
// 	const size_t max_size_;
// 	bool full_ = 0;
// };

cv::Mat *get_image(CAMERA_INSTANCE camera_instance, int width, int height) 
{
    IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 100};
    BUFFER *buffer = arducam_capture(camera_instance, &fmt, 3000);
    if (!buffer) 
        return NULL;
    
    // The actual width and height of the IMAGE_ENCODING_RAW_BAYER format and the IMAGE_ENCODING_I420 format are aligned, 
    // width 32 bytes aligned, and height 16 byte aligned.
    width = VCOS_ALIGN_UP(width, 32);
    height = VCOS_ALIGN_UP(height, 16);
    cv::Mat *image = new cv::Mat(cv::Size(width,(int)(height * 1.5)), CV_8UC1, buffer->data);
    cv::cvtColor(*image, *image, cv::COLOR_YUV2BGR_I420);
    arducam_release_buffer(buffer);
    return image;
}


int main(int argc, char **argv) 
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./arducam_stereo path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO, true, false);

    CAMERA_INSTANCE camera_instance;

    //LOG("Open Camera...");
    int res = arducam_init_camera(&camera_instance);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }

    int width = 640*2;
    int height = 480;

    //res = arducam_set_mode(camera_instance, 10); 
//    struct format fmt;
//    fmt.mode = 10; 
//    std::cout << fmt.mode << endl;
    
    res = arducam_set_resolution(camera_instance, &width, &height);
    if (res) {
         LOG("set resolution status = %d", res);
         return -1;
    } else {
         LOG("Current resolution is %dx%d", width, height);
         LOG("Notice:You can use the list_format sample program to see the resolution and control supported by the camera.");
    }
    
    cv::Mat imLeftRect;
    cv::Mat imRightRect;

    double tframe = 0;
    while(true)
    {
        cv::Mat* image = get_image(camera_instance, width, height);
        if(!image)
            continue;
	    cv::Mat img = *image;	
	    cv::Mat imLeft = img(cv::Rect(0, 0, (width/2), height));
	    cv::Mat imRight = img(cv::Rect((width/2), 0, (width/2), height)); 

        // cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        // cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        SLAM.TrackStereo(imLeft, imRight, tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double tracking_time = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        std::cout << tracking_time << endl;

        tframe = tframe + tracking_time;
        std::cout << tframe << endl;

	    delete image;
    
        if(cv::waitKey(1) == 27)
        {
            arducam_close_camera(camera_instance);
            break;
        }
    }

    SLAM.Shutdown();

    return 0;
}

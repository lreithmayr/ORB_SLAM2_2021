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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true, true);

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
    

    while(true)
    {
        cv::Mat* image = get_image(camera_instance, width, height);
        if(!image)
            continue;
	    cv::Mat img = *image;	
	    cv::Mat imLeft = img(cv::Rect(0, 0, (width/2), height));
	    cv::Mat imRight = img(cv::Rect((width/2), 0, (width/2), height)); 

        // FIXME: Get actual time between frames.
        double tframe = (double) 21.00053;

        SLAM.TrackStereo(imLeft, imRight, tframe);

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

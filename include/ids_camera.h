#ifndef __IDS_CAMERA_H__
#define __IDS_CAMERA_H__

// Standard Lib
#include <wchar.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <sstream>
#include <fstream> 
// OpenCV Lib
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
// IDS XS Lib
#include <uEye.h> 
#include <ueye_deprecated.h>

#define CAM_VIDEO_WIDTH 	1280 
#define CAM_VIDEO_HEIGHT 	720
#define CAM_VIDEO_BPP 		24

using namespace std;
using namespace cv;

namespace ids {

	class ids_camera{

	public:
		// Classic constructor
		ids_camera(const HIDS, const int, const int width = CAM_VIDEO_WIDTH, const int height = CAM_VIDEO_HEIGHT, const int bpp = CAM_VIDEO_BPP, const std::string rec = "", const bool offline = false, const std::string off_vid = "", const bool verbose = false);

		~ids_camera();
		
		// Start camera stream		
		bool cameraOn();

		// Recording video
		bool recON(std::string);
		bool recOFF();

		// Get current frame
		Mat get_frame();

	private:

		// Init function
		bool init();

		// Check if the video already exist
		bool is_file_exist(const char *);

		// terminate on error
		void terminate_on_error();

		// Variables

		bool verbose_;

		HIDS hCam_;
		int fps_;
		int width_, height_, bpp_;
		std::string rec_;

		cv::VideoWriter outVid_;
		cv::VideoCapture* off_video_;

		bool is_terminate_;
		bool recON_;
		bool offline_;


	};
} 
#endif 
#include "ids_camera.h" 

using namespace ids;

/// ----------------------------------------------------> Time manager <----------------------------------------------------

// double diff_ms(const timeval t1, const timeval t2)
// { 

// 	return ((t1.tv_usec + (t1.tv_sec - t2.tv_sec) * 1000000) - t2.tv_usec) / 1000;
// }

/// -------------------------------------------------------------------------------------------------------------------------------------

ids_camera::ids_camera(const HIDS id, const int fps, const int width, const int height, const int bpp, const std::string rec, const bool offline, const std::string off_vid, const bool verbose){


	// Initialize vars
	is_terminate_ = false;
	recON_ = false;

	// Get Params

	hCam_ = id;
	verbose_ = verbose;
	fps_ = fps;
	width_ = width;
	height_ = height;
	bpp_ = bpp;
	rec_ = rec; 
	offline_ = offline;

	if (offline_){

		off_video_ = new VideoCapture(off_vid.c_str()); 

		if(!off_video_->isOpened()){  // check if we succeeded
			std::cout << "Can not open the video" << std::endl;;
			return;
		}

		// Get Params 
		Mat fp;
		fp = get_frame();
		
		width_ = fp.cols;
		height_ = fp.rows;

	}else{

		if (!init()){
		    terminate_on_error();
		    return;
		}

		cameraOn();
		
		if (!rec.empty())
			recON(rec);
		
	}	
}


ids_camera::~ids_camera(){

	if (offline_)
		delete off_video_;
	else{

		if (verbose_)
			cout << "Closing IDS camera" << endl;

		is_ExitCamera(hCam_);
	}
}

bool ids_camera::init(){
	
	const double enable = 1;
	const double disable = 0;

	// Init camera

	if (verbose_)
		cout << "- Init camera --> ";

	if(is_InitCamera (&hCam_, NULL) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}

	// Memory allocation and display
	if (verbose_)
		cout << "- Alloc Image Mem --> ";

	char* imgMem;
	int memId;

	if (is_AllocImageMem(hCam_, width_, height_, bpp_, &imgMem, &memId) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}

	if (verbose_)
		cout << "- Set Image Mem --> ";

	if (is_SetImageMem (hCam_, imgMem, memId) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}

	if (verbose_)
		cout << "- Set Display Mode --> ";

	if (is_SetDisplayMode (hCam_, IS_SET_DM_DIB) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}

	if (verbose_)
		cout << "- Set Color Mode --> ";

	if (is_SetColorMode (hCam_, IS_SET_CM_RGB24) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	if (verbose_)
		cout << "- Set Image Size --> ";

	if (is_SetImageSize (hCam_, width_, height_) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	// Params

	// cout << "- Auto Gain --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_GAIN, &enable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto WhiteBalance --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_WHITEBALANCE, &enable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto FrameRate --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_FRAMERATE, &disable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto Shutter --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_SHUTTER, &disable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto Sensor Gain --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &enable, 0)!= IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto Sensor WhiteBalance --> ";

	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE,&enable,0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	// cout << "- Auto Sensor Shutter --> ";
	
	// if (is_SetAutoParameter (hCam_, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &disable, 0) != IS_SUCCESS)
	//     return false;
	// else
	//     cout << "OK" << endl;
	
	double NEWFPS;

	if (verbose_)
		cout << "- Set FrameRate --> ";

	// Set FrameRate
	if (is_SetFrameRate(hCam_, fps_, &NEWFPS) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	double parameter = 50;
	
	if (verbose_)
		cout << "- Set Exposure --> ";

	// Exposure set
	if (is_Exposure(hCam_, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*) &parameter, sizeof(parameter)) != IS_SUCCESS) 
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	UINT uiCaps = 0;

	if (verbose_)
		cout << "- Enable AutoFocus --> ";

	// Focus camera
	if (is_Focus (hCam_, FOC_CMD_GET_CAPABILITIES, &uiCaps, sizeof (uiCaps) ) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	if (verbose_)
		cout << "- Check if AutoFocus is Enable --> ";

	if (is_Focus (hCam_, FOC_CMD_GET_AUTOFOCUS_ENABLE, &uiCaps, sizeof (uiCaps) ) != IS_SUCCESS)
		return false;

	// Check on supporting of focus option

	if (uiCaps != 1){
		if (verbose_)
			cout << "NO" << endl;
	}
	else{
		if (verbose_)
			cout << "YES" << endl;
	}

	// Get pixel clock range
	UINT nRange[3];
	UINT nMin = 0, nMax = 0, nInc = 0;

	ZeroMemory(nRange, sizeof(nRange));

	if (verbose_)
		cout << "- Pixelclock get range --> ";

	if (is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange)) == IS_SUCCESS)
	{

		if (verbose_)
			cout << "OK" << endl;
	
		nMin = nRange[0];
		nMax = nRange[1];
		nInc = nRange[2];
	
		if (verbose_){
			cout << "\tMin: " << nMin << endl;
			cout << "\tMax: " << nMax << endl;
			cout << "\tInc: " << nInc << endl;
		}
	}
	else
		return false;
	
	if (verbose_)
		cout << "- Set Pixelclock --> ";

	if(is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_SET, (void*)&nMin, sizeof(nMin)) != IS_SUCCESS)
		return false;
	else{
		if (verbose_)
			cout << "OK" << endl;
	}
	
	return true;
}

// Terminate on camera error 
void ids_camera::terminate_on_error(){

	INT pErr;
	IS_CHAR* ppcErr;

	is_GetError(hCam_, &pErr, &ppcErr);

	if (verbose_){
		cout << "Error" << endl;
		cout << "Code: " << pErr << endl;
		cout << "Text: " << ppcErr << endl;
	}

	is_terminate_ = true;

	// Close camera
	is_ExitCamera(hCam_);

}

// Get Frame from ids camera
Mat ids_camera::get_frame(){

	Mat frame(height_, width_, CV_8UC3);

	if (offline_){
		VideoCapture capture = *off_video_ ;
		capture >> frame;
	}else{

		//pointer to where the image is stored
		uchar* pMemVoid; 

		// Check if camera is alive

		if (is_terminate_)
			return frame;
		
		// Pointer on matrix
		uchar* x = frame.ptr();

		// Get Image from camera
		if ( is_GetImageMem (hCam_, (void**) &pMemVoid) != IS_SUCCESS){
			if (verbose_)
				cout << "- Get an Image mem --> ERROR" << endl;
		}

		// Copy img in Mat structure

		for (int i = width_ * height_ * 3; i--; )
			x[i] = (uchar) pMemVoid[i];

		if (recON_)
			outVid_ << frame;

		return frame;
	}
}

bool ids_camera::cameraOn(){

	// Check if camera is alive

	if (is_terminate_)
		return false;

	// Enable camera capture routine
	if (verbose_)
		cout << "- CaptureVideo Enabled --> ";
	
	if (is_CaptureVideo (hCam_, IS_WAIT) != IS_SUCCESS){
	    terminate_on_error();
	    return false;
	}else{
		if (verbose_)
			cout << "OK" << endl;
	}	

	return true;
}

bool ids_camera::recON(std::string str){

	if (offline_){
		if (verbose_)
			cout << "- Unable to set REC Mode, the offline option is ON " << endl;
		return false;
	}

	if (recON_){
		if (verbose_)
			cout << "- REC is already On" << endl;

		return true;
	}

	// Check if the file already exist

	int count = 0;

	while (is_file_exist(str.c_str())){

		string str_c;
		stringstream strstream;
		strstream << count;
		strstream >> str_c;

		if (count == 0)
			str.erase(str.end() - 4, str.end());
		else
			str.erase(str.end() - str_c.size()  - 1, str.end());

		str += "_" + str_c + ".avi";

		count++;
	}
	
	int ex = static_cast<int>(CV_FOURCC('M', 'P', '4', 'V')); 
	
	// Transform from int to char via Bitwise operators
	char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};
	
	// Open video

	outVid_.open(str.c_str(), ex, 15.0, Size( width_, height_ ), true);

	if (outVid_.isOpened()) {
		recON_ = true;
		
		return true;
	}

	cout << "false" << endl;
	return false;

}

bool ids_camera::recOFF(){

	recON_ = false;

}


bool ids_camera::is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}




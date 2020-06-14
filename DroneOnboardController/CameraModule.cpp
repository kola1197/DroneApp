//
// Created by nickolay on 09.06.2020.
//

#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <sys/stat.h>
#include "CameraModule.h"
#include "stdio.h"

#ifdef __arm__
e_vbus_index CameraModule::sensor_id = e_vbus4;
cv::Mat CameraModule::g_greyscale_image_left = cv::Mat::zeros(240,320,CV_8UC1);
cv::Mat	CameraModule::g_greyscale_image_right = cv::Mat::zeros(240,320,CV_8UC1);
DJI_lock  CameraModule::g_lock;
DJI_event CameraModule::g_event;
#endif

int CameraModule::startThread() {
    int result = 0;
    std::cout<<"starting thread"<<std::endl;
    if (testMode == 1) {                                                                   //TODO: add check of video0/video2 exist
        std::thread thr([this]() {
            cv::VideoCapture leftCamera("/dev/video2");
            cv::VideoCapture rightCamera("/dev/video0");
            //cv::VideoCapture rightCamera(0);
            cv::Mat frame;
            cv::Mat out;
            int64 lastSave = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            //std::chrono::milliseconds lastSave = duration_cast< std::chrono::milliseconds >(
            //        std::chrono::system_clock::now().time_since_epoch()
            //);            //std::cout<<threadStop.get()<<" thread stop value"<<std::endl;
            while (!threadStop.get()) {
                leftCamera >> frame;
                cv::cvtColor(frame, out, CV_BGR2RGB);
                resize(out, out, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
                //cv::imwrite("BEFORE.jpg",out);
                leftImage.setImage(out.size(), out.data);
                //cv::imwrite("AFTER.jpg",*leftImage.getImage());

                rightCamera >> frame;
                cv::cvtColor(frame, out, CV_BGR2RGB);
                resize(out, out, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
                rightImage.setImage(out.size(), out.data);
                int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                if (imageCaptureMode.get() && (now - lastSave) > 20 )
                {
                    lastSave = now;
                    saveImages();
                }
            }
        });
        thr.detach();
    }
    #ifdef __arm__
    if (testMode == 0)
    {
    int h = 320;
    int w = 240;
    int *height;
    int *width ;
    height = &h;
    width = &w;
    reset_config();  // clear all data subscription

	int err_code = init_transfer(); //wait for board ready and init transfer thread
	//RETURN_IF_ERR( err_code );

	int online_status[CAMERA_PAIR_NUM];
	err_code = get_online_status(online_status);
	//RETURN_IF_ERR(err_code);
	// get cali param
	stereo_cali cali[CAMERA_PAIR_NUM];
	err_code = get_stereo_cali(cali);
	//RETURN_IF_ERR(err_code);
    
    #if !USE_GUIDANCE_ASSISTANT_CONFIG
        err_code = select_greyscale_image( sensor_id, true );
        //RETURN_IF_ERR( err_code );
        err_code = select_greyscale_image( sensor_id, false );
        //RETURN_IF_ERR( err_code );
    //#if SELECT_DEPTH_DATA
    //    err_code = select_depth_image( sensor_id );
    //    //RETURN_IF_ERR( err_code );
    //    err_code = select_disparity_image( sensor_id );
    //    //RETURN_IF_ERR( err_code );
    //#endif
        select_imu();
        select_ultrasonic();
        select_obstacle_distance();
        select_velocity();
        select_motion();
    #endif
    e_device_type dt;
	get_device_type(&dt);
	//cout<<"device type: "<<(dt==Guidance?"Guidance":"GuidanceLite")<<endl;

	get_image_size(width, height);
	//cout<<"(width, height)="<<WIDTH<<", "<<HEIGHT<<endl;

	err_code = set_sdk_event_handler( my_callback );
	//RETURN_IF_ERR( err_code );
	err_code = start_transfer();
	//RETURN_IF_ERR( err_code );

	// for setting exposure
	exposure_param para;
	para.m_is_auto_exposure = 1;
	para.m_step = 10;
	para.m_expected_brightness = 120;
	para.m_camera_pair_index = sensor_id;

        std::cout<<"Creating thread"<<std::endl;
        std::thread thr([this]
        {
            int h = 320;
            int w = 240;
            int *height;
            int *width ;
            height = &h;
            width = &w;
            cv::Mat frame;
            cv::Mat out;
            int64 lastSave = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            e_device_type dt;
            get_device_type(&dt);
            //cout<<"device type: "<<(dt==Guidance?"Guidance":"GuidanceLite")<<endl;

            get_image_size(width, height);
            //cout<<"(width, height)="<<WIDTH<<", "<<HEIGHT<<endl;

            set_sdk_event_handler( my_callback );
            //RETURN_IF_ERR( err_code );
            start_transfer();
            //RETURN_IF_ERR( err_code );
            std::cout<<"Thread created"<<std::endl;

            // for setting exposure
            exposure_param para;
            para.m_is_auto_exposure = 1;
            para.m_step = 10;
            para.m_expected_brightness = 120;
            para.m_camera_pair_index = CameraModule::sensor_id;
            while (!threadStop.get()) {
                //std::cout<<"Starting the loop"<<std::endl;
                g_event.wait_event();
                //std::cout<<"Got event"<<std::endl;
                /*if(!g_greyscale_image_left.empty())
                    imshow(string("left_")+char('0'+sensor_id), g_greyscale_image_left);
                if(!g_greyscale_image_right.empty())
                    imshow(string("right_")+char('0'+sensor_id), g_greyscale_image_right);
                if(!g_depth.empty()){
                    Mat depth8(HEIGHT,WIDTH,CV_8UC1);
                    g_depth.convertTo(depth8, CV_8UC1);
                    imshow(string("depth_")+char('0'+sensor_id), depth8);
                    printf("Depth at point (%d,%d) is %f meters!\n", HEIGHT/2, WIDTH/2,  float(g_depth.at<short>( HEIGHT/2,WIDTH/2))/128);
                }
                if(!g_disparity.empty()){
                    Mat disp8(HEIGHT,WIDTH, CV_8UC1);
                    g_disparity.convertTo(disp8, CV_8UC1);
                    imshow(string("disparity_")+char('0'+sensor_id), disp8);
                    printf("Disparity at point (%d,%d) is %f pixels!\n", HEIGHT/2, WIDTH/2,  float(g_disparity.at<short>( HEIGHT/2,WIDTH/2))/16);
                }*/
                
                //cv::cvtColor(g_greyscale_image_left, out, CV_BGR2RGB);
                //cv::cvtColor(g_greyscale_image_left, out, CV_GRAY2RGB);
                g_greyscale_image_left.copyTo(out);
                leftImage.setGrayImage(out.size(), out.data);
                
                //cv::cvtColor(g_greyscale_image_right, out, CV_BGR2RGB);
                //cv::cvtColor(g_greyscale_image_right, out, CV_GRAY2RGB);
                g_greyscale_image_right.copyTo(out);
                //std::cout<<"cvtColor right"<<std::endl;
                rightImage.setGrayImage(out.size(), out.data);
                //std::cout<<"setImage right"<<std::endl;
                int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                if (imageCaptureMode.get() && (now - lastSave) > 20 )
                {
                    lastSave = now;
                    saveImages();
                }
            }
        });
        thr.detach();

    }
    #else
    if (testMode == 0)
    {
        //testMode = 1;
        result = 1;
    }
    #endif
    return result;                                  // 0 - ok, 1 - this testMode can not be set
}

#ifdef __arm__
int CameraModule::my_callback(int data_type, int data_len, char *content)
{
	g_lock.enter();
	if (e_image == data_type && NULL != content)
	{
		image_data* data = (image_data* )content;
		if ( data->m_greyscale_image_left[sensor_id] ){
			g_greyscale_image_left = cv::Mat::zeros(240,320,CV_8UC1);
			memcpy( g_greyscale_image_left.data, data->m_greyscale_image_left[sensor_id], 76800 );
		}else g_greyscale_image_left.release();
		if ( data->m_greyscale_image_right[sensor_id] ){
			g_greyscale_image_right = cv::Mat::zeros(240,320,CV_8UC1);
			memcpy( g_greyscale_image_right.data, data->m_greyscale_image_right[sensor_id], 76800 );
		}else g_greyscale_image_right.release();
		/*if ( data->m_depth_image[sensor_id] ){
			g_depth = cv::Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
			memcpy( g_depth.data, data->m_depth_image[sensor_id], IMAGE_SIZE * 2 );
		}else g_depth.release();
		if ( data->m_disparity_image[sensor_id] ){
			g_disparity = cv::Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
			memcpy( g_disparity.data, data->m_disparity_image[sensor_id], IMAGE_SIZE * 2 );
		}else g_disparity.release();*/
	}
	g_lock.leave();
	g_event.set_event();

	return 0;
}

#endif

int CameraModule::getTestMode(){
    return testMode;
}

int CameraModule::setTestMode(int i)
{
    threadStop.set(true);
    testMode = i;
    return startThread();
}

CameraModule::CameraModule()
{
    //getDirectoryToSave();

}

CameraModule::~CameraModule()
{
    stopThread();
}

void CameraModule::setImageCaptureMode(bool mode)
{
    if (mode) {
        if (!imageCaptureMode.get())
        {
            getDirectoryToSave();
            saveCounter = 0;
            std::string path0 = dirToSave + "/0";
            std::string path1 = dirToSave + "/1";
            path0.erase(std::remove(path0.begin(), path0.end(), ' '), path0.end());
            path1.erase(std::remove(path1.begin(), path1.end(), ' '), path1.end());
            path0.erase(std::remove(path0.begin(), path0.end(), '\n'), path0.end());
            path1.erase(std::remove(path1.begin(), path1.end(), '\n'), path1.end());
            std::string com0 = "mkdir -p " + path0;
            std::string com1 = "mkdir -p " + path1;
            system(com0.data());
            system(com1.data());
        }
        imageCaptureMode.set(mode);
    }
    else{
        imageCaptureMode.set(mode);
    }
}

void CameraModule::saveImages()
{

    std::string path0 = dirToSave + "/0";
    std::string path1 = dirToSave + "/1";
    path0.erase(std::remove(path0.begin(),path0.end(),' '),path0.end());
    path1.erase(std::remove(path1.begin(),path1.end(),' '),path1.end());
    path0.erase(std::remove(path0.begin(),path0.end(),'\n'),path0.end());
    path1.erase(std::remove(path1.begin(),path1.end(),'\n'),path1.end());
    path0 += "/" + std::to_string(saveCounter)+".jpg";
    path1 += "/" + std::to_string(saveCounter)+".jpg";
    cv::imwrite(path0,*leftImage.getImage());
    cv::imwrite(path1,*rightImage.getImage());
    std::cout<<path0<<std::endl;
    saveCounter++;
}

void CameraModule::getDirectoryToSave()
{
    auto end = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    dirToSave = std::ctime(&end_time);
}

void CameraModule::stopThread()
{
    threadStop.set(true);
}

bool CameraModule::active()
{
    bool result = !threadStop.get();
    return result;
}
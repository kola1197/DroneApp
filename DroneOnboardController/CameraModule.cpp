//
// Created by nickolay on 09.06.2020.
//
#include <filesystem>
#include <iostream>
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
#include "opencv2/xfeatures2d.hpp"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>


//#ifdef __arm__
//e_vbus_index CameraModule::sensor_id = e_vbus4;
//cv::Mat CameraModule::g_greyscale_image_left = cv::Mat::zeros(240,320,CV_8UC1);
//cv::Mat	CameraModule::g_greyscale_image_right = cv::Mat::zeros(240,320,CV_8UC1);
//DJI_lock  CameraModule::g_lock;
//DJI_event CameraModule::g_event;
//#endif

#include <unistd.h>
#include <opencv/cv.hpp>

inline bool exists (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

int CameraModule::startThread() {
    int result = 0;
    std::cout << "starting thread" << std::endl;
    if (captureMode.get() == CaptureMode::TEST_DATASET) {
        std::thread thr([this]() {
            cv::Mat frame;
            cv::Mat out;
            std::string leftPart = "/home/nickolay/Odometry dataset/dataset/sequences/00/image_0/";//"/home/nickolay/Odometry Dataset/image_0/";//"/home/nickolay/Documents/DatasetsOdometry/currentTestData/0/";
            std::string rightPart = "/home/nickolay/Odometry dataset/dataset/sequences/00/image_0/";//"/home/nickolay/Odometry Dataset/image_0/";//"/home/nickolay/Documents/DatasetsOdometry/currentTestData/1/";
            //std::string leftPart = "/home/nickolay/test.jpg";
            //std::string rightPart = "/home/nickolay/test.jpg";

            int counter = 0;
            while ((!threadStop.get()) && counter < 1600) {
                //TODO: check file exists
                if (!imageForOdometryModuleUpdated.get()) {
                    std::string s = leftPart.c_str();
                    s.append("000000");
                    s.erase(s.begin() + s.size() - std::to_string(counter).size(), s.begin() + s.size());
                    s.append(std::to_string(counter).c_str());
                    s.append(".png");
                    //std::cout << "left path: " << s << std::endl;
                    /*if (exists(leftPart))
                    {
                        std::cout<<"exists"<<std::endl;
                    } else{
                        std::cout<<"NOT EXISTS"<<std::endl;
                    }*/
                    out = cv::imread(s);
                    if (!out.data) {
                        std::cout << "Image read error on test 2!!!" << std::endl;
                    }
                    //cv::imshow("testL", out);
                    leftPrevImage.setImage(leftImage.getImage()->size(), leftImage.getImage()->data);
                    leftImage.setImage(out.size(), out.data);
                    int type = out.type();
                    //std::cout<<"type: "<<type<<std::endl;
                    //cv::imshow("test", out);
                    //cv::waitKey(0);
                    s = rightPart.c_str();
                    s.append("000000");
                    s.erase(s.begin() + s.size() - std::to_string(counter).size(), s.begin() + s.size());
                    s.append(std::to_string(counter).c_str());
                    s.append(".png");
                    //std::cout<<"right path: "<<s<<std::endl;
                    out = cv::imread(s);
                    //cv::cvtColor(frame, out, CV_GRAY2RGB);
                    rightPrevImage.setImage(rightImage.getImage()->size(), rightImage.getImage()->data);
                    rightImage.setImage(out.size(), out.data);
                    gotImage.set(true);
                    usleep(33000);
                    frameNum.set(counter);
                    imageForOdometryModuleUpdated.set(true);
                    counter++;
                }
            }
            std::cout << "Thread done" << std::endl;
        });
        thr.detach();
    }
    if (captureMode.get() == CaptureMode::REALSENSE) {                                                                   //TODO: add check of video0/video2 exist
        std::thread thr([this]() {
            int counter = 0;
            std::cout<<"REALSENSE"<<std::endl;
            namedWindow("color test", cv::WINDOW_AUTOSIZE);
            namedWindow("depth test", cv::WINDOW_AUTOSIZE);
            //cv::VideoCapture leftCamera("/dev/video0");
            //cv::VideoCapture rightCamera("/dev/video2");
            //cv::VideoCapture rightCamera(0);
            rs2::config cfg;
            float ResultVector[3];
            float InputPixelAsFloat[2];
            InputPixelAsFloat[0] = 416;
            InputPixelAsFloat[1] = 316;
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16 , 30);
            cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);

            rs2::colorizer color_map;
            rs2::pipeline pipe;
            auto MyPipelineProfile = pipe.start(cfg);
            auto DepthStream = MyPipelineProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            rs2_intrinsics lDepthIntrinsics = DepthStream.get_intrinsics();
            for (int i = 0; i<10; i++)//to skip first few frames when device just initiated
                auto frames = pipe.wait_for_frames();
            while (!threadStop.get()){
                rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                rs2::frame color = data.get_color_frame();
                rs2::depth_frame localDepthFrame = data.get_depth_frame();
                const int w = color.as<rs2::video_frame>().get_width();
                const int h = color.as<rs2::video_frame>().get_height();
                float distance = localDepthFrame.get_distance(320, 240);

                rs2_deproject_pixel_to_point(ResultVector, &lDepthIntrinsics, InputPixelAsFloat, distance);
                std::cout <<"DEPROJECTED POINT: "<< "x = " << ResultVector[0] << ", y = " << ResultVector[1] << ", z = " << ResultVector[2] << std::endl;
                //std::cout<<localDepthFrame.get_data()<<std::endl;
                rs2::frame depth = localDepthFrame.apply_filter(color_map);

                cv::Mat colorImageA(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat colorImage;
                cv::cvtColor(colorImageA, colorImage, cv::COLOR_BGR2RGB);

                leftPrevImage.setImage(leftImage.getImage()->size(), leftImage.getImage()->data);
                leftImage.setImage(colorImage.size(), colorImage.data);

                const int ww = depth.as<rs2::video_frame>().get_width();
                const int hh = depth.as<rs2::video_frame>().get_height();
                cv::Mat depthImage (cv::Size(ww, hh), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

                //circle(depthImage, cv::Point(320, 240), 1, CV_RGB(0, 255, 0), 2);

                //imshow("color test", colorImage);
                //imshow("depth test", depthImage);
                //cv::waitKey(1);

                rightPrevImage.setImage(rightImage.getImage()->size(), rightImage.getImage()->data);
                rightImage.setImage(depthImage.size(), depthImage.data);
                DepthIntrinsics.set(lDepthIntrinsics);
                //depthFrame.set(localDepthFrame);
                depthImageMutex.lock();
                depthFrame = localDepthFrame;
                depthImageMutex.unlock();
                if (imageCaptureMode.get()){
                    char buffer[PATH_MAX];
                    getcwd(buffer, sizeof(buffer));
                    std::string protoPath("./" + dirToSave+"/0/"+std::to_string(counter)+".jpg");
                    std::string path("");
                    for (int i=0;i<protoPath.size();i++)
                    {
                        if (protoPath[i] != ' ' && protoPath[i]!='\n')
                        path.push_back(protoPath[i]);
                    }
                    ///home/nickolay/Code/DroneApp/cmake-build-debug/DroneOnboardController/SatFeb2718:57:352021/0
                    std::cout<<path<<std::endl;
                    cv::imwrite(path,colorImageA);
                    //cv::imwrite(std::string(buffer) + dirToSave+"/0/"+std::to_string(counter)+".bmp",colorImageA);
                }
                gotImage.set(true);
                usleep(33000);
                frameNum.set(counter);
                imageForOdometryModuleUpdated.set(true);
                counter++;
            }
        });
        thr.detach();
    }
    return result;                                  // 0 - ok, 1 - this testMode can not be set
}

//#ifdef __arm__
//int CameraModule::my_callback(int data_type, int data_len, char *content)
//{
//    g_lock.enter();
//    if (e_image == data_type && NULL != content)
//    {
//        image_data* data = (image_data* )content;
//        if ( data->m_greyscale_image_left[sensor_id] ){
//            g_greyscale_image_left = cv::Mat::zeros(240,320,CV_8UC1);
//            memcpy( g_greyscale_image_left.data, data->m_greyscale_image_left[sensor_id], 76800 );
//        }else g_greyscale_image_left.release();
//        if ( data->m_greyscale_image_right[sensor_id] ){
//            g_greyscale_image_right = cv::Mat::zeros(240,320,CV_8UC1);
//            memcpy( g_greyscale_image_right.data, data->m_greyscale_image_right[sensor_id], 76800 );
//        }else g_greyscale_image_right.release();
//        /*if ( data->m_depth_image[sensor_id] ){
//            g_depth = cv::Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
//            memcpy( g_depth.data, data->m_depth_image[sensor_id], IMAGE_SIZE * 2 );
//        }else g_depth.release();
//        if ( data->m_disparity_image[sensor_id] ){
//            g_disparity = cv::Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
//            memcpy( g_disparity.data, data->m_disparity_image[sensor_id], IMAGE_SIZE * 2 );
//        }else g_disparity.release();*/
//    }
//    g_lock.leave();
//    g_event.set_event();
//
//    return 0;
//}
//
//#endif

int CameraModule::getCaptureMode() {
    return captureMode.get();
}

int CameraModule::setCaptureMode(CaptureMode i) {
    threadStop.set(true);
    captureMode.set( i);
    return startThread();
}

CameraModule::CameraModule() {
    //getDirectoryToSave();

}

CameraModule::~CameraModule() {
    stopThread();
}

void CameraModule::setImageCaptureMode(bool mode) {
    if (mode) {
        if (!imageCaptureMode.get()) {
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
    } else {
        imageCaptureMode.set(mode);
    }
}

void CameraModule::saveImages() {

    std::string path0 = dirToSave + "/0";
    std::string path1 = dirToSave + "/1";
    path0.erase(std::remove(path0.begin(), path0.end(), ' '), path0.end());
    path1.erase(std::remove(path1.begin(), path1.end(), ' '), path1.end());
    path0.erase(std::remove(path0.begin(), path0.end(), '\n'), path0.end());
    path1.erase(std::remove(path1.begin(), path1.end(), '\n'), path1.end());
    path0 += "/" + std::to_string(saveCounter) + ".jpg";
    path1 += "/" + std::to_string(saveCounter) + ".jpg";
    cv::imwrite(path0, *leftImage.getImage());
    cv::imwrite(path1, *rightImage.getImage());
    std::cout << path0 << std::endl;
    saveCounter++;
}

void CameraModule::getDirectoryToSave() {
    auto end = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    dirToSave = std::ctime(&end_time);
}

void CameraModule::stopThread() {
    threadStop.set(true);
}

bool CameraModule::active() {
    bool result = !threadStop.get();
    return result;
}
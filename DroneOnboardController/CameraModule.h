//
// Created by nickolay on 09.06.2020.
//

#ifndef DRONEAPP_CAMERAMODULE_H
#define DRONEAPP_CAMERAMODULE_H


#include <opencv2/core/mat.hpp>
#include <mutex>
#include "DroneImage.h"
#include "../Utils/MutexBool.h"
#ifdef __arm__
#include "DJI_guidance.h"
#include "DJI_utility.h"
#endif

class CameraModule {

public:
    CameraModule();
    ~CameraModule();
    DroneImage leftImage;
    DroneImage rightImage;
    bool active();
    int startThread();
    void stopThread();
    void setImageCaptureMode(bool mode);
    int setTestMode(int i);

#ifdef __arm__                                                           //dji modules installed only on raspberry pi
    static int my_callback(int data_type, int data_len, char *content);
    static cv::Mat g_greyscale_image_left;
    static cv::Mat	g_greyscale_image_right;
    static e_vbus_index sensor_id;
    static DJI_lock  g_lock;
    static DJI_event g_event;
    //const int WIDTH = 320;
    //const int HEIGHT = 240;
    //const int IMAGE_SIZE = 76800;
#endif
private:
    MutexBool threadStop{false};
    int testMode = 0;      //1 - read images from video0/1 , 2 - from storage, 0 - dji system
    void getDirectoryToSave();
    std::string dirToSave = "";
    int saveCounter=0;
    MutexBool imageCaptureMode{false};

    void saveImages();
};


#endif //DRONEAPP_CAMERAMODULE_H

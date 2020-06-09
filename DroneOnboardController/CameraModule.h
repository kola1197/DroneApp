//
// Created by nickolay on 09.06.2020.
//

#ifndef DRONEAPP_CAMERAMODULE_H
#define DRONEAPP_CAMERAMODULE_H


#include <opencv2/core/mat.hpp>
#include <mutex>
#include "DroneImage.h"
#include "../Utils/MutexBool.h"

class CameraModule {

public:
    CameraModule();
    ~CameraModule();
    DroneImage leftImage;
    DroneImage rightImage;
    bool active();
    void startThread();
    void stopThread();
private:
    MutexBool threadStop{false};
    int testMode = 1;      //1 - read images from video0/1 , 2 - from storage, 0 - dji system
};


#endif //DRONEAPP_CAMERAMODULE_H

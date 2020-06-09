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
#include "CameraModule.h"

void CameraModule::startThread() {
    if (testMode == 1) {
        std::thread thr([this]() {
            std::cout<<"here 010"<<std::endl;
            cv::VideoCapture leftCamera("/dev/video2");
            std::cout<<"here 020"<<std::endl;
            cv::VideoCapture rightCamera("/dev/video0");

            //cv::VideoCapture rightCamera(0);
            cv::Mat frame;
            cv::Mat out;
            std::cout<<threadStop.get()<<" thread stop value"<<std::endl;
            while (!threadStop.get()) {
                leftCamera >> frame;
                cv::cvtColor(frame, out, CV_BGR2RGB);
                resize(out, out, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
                cv::imwrite("/home/nickolay/Code/DroneApp/OUT.jpg",out);
                leftImage.setImage(out.size(), out.data);

                rightCamera >> frame;
                cv::cvtColor(frame, out, CV_BGR2RGB);
                resize(out, out, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
                rightImage.setImage(out.size(), out.data);
            }
        });
        thr.detach();
    }
}

CameraModule::CameraModule()
{

}

CameraModule::~CameraModule()
{
    stopThread();
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
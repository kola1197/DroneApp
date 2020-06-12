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

void CameraModule::startThread() {
    if (testMode == 1) {
        std::thread thr([this]() {
            cv::VideoCapture leftCamera("/dev/video2");
            cv::VideoCapture rightCamera("/dev/video0");

            //cv::VideoCapture rightCamera(0);
            cv::Mat frame;
            cv::Mat out;
            //std::cout<<threadStop.get()<<" thread stop value"<<std::endl;
            while (!threadStop.get()) {
                leftCamera >> frame;
                cv::cvtColor(frame, out, CV_BGR2RGB);
                resize(out, out, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
                //cv::imwrite("/home/nickolay/Code/DroneApp/OUT.jpg",out);
                leftImage.setImage(out.size(), out.data);

                rightCamera >> frame;
                cv::cvtColor(frame, out, CV_BGR2RGB);
                resize(out, out, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
                rightImage.setImage(out.size(), out.data);
                if (imageCaptureMode.get())
                {
                    saveImages();
                }
            }
        });
        thr.detach();
    }
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
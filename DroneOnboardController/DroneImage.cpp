//
// Created by nickolay on 09.06.2020.
//

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "DroneImage.h"

DroneImage::DroneImage()
{

    data = cv::imread("../../testim.jpg", CV_LOAD_IMAGE_COLOR);
    cv::resize(data, data, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
    setImage(data.size(),data.data);
}

void DroneImage::setImage(std::shared_ptr<cv::Mat> im)
{
       mutex.lock();
       image = std::shared_ptr<cv::Mat>(new cv::Mat( im->size(), CV_8UC3, im->data));
       mutex.unlock();
}

void DroneImage::setImage(cv::Size size, uchar* dataArray)
{
    mutex.lock();
    data = cv::Mat(size, CV_8UC3, dataArray);
    data = data.clone();
    image = std::shared_ptr<cv::Mat>(new cv::Mat(data));
    //saveToTestDir();
    mutex.unlock();
}

void DroneImage::saveToTestDir()
{
    mutex.lock();
    std::cout<<"trying to save"<<std::endl;
    cv::imwrite("../../testOut.jpg",*image.get());
    std::cout<<"saved"<<std::endl;
    mutex.unlock();
}

 std::shared_ptr<cv::Mat> DroneImage::getImage()
 {
     mutex.lock();
     std::shared_ptr<cv::Mat> result = image;
     mutex.unlock();
     return result;
 }
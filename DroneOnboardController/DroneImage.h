//
// Created by nickolay on 09.06.2020.
//

#ifndef DRONEAPP_DRONEIMAGE_H
#define DRONEAPP_DRONEIMAGE_H


#include <opencv2/core/mat.hpp>
#include <mutex>
#include <memory>
#include "../Utils/AsyncVar.h"

class DroneImage {
public:
    DroneImage();
    std::shared_ptr<cv::Mat> getImage();
    void setImage(std::shared_ptr<cv::Mat> im);
    void setImage(cv::Size size, uchar* data);
    void setGrayImage(std::shared_ptr<cv::Mat> im);
    void setGrayImage(cv::Size size, uchar* data);
private:
    std::shared_ptr<cv::Mat> image;
    std::mutex mutex;
    cv::Mat data;
    void saveToTestDir();
};


#endif //DRONEAPP_DRONEIMAGE_H

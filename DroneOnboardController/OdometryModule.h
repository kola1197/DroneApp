//
// Created by nickolay on 25.01.2021.
//

#ifndef DRONEAPP_ODOMETRYMODULE_H
#define DRONEAPP_ODOMETRYMODULE_H

//#include "StatsServer.h"
#include "CameraModule.h"
#include "../Utils/AsyncVar.h"
#include "PX4Comannder.h"
#include "opencv2/core/types_c.h"

class OdometryModule {
public:
    OdometryModule(CameraModule* _camModule);
    OdometryModule();
    void startThread();
    double fps = 0;
    AsyncVar<CvPoint3D32f> coordinates{CvPoint3D32f{0,0,0}};
    AsyncVar<CvPoint3D32f> targetPoint{CvPoint3D32f{1,1,0}};
    void setCurrentPointAsZero();
    PX4Comannder px4Commander;
private:
    AsyncVar<bool> setZero{false};
    CameraModule* camModule;
    AsyncVar<bool> threadActive{true};
    void updateCoordinatsMono();
    void updateCoordinatsLidar();
    void updateCoordinatsORBLidar();
    int frameNum = 0;
    cv::Mat E, R, t, mask;
    double prev_delta = 0;
    double prev_prev_delta = 0;
    int framesDropped = 0;
    int framesCounter = 0;
    cv::Mat R_f, t_f;
    bool firstFrame = true;
    cv::Mat prevImage;
    std::vector<cv::Point2f> prevFeatures;
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
    void
    featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2,
                    std::vector<uchar> &status);

    void featureDetection(cv::Mat img_1, std::vector<cv::Point2f> &points1);

    double getAbsoluteScale(int frame_id, int sequence_id, double z_cal);
    std::vector<cv::KeyPoint> prevKeypoints;
    cv::Mat prevDescriptors;
    std::vector<cv::Point3f> pointsPoses;
    rs2::depth_frame prevDepthFrame = rs2::depth_frame(nullptr);
    void calculateTime(std::vector<std::chrono::microseconds> timeShot);


    void adaptive_non_maximal_suppresion(std::vector<cv::KeyPoint> &keypoints, const int num);
};


#endif //DRONEAPP_ODOMETRYMODULE_H

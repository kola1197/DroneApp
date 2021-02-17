//
// Created by nickolay on 25.01.2021.
//

#ifndef DRONEAPP_ODOMETRYMODULE_H
#define DRONEAPP_ODOMETRYMODULE_H

#include "StatsServer.h"
#include "../Utils/AsyncVar.h"

class OdometryModule {
public:
    OdometryModule(CameraModule* _camModule);
    OdometryModule();
    void startThread();
private:
    CameraModule* camModule;
    AsyncVar<bool> threadActive{true};
    void updateCoordinats();
    int frameNum = 0;
    cv::Mat E, R, t, mask;
    double prev_delta = 0;
    double prev_prev_delta = 0;
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
};


#endif //DRONEAPP_ODOMETRYMODULE_H

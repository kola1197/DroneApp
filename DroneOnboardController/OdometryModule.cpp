//
// Created by nickolay on 25.01.2021.
//

#include "OdometryModule.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv.hpp>
#include <thread>
#include <iostream>

OdometryModule::OdometryModule(CameraModule* _camModule)
{
    camModule = _camModule;
}

OdometryModule::OdometryModule()
{

}

void OdometryModule::startThread()
{
    std::thread odometryThread([this]()
    {
        while (threadActive.get())
        {
            if (camModule->gotImage.get()){
                updateCoordinats();
            } else {
                std::cout<<"Have not got image on camModule"<<std::endl;
            }
        }
        std::cout<<"Odometry thread stopped"<<std::endl;
    });
    odometryThread.detach();
}

void OdometryModule::updateCoordinats()
{
    std::cout<<" updateCoords "<<std::endl;
    cv::Mat leftImage(camModule->leftImage.getImage()->size(), camModule->leftImage.getImage()->type(), camModule->leftImage.getImage()->data );
    cv::Mat rightImage(camModule->rightImage.getImage()->size(), camModule->rightImage.getImage()->type(), camModule->rightImage.getImage()->data );
    //cv::imshow("testL", leftImage);
    //cv::imshow("testR", rightImage);
    //std::cout<<" imshow "<<std::endl;
    //cv::waitKey();
    cv::Mat leftImageUndistorted, rightImageUndistorted;
    cv::Mat leftCameraMatrix = (cv::Mat_<int>(3,3) <<  7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02,
                                                                    0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02,
                                                                    0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);
    cv::Mat leftDistCoeffsMat {0.000000000000e+00,0.000000000000e+00,0.000000000000e+00,0.000000000000e+00};
    undistort(leftImage, leftImageUndistorted,leftCameraMatrix, leftDistCoeffsMat);

    cv::Mat rightCameraMatrix = (cv::Mat_<int>(3,3) <<  7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02,
                                                                    0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02,
                                                                    0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);
    cv::Mat rightDistCoeffsMat { 0.000000000000e+00,0.000000000000e+00,0.000000000000e+00,0.000000000000e+00};
    undistort(rightImage, rightImageUndistorted, rightCameraMatrix, rightDistCoeffsMat);

    cv::Size imagesize = leftImageUndistorted.size();
    cv::Mat disparity_left=cv::Mat(imagesize.height,imagesize.width,leftImageUndistorted.type());

    cv::Mat leftGray,rightGray;
    cv::cvtColor(leftImageUndistorted,leftGray,cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightImageUndistorted,rightGray,cv::COLOR_BGR2GRAY);

    cv::Mat disp,disp8;
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(64,15);
    sbm->setDisp12MaxDiff(1);
    sbm->setSpeckleRange(14);
    sbm->setSpeckleWindowSize(2);
    sbm->setUniquenessRatio(3);
    sbm->setTextureThreshold(13);
    sbm->setMinDisparity(-3);
    sbm->setPreFilterCap(29);
    sbm->setPreFilterSize(7);
    sbm->compute(leftGray,rightGray,disparity_left);
    normalize(disparity_left, disp8, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow("testL", leftImageUndistorted);
    cv::imshow("testR", rightImageUndistorted);
    cv::imshow("depthMap", disp8);
    cv::waitKey();
}
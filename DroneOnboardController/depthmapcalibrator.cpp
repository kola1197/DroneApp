#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv.hpp>
#include "depthmapcalibrator.h"
#include "ui_depthmapcalibrator.h"
#include "OdometryModule.h"
#include <iostream>

DepthMapCalibrator::DepthMapCalibrator(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::DepthMapCalibrator)
{
    ui->setupUi(this);
}

DepthMapCalibrator::~DepthMapCalibrator()
{
    delete ui;
}

void DepthMapCalibrator::updateValues()
{
    ui->horizontalSlider;
    std::cout<<"Update values"<<std::endl;
    std::string leftPart = "/home/nickolay/Odometry/left/000000.png";//"/home/nickolay/Documents/DatasetsOdometry/currentTestData/0/";
    std::string rightPart = "/home/nickolay/Odometry/right/000000.png";//"/home/nickolay/Documents/DatasetsOdometry/currentTestData/1/";

    cv::Mat leftImage = cv::imread(leftPart);
    cv::Mat rightImage = cv::imread(rightPart);
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

    int numDisparties = ui->horizontalSlider->value() * 16;
    int blockSize = ui->horizontalSlider_2->value()%2==0 ? ui->horizontalSlider_2->value() + 1 : ui->horizontalSlider_2->value();

    int dispMaxDiff = ui->horizontalSlider_3->value();
    int speckleRange = ui->horizontalSlider_4->value();
    int speckleWindowSize = ui->horizontalSlider_10->value();
    int uniquenessRatio = ui->horizontalSlider_5->value();
    int textureTreshhold = ui->horizontalSlider_6->value();
    int minDispartity = ui->horizontalSlider_7->value();
    int preFilterCap = ui->horizontalSlider_8->value() < 63 ? ui->horizontalSlider_8->value() : 62;
    int preFilterSize = ui->horizontalSlider_9->value()%2==0 ? ui->horizontalSlider_9->value() + 1 : ui->horizontalSlider_9->value();

    ui->label_11->setText(QString::number(numDisparties));
    ui->label_12->setText(QString::number(blockSize));
    ui->label_13->setText(QString::number(dispMaxDiff));
    ui->label_14->setText(QString::number(speckleRange));
    ui->label_15->setText(QString::number(speckleWindowSize));
    ui->label_16->setText(QString::number(uniquenessRatio));
    ui->label_17->setText(QString::number(textureTreshhold));
    ui->label_18->setText(QString::number(minDispartity));
    ui->label_19->setText(QString::number(preFilterCap));
    ui->label_20->setText(QString::number(preFilterSize));


    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(numDisparties,blockSize);
    sbm->setDisp12MaxDiff(dispMaxDiff);
    sbm->setSpeckleRange(speckleRange);
    sbm->setSpeckleWindowSize(speckleWindowSize);
    sbm->setUniquenessRatio(uniquenessRatio);
    sbm->setTextureThreshold(textureTreshhold);
    sbm->setMinDisparity(minDispartity);
    sbm->setPreFilterCap(preFilterCap);
    sbm->setPreFilterSize(preFilterSize);
    sbm->compute(leftGray,rightGray,disparity_left);
    normalize(disparity_left, disp8, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow("testL", leftImageUndistorted);
    cv::imshow("testR", rightImageUndistorted);
    cv::imshow("depthMap", disp8);
    cv::waitKey();
}

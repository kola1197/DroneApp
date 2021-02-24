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
#include <fstream>

#define MIN_NUM_FEAT 2000

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
            if (camModule->gotImage.get() && camModule->imageForOdometryModuleUpdated.get()){
                updateCoordinats();
                camModule->imageForOdometryModuleUpdated.set(false);
                frameNum = camModule->frameNum.get();
            } else {
                //std::cout<<"Have not got image on camModule"<<std::endl;
            }
        }
        std::cout<<"Odometry thread stopped"<<std::endl;
    });
    odometryThread.detach();
}

void OdometryModule::updateCoordinats()         //try mono
{
    std::chrono::microseconds timeOnStart = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    std::chrono::microseconds timeOnSecondPartb = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    std::chrono::microseconds timeOnSecondPart = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    std::chrono::microseconds timeOnThirdPart = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    std::chrono::microseconds timeOnForthPart = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    std::chrono::microseconds timeOnFifthPart = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    //std::cout<<"update coords"<<std::endl;
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;
    cv::Point textOrg(10, 50);

    double scale = 1.00;
    char text[100];
    double focal = 718.8560;
    //cv::Point2d pp(607.1928, 185.2157);
    cv::Point2d pp(320, 240);
    cv::Mat img_1, img_2;
    //cv::Mat R_f, t_f; //the final rotation and tranlation vectors containing the

    cv::Mat prevImage_c;
    cv::Mat currImage_c;

    camModule->leftPrevImage.getImage()->copyTo(prevImage_c); //cv::imread(filename2);
    camModule->leftImage.getImage()->copyTo(currImage_c); //cv::imread(filename2);
    //std::cout<<"update coords 1"<<std::endl;
    timeOnSecondPart = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());

    if ( !currImage_c.data || !prevImage_c.data || prevImage_c.rows == 320 || prevImage_c.rows == 240) {
        std::cout<< " --(!) Error reading images " << std::endl; //return -1;
    }
    else {
        clock_t begin = clock();
        //std::cout<<"update coords 2"<<std::endl;

        if (firstFrame){
            cvtColor(currImage_c, img_2, cv::COLOR_BGR2GRAY);
            cvtColor(prevImage_c, img_1, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> points1, points2;        //vectors to store the coordinates of the feature points
            featureDetection(img_1, points1);        //detect features in img_1
            std::vector<uchar> status;
            //std::cout<<"update coords 2.5"<<std::endl;
            featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2
            //std::cout<<"update coords 3"<<std::endl;

            if (!points1.empty() && !points2.empty()){
                E = findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

                if (E.cols == 3 && E.rows == 3) {
                    recoverPose(E, points2, points1, R, t, focal, pp, mask);
                    prevFeatures = points2;
                    prevImage = img_2.clone();
                    firstFrame = false;

                    R_f = R.clone();
                    t_f = t.clone();

                    std::cout << " First frame done" << std::endl; //return -1;
                }
            }
        }
        else{
            cv::Mat  currImage;
            // we work with grayscale images
            //cvtColor(prevImage_c, prevImage, cv::COLOR_BGR2GRAY);
            cvtColor(currImage_c, currImage, cv::COLOR_BGR2GRAY);
            std::vector<uchar> status;
            std::vector<cv::Point2f> currFeatures;
            featureTracking(prevImage_c, currImage_c, prevFeatures, currFeatures, status);
            timeOnSecondPartb = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
            if (!currFeatures.empty()) {
                E = findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
                if (E.cols == 3 && E.rows == 3) {
                    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
                    timeOnThirdPart = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch());
//            if (!R_f.data){
//                R_f = R.clone();
//                t_f = t.clone();
//                std::cout<<"R_f set"<<std::endl;            }
                    cv::Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);

                    for (int i = 0; i <
                                    prevFeatures.size(); i++) {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
                        prevPts.at<double>(0, i) = prevFeatures.at(i).x;
                        prevPts.at<double>(1, i) = prevFeatures.at(i).y;

                        currPts.at<double>(0, i) = currFeatures.at(i).x;
                        currPts.at<double>(1, i) = currFeatures.at(i).y;
                    }
                    scale = 1.0;
                    timeOnForthPart = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch());
                    //scale = getAbsoluteScale(frameNum, 0, t.at<double>(2));
                    if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

                        t_f = t_f + scale * (R_f * t);
                        R_f = R * R_f;

                    }
                    if (setZero.get()) {
                        firstFrame = true;
                        setZero.set(false);
                        /*R_f = R.clone();
                        t_f = t.clone();*/
                    }
                    if (prevFeatures.size() < MIN_NUM_FEAT) {
                        //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
                        //cout << "trigerring redection" << endl;
                        featureDetection(prevImage, prevFeatures);
                        featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

                    }
                    prevImage = currImage.clone();
                    prevFeatures = currFeatures;

                    timeOnFifthPart = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch());
                    int x = int(t_f.at<double>(0)) + 300;
                    int y = int(t_f.at<double>(2)) + 100;
                    circle(traj, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);

                    rectangle(traj, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
                    coordinates.set(CvPoint3D32f(t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)));
                    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1),
                            t_f.at<double>(2));
                    putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
                    //std::cout<<text<<std::endl;
                    //imshow( "Road facing camera", currImage_c );
                    imshow("Trajectory", traj);
                    //std::cout<<text<<std::endl;
                    cv::waitKey(1);
                }
            }
        }
    }

    std::chrono::microseconds timeOnTheEnd = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());

    double delta_time = (timeOnTheEnd - timeOnStart).count();
    double avg_delta = (prev_delta + prev_prev_delta + delta_time) / 3;
    fps = 1000000/avg_delta;
    prev_prev_delta = prev_delta;
    prev_delta = delta_time;
    //std::cout<<"FPS: "<<fps<<"   last time - "<<delta_time<<std::endl;

    double firstPart = (timeOnSecondPart - timeOnStart).count();
    double secondPart = (timeOnSecondPartb - timeOnSecondPart).count();
    double thirdPart =  (timeOnThirdPart - timeOnSecondPartb).count();
    double forthPart = (timeOnForthPart - timeOnThirdPart).count();
    double fifthPart = (timeOnFifthPart - timeOnForthPart).count();
    //std::cout<<"Time frames: 1)" <<firstPart<<" 2) "<<secondPart<<" 3) "<<thirdPart<<" 4) "<<forthPart<<" 5) "<<fifthPart<<std::endl;

}

double OdometryModule::getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{

    std::string line;
    int i = 0;
    std::ifstream myfile ("/home/nickolay/Odometry dataset//00.txt");
    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            //cout << line << '\n';
            for (int j=0; j<12; j++)  {
                in >> z ;
                if (j==7) y=z;
                if (j==3)  x=z;
            }
            i++;
        }
        myfile.close();
    }

    else {
        std::cout << "Unable to open file"<<std::endl;
        return 0;
    }

    return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}


void OdometryModule::featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, std::vector<uchar>& status)	{

//this function automatically gets rid of points for which tracking fails

    std::vector<float> err;
    cv::Size winSize = cv::Size(15,15);
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    if (points1.size()>2) {
        calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
        int indexCorrection = 0;
        for( int i=0; i<status.size(); i++)
        {  cv::Point2f pt = points2.at(i- indexCorrection);
            if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
                if((pt.x<0)||(pt.y<0))	{
                    status.at(i) = 0;
                }
                points1.erase (points1.begin() + (i - indexCorrection));
                points2.erase (points2.begin() + (i - indexCorrection));
                indexCorrection++;
            }
        }
    }
}


void OdometryModule::featureDetection(cv::Mat img_1, std::vector<cv::Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
    std::vector<cv::KeyPoint> keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints_1, points1, std::vector<int>());
}

void OdometryModule::setCurrentPointAsZero()
{
    setZero.set(true);
}
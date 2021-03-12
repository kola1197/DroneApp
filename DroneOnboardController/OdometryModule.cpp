//
// Created by nickolay on 25.01.2021.
//

#include "OdometryModule.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <thread>
#include <iostream>
#include <fstream>
#include <librealsense2/rsutil.h>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/xfeatures2d.hpp"
#endif
#define MIN_NUM_FEAT 2000
#include <opencv2/video/tracking.hpp>
#include <string>
#include <vector>
#include <algorithm>

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
                updateCoordinatsORBLidar();
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

void OdometryModule::updateCoordinatsORBLidar(){
    std::vector<std::chrono::microseconds> timeShot;
    std::chrono::microseconds startTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
    timeShot.push_back(startTime);

    camModule->depthImageMutex.lock();
    cv::Mat colorImage = camModule->leftImage.getImage()->clone();
    rs2::depth_frame depthFrame(camModule->depthFrame);
    rs2_intrinsics intrinsics = camModule->DepthIntrinsics.get();
    camModule->depthImageMutex.unlock();
    /*float testResultVector[3];
    float testInputPixelAsFloat[2];
    testInputPixelAsFloat[0] = 320;
    testInputPixelAsFloat[1] = 240;
    float testdistance = depthFrame.get_distance(320, 240);
    rs2_deproject_pixel_to_point(testResultVector, &intrinsics, testInputPixelAsFloat, testdistance);
    std::cout <<"DEPROJECTED POINT after: "<< "x = " << testResultVector[0] << ", y = " << testResultVector[1] << ", z = " << testResultVector[2] << std::endl;*/

    cv::Mat greyImage;
    cv::cvtColor(colorImage, greyImage, cv::COLOR_RGB2GRAY);

    timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot1


    int minHessian = 700;

    int fast_threshold = 30;
    //bool nonmaxSuppression = true;
    std::vector<cv::KeyPoint> keypoints;
    //FAST(greyImage, keypoints, fast_threshold, nonmaxSuppression);

    cv::Mat descriptors;
    //siftDetector->detect( greyImage, keypoints, descriptors );
    //cv::Ptr<cv::xfeatures2d::SIFT> siftDetector = cv::xfeatures2d::SIFT::create(minHessian );

    auto detector_ = cv::ORB::create(3000);
    auto descriptor_ = cv::ORB::create();
    auto matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    cv::Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(fast_threshold, true);
    fastDetector->detect(greyImage, keypoints);

    //detector_->detect(greyImage, keypoints);
    adaptive_non_maximal_suppresion(keypoints, 500);
    descriptor_->compute(greyImage, keypoints, descriptors);


    //cv::Ptr<cv::xfeatures2d::SURF> siftDetector = cv::xfeatures2d::SURF::create(minHessian );

    timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot2
    //siftDetector->detectAndCompute( greyImage, cv::noArray(), keypoints, descriptors );
    //siftDetector->compute(greyImage, keypoints, descriptors);

    timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot3
    std::cout << "Keypoints size: " << keypoints.size() << "  prevKeypoints size " <<prevKeypoints.size() <<std::endl;
    std::cout << "Descriptors size: " << descriptors.size() << "  prevSescriptors size " <<prevDescriptors.size() <<std::endl;


    if (prevKeypoints.size()>4 && keypoints.size()>4) {


        std::vector<cv::DMatch> prettyGoodMatches;
        //cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        //std::vector<std::vector<cv::DMatch>>  knn_matches;
        //matcher->knnMatch(prevDescriptors, descriptors,  knn_matches,2);
        timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot4

        double cameraM[3][3] = {{camModule->DepthIntrinsics.get().fx, 0.000000, camModule->DepthIntrinsics.get().ppx}, {0.000000, camModule->DepthIntrinsics.get().fx, camModule->DepthIntrinsics.get().ppy}, {0, 0, 1}}; //camera matrix to be edited
        E = cv::Mat(3, 3, CV_64FC1, cameraM);
        std::vector<cv::Point3f> pts_3d;
        std::vector<cv::Point2f> pts_2d;
        std::vector<cv::DMatch> goodMatches;


        matcher_crosscheck_->match(prevDescriptors, descriptors, goodMatches);

        // calculate the min/max distance
        /*auto min_max = minmax_element(prettyGoodMatches.begin(), prettyGoodMatches.end(), [](const auto &lhs, const auto &rhs) {
            return lhs.distance < rhs.distance;
        });*/
        auto min_max = minmax_element(goodMatches.begin(), goodMatches.end());
        auto min_element = min_max.first;
        auto max_element = min_max.second;
        // std::cout << "Min distance: " << min_element->distance << std::endl;
        // std::cout << "Max distance: " << max_element->distance << std::endl;

        // threshold: distance should be smaller than two times of min distance or a give threshold
        double frame_gap = 1;//frame_current_.frame_id_ - frame_last_.frame_id_;
        for (int i = 0; i < goodMatches.size(); i++)
        {
            if (goodMatches.at(i).distance <= std::max(2.0 * min_element->distance, 30.0 * frame_gap))
            {
                prettyGoodMatches.push_back(goodMatches.at(i));
            }
        }

        /*const float ratio_thresh = 0.60f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                prettyGoodMatches.push_back(knn_matches[i][0]);
            }
        }*/

        /*double max_dist = 0; double min_dist = 100;
        for( int i = 0; i < prettyGoodMatches.size(); i++ )
        {
            double dist = prettyGoodMatches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        for( int i = 0; i < prettyGoodMatches.size(); i++ )
        {
            if( prettyGoodMatches[i].distance < 1.2*min_dist )
            {
                goodMatches.push_back( prettyGoodMatches[i]);
            }
        }*/

        std::vector<float> sortedDiff;
        for (cv::DMatch m : prettyGoodMatches) {
            sortedDiff.push_back(sqrt((prevKeypoints[m.queryIdx].pt.x - keypoints[m.trainIdx].pt.x) * (prevKeypoints[m.queryIdx].pt.x - keypoints[m.trainIdx].pt.x)
                                      + (prevKeypoints[m.queryIdx].pt.y - keypoints[m.trainIdx].pt.y) * (prevKeypoints[m.queryIdx].pt.y - keypoints[m.trainIdx].pt.y)));
            //sortedDiff.push_back(m.distance);
        }
        std::sort(sortedDiff.begin(),sortedDiff.end());
        float medianDistance = 1;
        if (sortedDiff.size()>1) {
            medianDistance = sortedDiff[(int) sortedDiff.size() / 2];
        }
        timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot5

        /*for (size_t i = 0; i < prettyGoodMatches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                goodMatches.push_back(knn_matches[i][0]);
            }
        }*/
        std::vector<cv::Point2f> debugLines;
        float distanceTreshold = 3.5f;
        //std::cout << "A total of found " << knn_matches.size() << " Group Match Point" << std::endl;
        for (cv::DMatch m : prettyGoodMatches)
        {
            /*ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
            if (d == 0)   // bad depth
                continue;
            float dd = d / 5000.0;*/
            //cv::Point2d p1 = pixel2cam(prevKeypoints[m.queryIdx].pt, K); // Pixel coordinates to camera normalized coordinates
            float ResultVector[3];
            float InputPixelAsFloat[2] {prevKeypoints[m.queryIdx].pt.x,prevKeypoints[m.queryIdx].pt.y};
            //int w = depthFrame.get_width();
            int pt [2];
            pt[0] = (int) (InputPixelAsFloat[0] * prevDepthFrame.get_width()/greyImage.cols);
            pt[1] = (int) (InputPixelAsFloat[1] * prevDepthFrame.get_height()/greyImage.rows);
            float distance = prevDepthFrame.get_distance(pt[0],pt[1]);
            float mDist = sqrt((prevKeypoints[m.queryIdx].pt.x - keypoints[m.trainIdx].pt.x) * (prevKeypoints[m.queryIdx].pt.x - keypoints[m.trainIdx].pt.x)
                               + (prevKeypoints[m.queryIdx].pt.y - keypoints[m.trainIdx].pt.y) * (prevKeypoints[m.queryIdx].pt.y - keypoints[m.trainIdx].pt.y));
            if (distance<6 && distance > 0.1 /*&& mDist <= (distanceTreshold * medianDistance) && mDist >= ( medianDistance / distanceTreshold)*/ ){
                rs2_deproject_pixel_to_point(ResultVector, &intrinsics, InputPixelAsFloat, distance);
                pts_3d.push_back(cv::Point3f(ResultVector[0],ResultVector[1],ResultVector[2]));
                pts_2d.push_back(keypoints[m.trainIdx].pt);          // Add the 2D point of the feature position of the second image
                debugLines.push_back(cv::Point2f(prevKeypoints[m.queryIdx].pt.x,prevKeypoints[m.queryIdx].pt.y));
                debugLines.push_back(keypoints[m.trainIdx].pt);
            }
        }
        timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot6
#ifdef x64
        cv::Mat imToShow = colorImage.clone();
        for (int i=0;i<pts_3d.size();i++ ){
            char tttt[100];
            int fontFace = cv::FONT_HERSHEY_PLAIN;
            double fontScale = 1;
            int thickness = 1;
            circle(imToShow, pts_2d[i], 1, CV_RGB(255, 0, 0), 2);
            sprintf(tttt, "%02f, %02f, %02f", pts_3d[i].x,pts_3d[i].y,pts_3d[i].z);
            putText(imToShow, tttt, cv::Point2d( pts_2d[i].x - 10, pts_2d[i].y - 10), fontFace, fontScale, cv::Scalar::all(255), thickness, 6);
        }
        for (int i =0;i<debugLines.size();i=i+2){
            cv::line(imToShow,debugLines[i],debugLines[i+1], CV_RGB(0, 255, 0),3);
        }
        cv::imshow("test", imToShow);
        cv::waitKey(1);
#endif
        std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;
        if (pts_3d.size()>7 && pts_2d.size()>7){
            cv::Mat r, t;
            cv::Mat dr, dt;
            cv::solvePnPRansac(pts_3d, pts_2d, E, cv::Mat(), r, t, false,  100, 4.0, 0.99);
            //solvePnP(pts_3d, pts_2d, E, cv::Mat(), r, t, false);
            cv::Mat R;
            cv::Rodrigues(r, R);
            if (t_f.empty() && R_f.empty()){
                R_f = R.clone();
                t_f = t.clone();
            } else{
                dt = t_f + (R_f * t);
                dr = R * R_f;
                if (std::abs(t_f.at<double>(0) - dt.at<double>(0)) < 2 && std::abs(t_f.at<double>(1) - dt.at<double>(1)) < 2
                    && std::abs(t_f.at<double>(2) - dt.at<double>(2)) < 2){
                    t_f = t_f + (R_f * t);
                    R_f = R * R_f;
                }
            }
            //std::cout << "R=" << std::endl << R << std::endl;
            //std::cout << "t=" << std::endl << t << std::endl;
            char text[100];
            sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1),
                    t_f.at<double>(2));
            coordinates.set(cvPoint3D32f(t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)));
            std::cout << text<< std::endl;
        }
        else {
            framesDropped++;
        }
        framesCounter ++;
        std::cout<<framesDropped<< " frames dropped ( "<<100*framesDropped/framesCounter<<"% )"<<std::endl;
    }

    prevKeypoints = keypoints;
    prevDescriptors = descriptors;
    prevDepthFrame = depthFrame;
    std::chrono::microseconds endTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    timeShot.push_back(endTime);
    calculateTime(timeShot);
}

void OdometryModule::adaptive_non_maximal_suppresion(std::vector<cv::KeyPoint> &keypoints, const int num)
{
    // if number of keypoints is already lower than the threshold, return
    if (keypoints.size() < num)
    {
        return;
    }

    // sort the keypoints according to its reponse (strength)
    std::sort(keypoints.begin(), keypoints.end(), [&](const cv::KeyPoint &lhs, const cv::KeyPoint &rhs) {
        return lhs.response > rhs.response;
    });

    // vector for store ANMS points
    std::vector<cv::KeyPoint> ANMSpt;

    std::vector<double> rad_i;
    rad_i.resize(keypoints.size());

    std::vector<double> rad_i_sorted;
    rad_i_sorted.resize(keypoints.size());

    // robust coefficient: 1/0.9 = 1.1
    const float c_robust = 1.11;

    // computing the suppression radius for each feature (strongest overall has radius of infinity)
    // the smallest distance to another point that is significantly stronger (based on a robustness parameter)
    for (int i = 0; i < keypoints.size(); ++i)
    {
        const float response = keypoints.at(i).response * c_robust;

        // maximum finit number of double
        double radius = std::numeric_limits<double>::max();

        for (int j = 0; j < i && keypoints.at(j).response > response; ++j)
        {
            radius = std::min(radius, cv::norm(keypoints.at(i).pt - keypoints.at(j).pt));
        }

        rad_i.at(i) = radius;
        rad_i_sorted.at(i) = radius;
    }

    // sort it
    std::sort(rad_i_sorted.begin(), rad_i_sorted.end(), [&](const double &lhs, const double &rhs) {
        return lhs > rhs;
    });

    // find the final radius
    const double final_radius = rad_i_sorted.at(num - 1);
    for (int i = 0; i < rad_i.size(); ++i)
    {
        if (rad_i.at(i) >= final_radius)
        {
            ANMSpt.push_back(keypoints.at(i));
        }
    }

    // swap address to keypoints, O(1) time
    keypoints.swap(ANMSpt);
}

void OdometryModule::updateCoordinatsLidar()
{
    std::vector<std::chrono::microseconds> timeShot;
    std::chrono::microseconds startTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
    timeShot.push_back(startTime);

    camModule->depthImageMutex.lock();
    cv::Mat colorImage = camModule->leftImage.getImage()->clone();
    rs2::depth_frame depthFrame(camModule->depthFrame);
    rs2_intrinsics intrinsics = camModule->DepthIntrinsics.get();
    camModule->depthImageMutex.unlock();
    /*float testResultVector[3];
    float testInputPixelAsFloat[2];
    testInputPixelAsFloat[0] = 320;
    testInputPixelAsFloat[1] = 240;
    float testdistance = depthFrame.get_distance(320, 240);
    rs2_deproject_pixel_to_point(testResultVector, &intrinsics, testInputPixelAsFloat, testdistance);
    std::cout <<"DEPROJECTED POINT after: "<< "x = " << testResultVector[0] << ", y = " << testResultVector[1] << ", z = " << testResultVector[2] << std::endl;*/

    cv::Mat greyImage;
    cv::cvtColor(colorImage, greyImage, cv::COLOR_RGB2GRAY);

    timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot1


    int minHessian = 700;

    int fast_threshold = 30;
    //bool nonmaxSuppression = true;
    std::vector<cv::KeyPoint> keypoints;
    //FAST(greyImage, keypoints, fast_threshold, nonmaxSuppression);

    cv::Mat descriptors;
    //siftDetector->detect( greyImage, keypoints, descriptors );
    //cv::Ptr<cv::xfeatures2d::SIFT> siftDetector = cv::xfeatures2d::SIFT::create(minHessian );
    cv::Ptr<cv::xfeatures2d::SURF> siftDetector = cv::xfeatures2d::SURF::create(minHessian );

    timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot2
    //siftDetector->detectAndCompute( greyImage, cv::noArray(), keypoints, descriptors );
    cv::Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(fast_threshold, true);
    fastDetector->detect(greyImage, keypoints);
    siftDetector->compute(greyImage, keypoints, descriptors);

    timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot3
    std::cout << "Keypoints size: " << keypoints.size() << "  prevKeypoints size " <<prevKeypoints.size() <<std::endl;
    std::cout << "Descriptors size: " << descriptors.size() << "  prevSescriptors size " <<prevDescriptors.size() <<std::endl;


    if (prevKeypoints.size()>4 && keypoints.size()>4) {


        std::vector<cv::DMatch> prettyGoodMatches;
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        std::vector<std::vector<cv::DMatch>>  knn_matches;
        matcher->knnMatch(prevDescriptors, descriptors,  knn_matches,2);
        timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot4

        double cameraM[3][3] = {{camModule->DepthIntrinsics.get().fx, 0.000000, camModule->DepthIntrinsics.get().ppx}, {0.000000, camModule->DepthIntrinsics.get().fx, camModule->DepthIntrinsics.get().ppy}, {0, 0, 1}}; //camera matrix to be edited
        E = cv::Mat(3, 3, CV_64FC1, cameraM);
        std::vector<cv::Point3f> pts_3d;
        std::vector<cv::Point2f> pts_2d;
        std::vector<cv::DMatch> goodMatches;

        const float ratio_thresh = 0.60f;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                prettyGoodMatches.push_back(knn_matches[i][0]);
            }
        }

        /*double max_dist = 0; double min_dist = 100;
        for( int i = 0; i < prettyGoodMatches.size(); i++ )
        {
            double dist = prettyGoodMatches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        for( int i = 0; i < prettyGoodMatches.size(); i++ )
        {
            if( prettyGoodMatches[i].distance < 1.2*min_dist )
            {
                goodMatches.push_back( prettyGoodMatches[i]);
            }
        }*/

        std::vector<float> sortedDiff;
        for (cv::DMatch m : prettyGoodMatches) {
            sortedDiff.push_back(sqrt((prevKeypoints[m.queryIdx].pt.x - keypoints[m.trainIdx].pt.x) * (prevKeypoints[m.queryIdx].pt.x - keypoints[m.trainIdx].pt.x)
            + (prevKeypoints[m.queryIdx].pt.y - keypoints[m.trainIdx].pt.y) * (prevKeypoints[m.queryIdx].pt.y - keypoints[m.trainIdx].pt.y)));
            //sortedDiff.push_back(m.distance);
        }
        std::sort(sortedDiff.begin(),sortedDiff.end());
        float medianDistance = 1;
        if (sortedDiff.size()>1) {
             medianDistance = sortedDiff[(int) sortedDiff.size() / 2];
        }
        timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot5

        /*for (size_t i = 0; i < prettyGoodMatches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                goodMatches.push_back(knn_matches[i][0]);
            }
        }*/
        std::vector<cv::Point2f> debugLines;
        float distanceTreshold = 3.5f;
        //std::cout << "A total of found " << knn_matches.size() << " Group Match Point" << std::endl;
        for (cv::DMatch m : prettyGoodMatches)
        {
            /*ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
            if (d == 0)   // bad depth
                continue;
            float dd = d / 5000.0;*/
            //cv::Point2d p1 = pixel2cam(prevKeypoints[m.queryIdx].pt, K); // Pixel coordinates to camera normalized coordinates
            float ResultVector[3];
            float InputPixelAsFloat[2] {prevKeypoints[m.queryIdx].pt.x,prevKeypoints[m.queryIdx].pt.y};
            //int w = depthFrame.get_width();
            int pt [2];
            pt[0] = (int) (InputPixelAsFloat[0] * prevDepthFrame.get_width()/greyImage.cols);
            pt[1] = (int) (InputPixelAsFloat[1] * prevDepthFrame.get_height()/greyImage.rows);
            float distance = prevDepthFrame.get_distance(pt[0],pt[1]);
            float mDist = sqrt((prevKeypoints[m.queryIdx].pt.x - keypoints[m.trainIdx].pt.x) * (prevKeypoints[m.queryIdx].pt.x - keypoints[m.trainIdx].pt.x)
                    + (prevKeypoints[m.queryIdx].pt.y - keypoints[m.trainIdx].pt.y) * (prevKeypoints[m.queryIdx].pt.y - keypoints[m.trainIdx].pt.y));
            if (distance<6 && distance > 0.1 && mDist <= (distanceTreshold * medianDistance) && mDist >= ( medianDistance / distanceTreshold) ){
                rs2_deproject_pixel_to_point(ResultVector, &intrinsics, InputPixelAsFloat, distance);
                pts_3d.push_back(cv::Point3f(ResultVector[0],ResultVector[1],ResultVector[2]));
                pts_2d.push_back(keypoints[m.trainIdx].pt);          // Add the 2D point of the feature position of the second image
                debugLines.push_back(cv::Point2f(prevKeypoints[m.queryIdx].pt.x,prevKeypoints[m.queryIdx].pt.y));
                debugLines.push_back(keypoints[m.trainIdx].pt);
            }
        }
        timeShot.push_back(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()));     //timeshot6
#ifdef x64
        cv::Mat imToShow = colorImage.clone();
        for (int i=0;i<pts_3d.size();i++ ){
            char tttt[100];
            int fontFace = cv::FONT_HERSHEY_PLAIN;
            double fontScale = 1;
            int thickness = 1;
            circle(imToShow, pts_2d[i], 1, CV_RGB(255, 0, 0), 2);
            sprintf(tttt, "%02f, %02f, %02f", pts_3d[i].x,pts_3d[i].y,pts_3d[i].z);
            putText(imToShow, tttt, cv::Point2d( pts_2d[i].x - 10, pts_2d[i].y - 10), fontFace, fontScale, cv::Scalar::all(255), thickness, 6);
        }
        for (int i =0;i<debugLines.size();i=i+2){
            cv::line(imToShow,debugLines[i],debugLines[i+1], CV_RGB(0, 255, 0),3);
        }
        cv::imshow("test", imToShow);
        cv::waitKey(1);
#endif
        std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;
        if (pts_3d.size()>7 && pts_2d.size()>7){
            cv::Mat r, t;
            cv::Mat dr, dt;
            cv::solvePnPRansac(pts_3d, pts_2d, E, cv::Mat(), r, t, false,  100, 4.0, 0.99);
            //solvePnP(pts_3d, pts_2d, E, cv::Mat(), r, t, false);
            cv::Mat R;
            cv::Rodrigues(r, R);
            if (t_f.empty() && R_f.empty()){
                R_f = R.clone();
                t_f = t.clone();
            } else{
                dt = t_f + (R_f * t);
                dr = R * R_f;
                if (std::abs(t_f.at<double>(0) - dt.at<double>(0)) < 2 && std::abs(t_f.at<double>(1) - dt.at<double>(1)) < 2
                && std::abs(t_f.at<double>(2) - dt.at<double>(2)) < 2){
                    t_f = t_f + (R_f * t);
                    R_f = R * R_f;
                }
            }
            //std::cout << "R=" << std::endl << R << std::endl;
            //std::cout << "t=" << std::endl << t << std::endl;
            char text[100];
            sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1),
                    t_f.at<double>(2));
            coordinates.set(cvPoint3D32f(t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)));
            std::cout << text<< std::endl;
        }
        else {
            framesDropped++;
        }
        framesCounter ++;
        std::cout<<framesDropped<< " frames dropped ( "<<100*framesDropped/framesCounter<<"% )"<<std::endl;
    }

    prevKeypoints = keypoints;
    prevDescriptors = descriptors;
    prevDepthFrame = depthFrame;
    std::chrono::microseconds endTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    timeShot.push_back(endTime);
    calculateTime(timeShot);
}

void OdometryModule::calculateTime(std::vector<std::chrono::microseconds> timeShot)
{
    double delta_time = (timeShot[timeShot.size() -1] - timeShot[0]).count();
    double avg_delta = (prev_delta + prev_prev_delta + delta_time) / 3;
    fps = 1000000/avg_delta;
    prev_prev_delta = prev_delta;
    prev_delta = delta_time;
    //std::cout<<"FPS: "<<fps<<"   last time - "<<delta_time<<std::endl;
    std::cout<<"FPS: "<<fps<<" Iteration time: "<<delta_time<<std::endl;
    for (int i=1;i<timeShot.size();i++)
    {
        std::cout<<i<<") "<<(timeShot[i] - timeShot[i-1]).count()<<"  ";
    }
    std::cout<<""<<std::endl;
}

void OdometryModule::updateCoordinatsMono()         //try mono
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
    char textAngles[100];
    double focal = camModule->DepthIntrinsics.get().fx;//camModule->DepthIntrinsics.get();//718.8560;
    //cv::Point2d pp(607.1928, 185.2157);
    cv::Point2d pp(camModule->DepthIntrinsics.get().ppx, camModule->DepthIntrinsics.get().ppy);
    double cameraM[3][3] = {{camModule->DepthIntrinsics.get().fx, 0.000000, camModule->DepthIntrinsics.get().ppx}, {0.000000, camModule->DepthIntrinsics.get().fx, camModule->DepthIntrinsics.get().ppy}, {0, 0, 1}}; //camera matrix to be edited

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
                E = findEssentialMat(points2, points1, 1.0, pp, cv::RANSAC, 0.999, 1.0);
                E = cv::Mat(3, 3, CV_64FC1, cameraM);
                if (E.cols == 3 && E.rows == 3) {
                    recoverPose(E, points2, points1, R, t, 1.0, pp);
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
                //E = findEssentialMat(currFeatures, prevFeatures, 1.0, pp, cv::RANSAC, 0.999, 1.0, mask);
                E = cv::Mat(3, 3, CV_64FC1, cameraM);
                if (E.cols == 3 && E.rows == 3) {
                    recoverPose(E, currFeatures, prevFeatures, R, t, 1.0, pp);
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

                    rectangle(traj, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0, 0, 0), cv::FILLED);
                    coordinates.set(cvPoint3D32f(t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2)));
                    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1),
                            t_f.at<double>(2));
                    sprintf(textAngles, "Angles: alpha = %02f beta = %02f gamma = %02f", R_f.at<double>(0), R_f.at<double>(1),
                            R_f.at<double>(2));
                    circle(traj, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);
                    putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
                    //std::cout<<text<<std::endl;
                    std::cout<<textAngles<<std::endl;
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
    int fast_threshold = 30;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints_1, points1, std::vector<int>());
}

void OdometryModule::setCurrentPointAsZero()
{
    setZero.set(true);
}

//
// Created by nickolay on 09.06.2020.
//
#include <filesystem>
#include <iostream>
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
#include "opencv2/xfeatures2d.hpp"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>


//#ifdef __arm__
//e_vbus_index CameraModule::sensor_id = e_vbus4;
//cv::Mat CameraModule::g_greyscale_image_left = cv::Mat::zeros(240,320,CV_8UC1);
//cv::Mat	CameraModule::g_greyscale_image_right = cv::Mat::zeros(240,320,CV_8UC1);
//DJI_lock  CameraModule::g_lock;
//DJI_event CameraModule::g_event;
//#endif

#include <unistd.h>
#include <fstream>
#include <linux/prctl.h>
#include <sys/prctl.h>

inline bool exists (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

int CameraModule::startThread() {
    int result = 0;
    std::cout << "starting thread" << std::endl;
    /*if (captureMode.get() == CaptureMode::TEST_DATASET) {
        std::thread thr([this]() {
            cv::Mat frame;
            cv::Mat out;
            std::string leftPart = "/home/nickolay/Odometry dataset/dataset/sequences/00/image_0/";//"/home/nickolay/Odometry Dataset/image_0/";//"/home/nickolay/Documents/DatasetsOdometry/currentTestData/0/";
            std::string rightPart = "/home/nickolay/Odometry dataset/dataset/sequences/00/image_0/";//"/home/nickolay/Odometry Dataset/image_0/";//"/home/nickolay/Documents/DatasetsOdometry/currentTestData/1/";
            //std::string leftPart = "/home/nickolay/test.jpg";
            //std::string rightPart = "/home/nickolay/test.jpg";

            int counter = 0;
            while ((!threadStop.get()) && counter < 1600) {
                //TODO: check file exists
                if (!imageForOdometryModuleUpdated.get()) {
                    std::string s = leftPart.c_str();
                    s.append("000000");
                    s.erase(s.begin() + s.size() - std::to_string(counter).size(), s.begin() + s.size());
                    s.append(std::to_string(counter).c_str());
                    s.append(".png");
                    //std::cout << "left path: " << s << std::endl;

                    out = cv::imread(s);
                    if (!out.data) {
                        std::cout << "Image read error on test 2!!!" << std::endl;
                    }
                    //cv::imshow("testL", out);
                    leftPrevImage.setImage(leftImage.getImage()->size(), leftImage.getImage()->data);
                    leftImage.setImage(out.size(), out.data);
                    int type = out.type();
                    //std::cout<<"type: "<<type<<std::endl;
                    //cv::imshow("test", out);
                    //cv::waitKey(0);
                    s = rightPart.c_str();
                    s.append("000000");
                    s.erase(s.begin() + s.size() - std::to_string(counter).size(), s.begin() + s.size());
                    s.append(std::to_string(counter).c_str());
                    s.append(".png");
                    //std::cout<<"right path: "<<s<<std::endl;
                    out = cv::imread(s);
                    //cv::cvtColor(frame, out, CV_GRAY2RGB);
                    rightPrevImage.setImage(rightImage.getImage()->size(), rightImage.getImage()->data);
                    rightImage.setImage(out.size(), out.data);
                    gotImage.set(true);
                    usleep(33000);
                    frameNum.set(counter);
                    imageForOdometryModuleUpdated.set(true);
                    counter++;
                }
            }
            std::cout << "Thread done" << std::endl;
        });
        thr.detach();
    }*/
    if (captureMode.get() == CaptureMode::REALSENSE) {                                                                   //TODO: add check of video0/video2 exist
        std::thread thr([this]() {
            std::string s = "CM_Realsencz";
            prctl(PR_SET_NAME,(char *)s.c_str());
            int counter = 0;
            std::cout<<"REALSENSE"<<std::endl;
            cv::VideoCapture cap(2 ); // open the video camera no. 10
            if (cap.isOpened()) {
                #define RIGHT_CAMERA_EXISTS
                cap.set(cv::CAP_PROP_FRAME_WIDTH, 1240);
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
            }
            rs2::config cfg;
            float ResultVector[3];
            float InputPixelAsFloat[2];
            InputPixelAsFloat[0] = 320;
            InputPixelAsFloat[1] = 240;
            cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16 , 60);
            cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 60);
            //cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 60);

            rs2::colorizer color_map;
            rs2::pipeline pipe;
            auto MyPipelineProfile = pipe.start(cfg);
            auto DepthStream = MyPipelineProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            auto ColorStream = MyPipelineProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            rs2_intrinsics lDepthIntrinsics = DepthStream.get_intrinsics();
            rs2_intrinsics lColorIntrinsics = ColorStream.get_intrinsics();

            //rs2::pipeline_profile profile = pipe.start();
            float depth_scale = get_depth_scale(MyPipelineProfile.get_device());
            rs2_stream align_to = find_stream_to_align(MyPipelineProfile.get_streams());
            rs2::align align(align_to);

            for (int i = 0; i<10; i++)//to skip first few frames when device just initiated
                auto frames = pipe.wait_for_frames();
            while (!threadStop.get()){

                rs2::frameset frameset = pipe.wait_for_frames(); // Wait for next set of frames from the camera
                rs2::frame color = frameset.get_color_frame();


                if (profile_changed(pipe.get_active_profile().get_streams(), MyPipelineProfile.get_streams()))
                {
                    //If the profile was changed, update the align object, and also get the new device's depth scale
                    MyPipelineProfile = pipe.get_active_profile();
                    align_to = find_stream_to_align(MyPipelineProfile.get_streams());
                    align = rs2::align(align_to);
                    depth_scale = get_depth_scale(MyPipelineProfile.get_device());
                }
                auto processed = align.process(frameset);
                //rs2::depth_frame localDepthFrame = data.get_depth_frame();
                rs2::depth_frame localDepthFrame = processed.get_depth_frame();


                const int w = color.as<rs2::video_frame>().get_width();
                const int h = color.as<rs2::video_frame>().get_height();


                //float distance = localDepthFrame.get_distance(320, 240);

                //rs2_deproject_pixel_to_point(ResultVector, &lDepthIntrinsics, InputPixelAsFloat, distance);
                //std::cout <<"DEPROJECTED POINT before: "<< "x = " << ResultVector[0] << ", y = " << ResultVector[1] << ", z = " << ResultVector[2] << std::endl;
                //std::cout<<localDepthFrame.get_data()<<std::endl;
                rs2::frame depth = localDepthFrame.apply_filter(color_map);

                cv::Mat colorImageA(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat colorImage;
                cv::cvtColor(colorImageA, colorImage, cv::COLOR_BGR2RGB);

                leftPrevImage.setImage(leftImage.getImage()->size(), leftImage.getImage()->data);
                leftImage.setImage(colorImage.size(), colorImage.data);

                const int ww = depth.as<rs2::video_frame>().get_width();
                const int hh = depth.as<rs2::video_frame>().get_height();
                cv::Mat depthImage (cv::Size(ww, hh), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

                //circle(depthImage, cv::Point(320, 240), 1, CV_RGB(0, 255, 0), 2);

                //imshow("color test", colorImage);
                //imshow("depth test", depthImage);
                //cv::waitKey(1);

                rightPrevImage.setImage(rightImage.getImage()->size(), rightImage.getImage()->data);
                rightImage.setImage(depthImage.size(), depthImage.data);
                DepthIntrinsics.set(lDepthIntrinsics);
                ColorIntrinsics.set(lColorIntrinsics);
                //depthFrame.set(localDepthFrame);
                depthImageMutex.lock();
                //prevDepthFrame = depthFrame;
                depthFrame = localDepthFrame;
                //if (counter == 0){
                //    prevDepthFrame = depthFrame;
                //}


                if (cap.isOpened()) {
                    //#define RIGHT_CAMERA_EXISTS
                    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 1240);
                    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
                    cv::Mat frame;
                    bool bSuccess = cap.read(frame);

                    rightBoardImage.setImage(frame.size(), frame.data);
                }
                depthImageMutex.unlock();
                bool newDatasetMode = true;
                if (newDatasetMode) {
                    if (imageCaptureMode.get() != datasetSaveMode) {
                        datasetSaveMode = imageCaptureMode.get();
                        if (imageCaptureMode.get()) {
                            dataset = Dataset();
                        } else {
                            getDirectoryToSave();
                            dataset.saveDataset(dirToSave);
                            dataset.clear();
                            dataset = Dataset();
                        }
                    }
                    if (imageCaptureMode.get()) {
                        std::chrono::microseconds startTime = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
                        writeDataToDataset();
                        std::chrono::microseconds endTime = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
                        double deltaTime = (endTime - startTime).count();
                        double fps = 1000000 / deltaTime;
                        std::cout << "Writing dataset to RAM " << deltaTime << " ms. fps: " << fps << std::endl;
                    }
                }
                else{

                    std::chrono::microseconds startTime = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
                    saveImages();
                    std::chrono::microseconds endTime = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch());
                    double deltaTime = (endTime - startTime).count();
                    double fps = 1000000 / deltaTime;
                    std::cout << "Writing dataset to ssd " << deltaTime << " ms. fps: " << fps << std::endl;
                }

                gotImage.set(true);
                frameNum.set(counter);
                imageForOdometryModuleUpdated.set(true);
                //usleep(33000);
                counter++;
            }
        });
        thr.detach();
    }
    if (captureMode.get() == CaptureMode::TEST_DATASET) {
        std::thread thr([this]() {
            std::string s = "CM_TestDataset";
            prctl(PR_SET_NAME,(char *)s.c_str());
        int counter = 0;
        std::cout<<"DATASET"<<std::endl;
        std::string path = "/home/nickolay/My_Dataset/06";
        std::string imageFolderPath = path + "/0/";
        std::string depthFolderPath = path + "/3/";
        std::string intristicFolderPath = path + "/4/";
        cv::Mat out;

        while ((!threadStop.get()) && counter < 1600) {
            std::string imagePath = imageFolderPath + std::to_string(counter) + ".jpg";
            std::string depthPath = depthFolderPath + std::to_string(counter) +  ".dpt";
            std::string intristicPath = intristicFolderPath + std::to_string(counter) + ".intr";

            out = cv::imread(imagePath);
            if (!out.data) {
                //std::cout << "Image read error in dataset reader " + imagePath << std::endl;
                endOfImageStream.set(true);
            }
            else {

                leftPrevImage.setImage(leftImage.getImage()->size(), leftImage.getImage()->data);
                leftImage.setImage(out.size(), out.data);
                std::vector<std::vector<double>> depthArray;
                std::ifstream in(depthPath); // окрываем файл для чтения
                double depth1[leftImage.getImage()->size().width][leftImage.getImage()->size().height];
                if (in.is_open()) {
                    for (int i = 0; i < leftImage.getImage()->size().width; i++) {
                        for (int j = 0; j < leftImage.getImage()->size().height; j++) {
                            double d = -1;
                            in >> d;
                            depth1[i][j] = d;
                            //depthArray[i].push_back(d);
                        }
                    }
                    /*for (auto & i : depth1)
                    {
                        for (double j : i)
                        {
                            std::cout<<j<<" ";
                            //std::cout<<depthArray[i][j]<<" ";
                        }
                        std::cout<<"\n";
                    }*/
                }
                in.close();     // закрываем файл

                std::ifstream fin;
                fin.open(intristicPath, std::ios_base::in);
                rs2_intrinsics r = DepthIntrinsics.get();
                fin.read(reinterpret_cast<char *>(&r), sizeof(r));
                DepthIntrinsics.set(r);
                //fin.read(&r, sizeof (rs2_intrinsics));
                fin.close();
                depthMutex.lock();
                for (int i = 0; i < leftImage.getImage()->rows; i++) {
                    for (int j = 0; j < leftImage.getImage()->cols; j++) {
                        depth[i][j] = depth1[i][j];
                    }
                }
                depthMutex.unlock();
                gotImage.set(true);
                frameNum.set(counter);
                imageForOdometryModuleUpdated.set(true);
                counter++;
                usleep(63000);
            }
        }
        });
        thr.detach();
    }
    return result;                                  // 0 - ok, 1 - this testMode can not be set
}

//#ifdef __arm__
//int CameraModule::my_callback(int data_type, int data_len, char *content)
//{
//    g_lock.enter();
//    if (e_image == data_type && NULL != content)
//    {
//        image_data* data = (image_data* )content;
//        if ( data->m_greyscale_image_left[sensor_id] ){
//            g_greyscale_image_left = cv::Mat::zeros(240,320,CV_8UC1);
//            memcpy( g_greyscale_image_left.data, data->m_greyscale_image_left[sensor_id], 76800 );
//        }else g_greyscale_image_left.release();
//        if ( data->m_greyscale_image_right[sensor_id] ){
//            g_greyscale_image_right = cv::Mat::zeros(240,320,CV_8UC1);
//            memcpy( g_greyscale_image_right.data, data->m_greyscale_image_right[sensor_id], 76800 );
//        }else g_greyscale_image_right.release();
//        /*if ( data->m_depth_image[sensor_id] ){
//            g_depth = cv::Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
//            memcpy( g_depth.data, data->m_depth_image[sensor_id], IMAGE_SIZE * 2 );
//        }else g_depth.release();
//        if ( data->m_disparity_image[sensor_id] ){
//            g_disparity = cv::Mat::zeros(HEIGHT,WIDTH,CV_16SC1);
//            memcpy( g_disparity.data, data->m_disparity_image[sensor_id], IMAGE_SIZE * 2 );
//        }else g_disparity.release();*/
//    }
//    g_lock.leave();
//    g_event.set_event();
//
//    return 0;
//}
//
//#endif

int CameraModule::getCaptureMode() {
    return captureMode.get();
}

int CameraModule::setCaptureMode(CaptureMode i) {
    threadStop.set(true);
    captureMode.set( i);
    return startThread();
}

CameraModule::CameraModule() {
    //getDirectoryToSave();

}

CameraModule::~CameraModule() {
    stopThread();
}

float CameraModule::getDist(int x, int y) {
    depthMutex.lock();
    float result = (float) depth[x][y];
    depthMutex.unlock();
    return result;
}

void CameraModule::setImageCaptureMode(bool mode) {
    if (mode) {
        if (!imageCaptureMode.get()) {
            getDirectoryToSave();
            saveCounter = 0;
            std::string path0 = dirToSave + "/0";
            std::string path1 = dirToSave + "/1";
            std::string path2 = dirToSave + "/2";
            std::string path3 = dirToSave + "/3";
            std::string path4 = dirToSave + "/4";


            path0.erase(std::remove(path0.begin(), path0.end(), ' '), path0.end());
            path1.erase(std::remove(path1.begin(), path1.end(), ' '), path1.end());
            path2.erase(std::remove(path2.begin(), path2.end(), ' '), path2.end());
            path3.erase(std::remove(path3.begin(), path3.end(), ' '), path3.end());
            path4.erase(std::remove(path4.begin(), path4.end(), ' '), path4.end());


            path0.erase(std::remove(path0.begin(), path0.end(), '\n'), path0.end());
            path1.erase(std::remove(path1.begin(), path1.end(), '\n'), path1.end());
            path2.erase(std::remove(path2.begin(), path2.end(), '\n'), path2.end());
            path3.erase(std::remove(path3.begin(), path3.end(), '\n'), path3.end());
            path4.erase(std::remove(path4.begin(), path4.end(), '\n'), path4.end());

            std::string com0 = "mkdir -p " + path0;
            std::string com1 = "mkdir -p " + path1;
            std::string com2 = "mkdir -p " + path2;
            std::string com3 = "mkdir -p " + path3;
            std::string com4 = "mkdir -p " + path4;

            system(com0.data());
            system(com1.data());
            system(com2.data());
            system(com3.data());
            system(com4.data());
        }
        imageCaptureMode.set(mode);
    } else {
        imageCaptureMode.set(mode);
    }
}

void CameraModule::saveImages() {

    std::string path0 = dirToSave + "/0";
    std::string path1 = dirToSave + "/1";
    std::string path2 = dirToSave + "/2";
    std::string path3 = dirToSave + "/3";
    std::string path4 = dirToSave + "/4";


    path0.erase(std::remove(path0.begin(), path0.end(), ' '), path0.end());
    path1.erase(std::remove(path1.begin(), path1.end(), ' '), path1.end());
    path2.erase(std::remove(path2.begin(), path2.end(), ' '), path2.end());
    path3.erase(std::remove(path3.begin(), path3.end(), ' '), path3.end());
    path4.erase(std::remove(path4.begin(), path4.end(), ' '), path4.end());

    path0.erase(std::remove(path0.begin(), path0.end(), '\n'), path0.end());
    path1.erase(std::remove(path1.begin(), path1.end(), '\n'), path1.end());
    path2.erase(std::remove(path2.begin(), path2.end(), '\n'), path2.end());
    path3.erase(std::remove(path3.begin(), path3.end(), '\n'), path3.end());
    path4.erase(std::remove(path4.begin(), path4.end(), '\n'), path4.end());

    path0 += "/" + std::to_string(saveCounter) + ".jpg";
    path1 += "/" + std::to_string(saveCounter) + ".jpg";
    path2 += "/" + std::to_string(saveCounter) + ".jpg";
    path3 += "/" + std::to_string(saveCounter) + ".dpt";
    path4 += "/" + std::to_string(saveCounter) + ".intr";

    cv::imwrite(path0, *leftImage.getImage());
    cv::imwrite(path1, *rightImage.getImage());
#ifdef RIGHT_CAMERA_EXISTS
    cv::imwrite(path2, *rightBoardImage.getImage());
#endif
    saveDepth(path3);
    if (saveCounter==0){
        saveIntristics(path4);
    }
    //std::cout << path0 << std::endl;
    saveCounter++;
}

void CameraModule::writeDataToDataset() {
    dataset.forwardCamImages.push_back(leftImage.getImage()->clone());
    dataset.depthImages.push_back(rightImage.getImage()->clone());
    dataset.rightBoardImages.push_back(rightBoardImage.getImage()->clone());
    dataset.depthIntristics = DepthIntrinsics.get();
    std::vector<std::vector<double>> d;
    for (int i=0;i<960;i++){
        std::vector<double> dd;
        for (int j=0;j<540;j++){
            double d = depthFrame.get_distance(i,j);
            dd.push_back(d);
        }
        d.push_back(dd);
    }
    dataset.depthValues.push_back(d);
}

void CameraModule::saveIntristics(std::string path){
    std::ofstream fout;
    fout.open(path,std::ios_base::out);
    rs2_intrinsics r = DepthIntrinsics.get();
    fout.write(reinterpret_cast<char*>(&r), sizeof(r));
}

void CameraModule::saveDepth(const std::string& path){
    std::ofstream fout;
    fout.open(path,std::ios_base::out);
    double depth [depthFrame.get_width()][depthFrame.get_height()];
    double d = 0;
    for (int i=0;i<depthFrame.get_width();i++){
        for (int j=0;j<depthFrame.get_height();j++){
            d = depthFrame.get_distance(i,j);
            depth[i][j] = d;
            fout<<" "<<depth[i][j]<<" ";
        }
        fout<<"\n";
    }
    fout.close();
}

void CameraModule::getDirectoryToSave() {
    auto end = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    dirToSave = std::ctime(&end_time);
}

void CameraModule::stopThread() {
    threadStop.set(true);
}

bool CameraModule::active() {
    bool result = !threadStop.get();
    return result;
}


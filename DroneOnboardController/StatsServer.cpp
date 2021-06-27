//
// Created by nickolay on 30.05.2020.
//
#include <cstdio>
#include "StatsServer.h"
#include <cstring>    // sizeof()
#include <iostream>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>    // close()
#include <thread>
#include "opencv2/core.hpp"
#include "OdometryModule.h"
#include "CpuInfo.h"
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fcntl.h>
#include <fstream>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#define PORT 66666

//template<class To, class From>
//To any_cast(From v)
//{
//    return static_cast<To>(static_cast<void *>(v));
//}

StatsServer::StatsServer()
{
    
}

StatsServer::~StatsServer()
{

}

void StatsServer::readVoid()
{
    char buffer[1024] = {0};
    valread = read(sock , buffer, 1024);
    printf("Got message: %s""\n",buffer );
}

void StatsServer::startServer(){
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    //std::cout<<"here1"<<std::endl;

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
    //std::cout<<"here3"<<std::endl;

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address,
             sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    
    std::cout<<"waiting client"<<std::endl;
    if ((sock = accept(server_fd, (struct sockaddr *)&address,
                       (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    //sendAllert("testtesttest");
    std::thread thr([this]()
    {
        std::cout<<"reading messages"<<std::endl;
        while (!serverStop.get()) {
            getMessage();
        }
    });
    thr.detach();
    int res = camModule.startThread();
    if (res == 0) {
        odometryModule = new OdometryModule(&camModule);
        odometryModule->startThread();
        UpdateSettingsForGroundStation();
        while (!stopThread()) {
            updateDataForGroundStation();
        }
    }
    else{
        sendAllert("Wrong image capture mode");
        while (!stopThread())
        {
            sleep(1);
        }
    };

    //camModule->startThread();
//    int testcounter = 0;
//    cv::VideoCapture camera(0);
//    cv::Mat frame;
//    cv::Mat out;
//    while (!stopThread())
//    {
//        testcounter++;
//        std::string s = std::to_string(testcounter);
//        camera >> frame;
//        cv::Mat grey;
//        cvtColor(frame, grey, CV_BGR2RGB);
//
//        //sendSystemMessage(s);
//        resize(grey, out, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
//        sendImage(out, true);
//        usleep(40000);
//    }

}

std::vector<double> StatsServer::odometryTest(int fast_threshold, bool nonmaxSupression, std::ofstream* fout) {
    std::vector<double> result;
    int res = camModule.startThread();
    if (res == 0) {
        odometryModule = new OdometryModule(&camModule);
        odometryModule->fast_threshold = fast_threshold;
        odometryModule->nonmaxSupression = nonmaxSupression;
        odometryModule->startThread();
        UpdateSettingsForGroundStation();
        while (!camModule.endOfImageStream.get()) {
            //updateDataForGroundStation();
            usleep(33000);
        }
    }
    else {
        sendAllert("Wrong image capture mode");
        while (!stopThread()) {
            sleep(1);
        }
    }

    double diff = sqrt(odometryModule->coordinates.get().x*odometryModule->coordinates.get().x +
                               odometryModule->coordinates.get().y*odometryModule->coordinates.get().y +
                               odometryModule->coordinates.get().z*odometryModule->coordinates.get().z);
    std::cout<<" fast_threshold: "<<fast_threshold<<" nonmaxSupression: " <<nonmaxSupression<<std::endl;
    std::cout<<"Distance - " << diff<<" Coordinates x: "<<odometryModule->coordinates.get().x<<" y: "<<odometryModule->coordinates.get().y<<" z: "<<odometryModule->coordinates.get().z<<std::endl;
    //*fout << " fast_threshold: "<<fast_threshold<<" nonmaxSupression: " <<nonmaxSupression<<"\n";
    //*fout << "Distance - " << sqrt<<" Coordinates x: "<<odometryModule->coordinates.get().x<<" y: "<<odometryModule->coordinates.get().y<<" z: "<<odometryModule->coordinates.get().z<<"\n";
    //*fout << "\n";
    result.push_back(fast_threshold);
    result.push_back(nonmaxSupression);
    result.push_back(diff);
    result.push_back(odometryModule->coordinates.get().x);
    result.push_back(odometryModule->coordinates.get().y);
    result.push_back(odometryModule->coordinates.get().z);
    return result;
}

void StatsServer::ramTest(unsigned long &memSize, unsigned long &memFree)
{
    memSize = 0;
    memFree = 0;
    std::string token;
    std::ifstream file("/proc/meminfo");
    int counter = 0;
    while(file >> token) {
        if(token == "MemTotal:") {
            unsigned long mem;
            if(file >> mem) {
                memSize = mem;
            }
            counter++;
        }
        if(token == "MemAvailable:") {
            unsigned long mem;
            if(file >> mem) {
                memFree = mem;
            }
            counter++;
        }
        // ignore rest of the line
        if (counter==2) {
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    std::cout<<"MemSize: "<<memSize<<" MemFree: "<<memFree<<std::endl;
}

void StatsServer::updateDataForGroundStation()
{
    std::chrono::microseconds timeNow = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    if ((timeNow - lastDataUpdateForGroundStation).count() > 16666) {
        if (imageSendMode.get()) {
            //std::cout<<"image sent"<<std::endl;
            //cv::imwrite("testLeftGREYOut.jpg",*camModule.leftImage.getImage());
            bool isGrey = camModule.getCaptureMode() == 0;
            sendImage(camModule.leftImage.getImage(), true, isGrey);
            sendImage(camModule.rightImage.getImage(), false, isGrey);
        }
        sendCoordinates();
        SystemMessage s{};
        s.type = SystemMessage::FPS_COUNTER;
        s.i[0] = (int) odometryModule->fps;
        sendMessage(s);


        unsigned long memSize = 0;
        unsigned long memFree = 0;
        ramTest(memSize, memFree);
        SystemMessage ramData{};
        ramData.type = SystemMessage::RAM_DATA;
        ramData.i[0] = (int) (memSize/1024);
        ramData.i[1] = (int) (memFree/1024);
        sendMessage(ramData);

        SystemMessage cpuData{};
        cpuData.type = SystemMessage::CPU_DATA;
        std::vector<float> cpuUsage = CpuInfo::getCPULoad();
        int coresCount = cpuUsage.size()-1;
        cpuData.f[0] = cpuUsage[0];
        cpuData.f[1] = coresCount;
        cpuData.f[2] = CpuInfo::getCPUTemp();
        for (int i = 1; i<cpuUsage.size(); i++){
            cpuData.i[i-1] = cpuUsage[i];
        }
        sendMessage(cpuData);

        lastDataUpdateForGroundStation = timeNow;
    }
}

void StatsServer::sendCoordinates()
{
    SystemMessage s{};
    s.type = SystemMessage::COORDINATES;
    CvPoint3D32f point = odometryModule->coordinates.get();
    s.f[0] = point.x;
    s.f[1] = point.y;
    s.f[2] = point.z;
    sendMessage(s);
}

void StatsServer::getMessage()
{

    HarbingerMessage h;
    char hmsg[sizeof (h)];
    int hbytes;
    for (int i = 0; i < sizeof(h); i += hbytes) {
        if ((hbytes = recv(sock, hmsg +i, sizeof(h)  - i, 0)) == -1){
            //std::cout<<"error"<<std::endl;
            //errorServerStop();
        }
    }
    std::memcpy(&h,hmsg , sizeof(h));
    switch (h.type) {
        case SYSTEM_MESSAGE:
            getSystemMessage();
            break;
        case MESSAGE_WITH_GRAY_IMAGE:
            getMessageWithGrayImage();
            break;
        case MESSAGE_WITH_IMAGE:
            getMessageWithColorImage();
            break;
        case PING_MESSAGE:
            getPingMessage();
            break;
        case COMMAND_MESSAGE:
            getCommandMessage();
        case SETTINGS_MESSAGE:
            getSettingsMessage();
        default:
            break;
    }
}

void StatsServer::getSettingsMessage(){
    SettingsMessage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1)
            std::cout<<"error"<<std::endl;
    }
    std::memcpy(&m,msg , sizeof(m));
    switch (m.type) {
        case SettingsMessage::VEHICLE_MODE:
            odometryModule->px4Commander.vehicleMode.set((VehicleMode)m.i[0]);
            updateVehicleStatusForGroundStation();
            break;
        default:
            break;

    }
}

void StatsServer::getCommandMessage(){
    CommandMessage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1)
            std::cout<<"error"<<std::endl;
    }
    std::memcpy(&m,msg , sizeof(m));
    switch (m.type) {
        case CommandMessage::SET_TARGET:
            odometryModule->targetPoint.set(CvPoint3D32f{m.f[0],m.f[1],m.f[2]});
            updateTargetPointForGroundStation();
            break;
        case CommandMessage::SET_THIS_POINT_AS_ZERO:
            odometryModule->setCurrentPointAsZero();
            break;
        case CommandMessage::START:

            break;
        default:
            break;

    }
}

void StatsServer::getSystemMessage(){
    SystemMessage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1)
            std::cout<<"error"<<std::endl;
    }
    std::memcpy(&m,msg , sizeof(m));
    std::cout<<"got SYSTEM_MESSAGE::"<<m.type<<": "<<m.text<<std::endl;
    SystemMessage answer;
    switch (m.type){
        case SystemMessage::TEXT_ALLERT:
            std::cout<<m.text<<std::endl;
            break;
        case SystemMessage::START_VIDEO_STREAM:
            std::cout<<"Starting send video"<<std::endl;
            imageSendMode.set(true);
            answer.type = SystemMessage::VIDEO_STREAM_STATUS;
            answer.i[0] = 1;
            sendMessage(answer);
            break;
        case SystemMessage::STOP_VIDEO_STREAM:
            std::cout<<"Stop sending video"<<std::endl;
            imageSendMode.set(false);
            answer.type = SystemMessage::VIDEO_STREAM_STATUS;
            answer.i[0] = 0;
            sendMessage(answer);
            break;
        case SystemMessage::START_IMAGE_CAPTURE:
            camModule.setImageCaptureMode(true);
            answer.type = SystemMessage::VIDEO_CAPTURE_STATUS;
            answer.i[0] = 1;
            sendMessage(answer);
            sendAllert("Start recording");
            break;
        case SystemMessage::STOP_IMAGE_CAPTURE:
            camModule.setImageCaptureMode(false);
            answer.type = SystemMessage::VIDEO_CAPTURE_STATUS;
            answer.i[0] = 0;
            sendMessage(answer);
            sendAllert("Recording stopped");
            break;
        case SystemMessage::CONNECT_TO_PX:  //TODO: make asynk
            UpConnectionToPX4();
            break;
        default:
            break;
    }
}

void StatsServer::getMessageWithGrayImage(){
    MessageWithGrayImage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1)
            std::cout<<"error"<<std::endl;
    }
    std::memcpy(&m,msg , sizeof(m));
    std::cout<<"got MESSAGE_WITH_GREY_IMAGE on vehicle. Check Server";
}

void StatsServer::getMessageWithColorImage(){
    MessageWithImage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1)
            std::cout<<"error"<<std::endl;
    }
    std::memcpy(&m,msg , sizeof(m));
    std::cout<<"got MESSAGE_WITH_IMAGE on vehicle. Check Server";
}

void StatsServer::getPingMessage(){
    PingMessage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1){
            std::cout<<"error"<<std::endl;
        }
    }
    std::memcpy(&m, msg, sizeof(m));
    //startThreadstd::cout<<"PING"<<std::endl;
    m.time[1] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    sendMessage(m);
}

//todo add mutex
bool StatsServer::stopThread()
{
    return stopBool;
}

int StatsServer::GetCPULoad() {
    int FileHandler;
    char FileBuffer[1024];
    float load;

    FileHandler = open("/proc/loadavg", O_RDONLY);
    if(FileHandler < 0) {
        return -1; }
    read(FileHandler, FileBuffer, sizeof(FileBuffer) - 1);
    sscanf(FileBuffer, "%f", &load);
    close(FileHandler);
    return (int)(load * 100);
}

void StatsServer::sendAllert(std::string s)
{
    SystemMessage m;
    m.type = SystemMessage::TEXT_ALLERT;
    const char* text = s.c_str();
    if (s.length() < 200)                        //200 - m.text.size
    {
        for (int i=0;i<s.length();i++) {
            m.text[i] = s[i];
        }
    }
    sendMessage(m);
}

void StatsServer::sendImage(std::shared_ptr<cv::Mat> _image, bool left, bool isGrey)
{
    cv::Mat image;
    cv::resize(*_image,image,cv::Size(320,240));
    if (isGrey)
    {
        MessageWithGrayImage m{};
        counter++;
        m.height = image.size().height;//*image.size().height;
        m.width = image.size().width;
        m.dataSize = image.total()*image.elemSize();
        std::cout<<"gray size"<<m.dataSize<<std::endl;
        std::string text = "";
        if (left)
        {
            text = "left image";
            m.type = MessageWithGrayImage::LEFT_IMAGE;
        } else{
            text = "right image";
            m.type = MessageWithGrayImage::RIGHT_IMAGE;
        }
        for (int i=0;i<200;i++)
        {
            m.text[i] = '0';
        }
        for (int i=0;i<text.length();i++)
        {
            m.text[i] = text.at(i);
        }
        for (int i =0;i< m.dataSize;i++)
        {
            m.imData[i] = image.data[i];
        }
        sendMessage(m);
    } else {
        MessageWithImage m{};
        counter++;
        m.height = image.size().height;//*image.size().height;
        m.width = image.size().width;
        m.dataSize = image.total()*image.elemSize();
        std::string text = "";
        if (left)
        {
            text = "left image";
            m.type = MessageWithImage::LEFT_IMAGE;
        } else{
            text = "right image";
            m.type = MessageWithImage::RIGHT_IMAGE;
        }
        for (int i=0;i<200;i++)
        {
            m.text[i] = '0';
        }
        for (int i=0;i<text.length();i++)
        {
            m.text[i] = text.at(i);
        }
        for (int i =0;i< m.dataSize;i++)
        {
            m.imData[i] = image.data[i];
        }
        sendMessage(m);
    }
}

void StatsServer::updateTargetPointForGroundStation(){
    CommandMessage c{};
    c.type = CommandMessage::SET_TARGET;
    CvPoint3D32f targetPoint = odometryModule->targetPoint.get();
    c.f[0] = targetPoint.x;
    c.f[1] = targetPoint.y;
    c.f[2] = targetPoint.z;
    sendMessage(c);
}

void StatsServer::updateVehicleStatusForGroundStation()
{
    SettingsMessage s{};
    s.type = SettingsMessage::VEHICLE_MODE;
    s.i[0] = odometryModule->px4Commander.vehicleMode.get();
    sendMessage(s);
}

void StatsServer::UpdateSettingsForGroundStation()
{
    updateTargetPointForGroundStation();
    updateVehicleStatusForGroundStation();
}

void StatsServer::UpConnectionToPX4(){
    std::thread thr([this]() {
        odometryModule->px4Commander.startDronekit();
        std::cout<<"dronekit started"<<std::endl;
        int connectionCounter = 40;
        bool connected = false;
        while (connectionCounter>1 && !connected){
            connectionCounter--;
            usleep(1000000);
            connected = odometryModule->px4Commander.connectToPX4();
            SystemMessage s{};
            s.type = SystemMessage::PX_CONNECTION_STATUS;
            s.i[0] = connected ? 1: 0;
            s.i[1] = connectionCounter;
            sendMessage(s);
            std::cout<<connectionCounter<<" iterations remain to connect px4"<<std::endl;
        }
        //std::cout<<"Failed to connect to PX4"<<std::endl;
    });
    thr.detach();
}

#pragma clang diagnostic pop
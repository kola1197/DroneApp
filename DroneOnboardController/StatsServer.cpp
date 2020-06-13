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
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

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
    //std::cout<<"here2"<<std::endl;

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
        while (true) {
            HarbingerMessage h;
            char hmsg[sizeof (h)];
            int hbytes;
            for (int i = 0; i < sizeof(h); i += hbytes) {
                if ((hbytes = recv(sock, hmsg +i, sizeof(h)  - i, 0)) == -1)
                    std::cout<<"error"<<std::endl;
            }
            std::memcpy(&h,hmsg , sizeof(h));
            if (h.type == HarbingerMessage::MESSAGE_WITH_IMAGE)
            {
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
            if (h.type == HarbingerMessage::SYSTEM_MESSAGE)
            {
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
                    default:
                        break;
                }
            }
        }
    });
    thr.detach();
    camModule.startThread();
    int testcounter = 0;
    while (!stopThread())
    {
        if (imageSendMode.get()) {
            //cv::imwrite("testLeftGREYOut.jpg",*camModule.leftImage.getImage());
            sendImage(camModule.leftImage.getImage(), true);
            sendImage(camModule.rightImage.getImage(), false);
            testcounter++;
        }
        //std::cout<<"images sent "<<testcounter<<std::endl;
    }

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


//todo add mutex
bool StatsServer::stopThread()
{
    return stopBool;
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

void StatsServer::sendImage(std::shared_ptr<cv::Mat> image, bool left)
{

    MessageWithImage m{};
    counter++;
    m.height = image.get()->size().height;//*image.size().height;
    m.width = image->size().width;
    m.dataSize = image->total()*image->elemSize();
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
        m.imData[i] = image->data[i];
    }
    sendMessage(m);
}



void StatsServer::sendMessage(SystemMessage m)
{
    HarbingerMessage h;
    h.type = HarbingerMessage::SYSTEM_MESSAGE;
    h.code = 239;
    sendMutex.lock();
    send(sock, &h, sizeof(h), 0);
    //std::cout<<"sizeof m"<< sizeof(m)<<std::endl;
    send(sock, &m, sizeof(m), 0);
    sendMutex.unlock();
}

void StatsServer::sendMessage(MessageWithImage m)
{
    HarbingerMessage h;
    h.type = HarbingerMessage::MESSAGE_WITH_IMAGE;
    h.code = 239;
    sendMutex.lock();
    send(sock, &h, sizeof(h), 0);
    send(sock, &m, sizeof(m), 0);
    sendMutex.unlock();
}
#pragma clang diagnostic pop
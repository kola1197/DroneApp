//
// Created by nickolay on 30.05.2020.
//
#include <cstdio>
#include "StatsServer.h"
#include <cstring>    // sizeof()
#include <iostream>
#include <string>

// headers for socket(), getaddrinfo() and friends
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <unistd.h>    // close()
#include <thread>
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include "opencv2/core.hpp"


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
    valread = read( new_socket , buffer, 1024);
    printf("Got message: %s""\n",buffer );
}

void StatsServer::startServer(){

    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    char *hello = "Hello from server";

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
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                             (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    std::thread thr([this]()
    {
        std::cout<<"reading messages"<<std::endl;
        while (true) {
            char buffer[1024] = {0};
            valread = read(new_socket, buffer, 1024);
        }
    });
    thr.detach();
    camModule.startThread();
    int testcounter = 0;
    while (!stopThread())
    {
        std::cout<<"here"<<std::endl;
        sendImage(camModule.leftImage.getImage(), true);
        sendImage(camModule.rightImage.getImage(), false);
        testcounter++;
        std::cout<<"images sent "<<testcounter<<std::endl;
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

void StatsServer::sendImage(std::shared_ptr<cv::Mat> image, bool left)
{
    Message m;
    counter++;
    //std::cout<<"frame№"<<counter<<std::endl;
    m.height = image->size().height;
    m.width = image->size().width;
    m.dataSize = image->total()*image->elemSize();
    //std::cout<<"size"<<m.dataSize<<std::endl;
    std::string text = "";
    if (left)
    {
        text = "left image";
        m.type = Type::LEFT_IMAGE;
        cv::Mat lll(image->size(), CV_8UC3, image->data);
        //cv::imwrite("/home/nickolay/Code/DroneApp/LEFTPTR.jpg",*image.get());
        //cv::imwrite("/home/nickolay/Code/DroneApp/LEFT.jpg",lll);
        cv::imshow( "left", lll);
    } else{
        text = "right image";
        m.type = Type::RIGHT_IMAGE;
        cv::Mat lll(image->size(), CV_8UC3, image->data);
        cv::imshow( "right", lll);
    }
    for (int i=0;i<200;i++)
    {
        m.text[i]='0';
    }
    for (int i=0;i<text.length();i++)
    {
        m.text[i]=text.at(i);
    }
    for (int i =0;i< m.dataSize;i++)
    {
        m.imData[i] = image->data[i];
    }
    send(new_socket, &m, sizeof(m), 0);
}

void StatsServer::sendImage(cv::Mat image, bool left)
{
    Message m;
    counter++;
    //std::cout<<"frame№"<<counter<<std::endl;
    m.height = image.size().height;
    m.width = image.size().width;
    m.dataSize = image.total()*image.elemSize();
    //std::cout<<"size"<<m.dataSize<<std::endl;
    std::string text = "";
    if (left)
    {
        text = "left image";
        m.type = Type::LEFT_IMAGE;
    } else{
        text = "right image";
        m.type = Type::RIGHT_IMAGE;
    }
    for (int i=0;i<200;i++)
    {
        m.text[i]='0';
    }
    for (int i=0;i<text.length();i++)
    {
        m.text[i]=text.at(i);
    }
    for (int i =0;i< m.dataSize;i++)
    {
        m.imData[i] = image.data[i];
    }
    send(new_socket, &m, sizeof(m), 0);
}

void StatsServer::sendSystemMessage(std::string text)
{
    Message m;
    cv::Mat im = cv::imread("/home/nickolay/DroneStatsServer/testim.jpg", CV_LOAD_IMAGE_COLOR);
    for (int i=0;i<200;i++)
    {
        m.text[i]='0';
    }
    for (int i=0;i<text.length();i++)
    {
        m.text[i]=text.at(i);
    }
    m.i=239239;
    m.height = im.size().height;
    m.width = im.size().width;
    m.type = Type::SYSTEM_MESSAGE;
    m.dataSize = im.total()*im.elemSize();
    for (int i =0;i<200;i++)
    {
        std::cout<<m.text[i];
    }
    for (int i =0;i< m.dataSize;i++)
    {
        m.imData[i] = im.data[i];
    }
    send(new_socket, &m, sizeof(m), 0);
}

void StatsServer::sendMessage(Message *m)
{
    send(new_socket, &m, sizeof(*m), 0);
}
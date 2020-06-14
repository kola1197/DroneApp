#include "statsclient.h"
#include <sys/socket.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <QTimer>
#include <thread>
#include <opencv/cv.hpp>
#include <QtGui/QImage>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#define PORT 66666

StatsClient::StatsClient():QObject()
{

}

void StatsClient::connectToDroneServer(std::string ip)
{
    struct sockaddr_in serv_addr;
    char *hello = "Hello from client";
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
       printf("\n Socket creation error \n");
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr)<=0)
    {
       printf("\nInvalid address/ Address not supported \n");
    }
    if (::connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\nConnection Failed \n");
    }
    emit transmitConnectionStatus(true);
    closeConnectionThreadBool.set(false);
    //sendData("greetings");
    std::thread thr([this]()
    {
        std::cout<<"reading messages"<<std::endl;
        while (!closeConnectionThreadBool.get()) {
            HarbingerMessage h;
            char hmsg[sizeof (h)];
            int hbytes;
            for (int i = 0; i < sizeof(h); i += hbytes) {
                if ((hbytes = recv(sock, hmsg +i, sizeof(h)  - i, 0)) == -1){
                    std::cout<<"error"<<std::endl;
                    errorServerStop();
                }
            }
            std::memcpy(&h,hmsg , sizeof(h));
            if (h.type == HarbingerMessage::MESSAGE_WITH_IMAGE)
            {
                MessageWithImage m;
                char msg[sizeof (m)];
                int bytes;
                for (int i = 0; i < sizeof(m); i += bytes) {
                    if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1){
                        std::cout<<"error"<<std::endl;
                        errorServerStop();
                    }
                }
                std::memcpy(&m,msg , sizeof(m));
                if ( m.type == MessageWithImage::LEFT_IMAGE)
                {
                    uchar imdata [m.dataSize];
                    for (int i = 0;i<m.dataSize;i++)
                    {
                        imdata[i] = m.imData[i];
                    }
                    cv::Mat img1(cv::Size(m.width, m.height), CV_8UC3, imdata);
                    //cv::imwrite("../TESTGREY.jpg",img1);
                    QImage image = mat2RealQImage(img1);
                    //counter++;
                    //std::cout<<"counter = "<<counter<<std::endl;
                    emit transmit_to_left_image(image);
                    //cv::imshow("Left image", img1);
                }
                if ( m.type == MessageWithImage::RIGHT_IMAGE )
                {
                    uchar imdata [m.dataSize];
                    for (int i = 0;i<m.dataSize;i++)
                    {
                        imdata[i] = m.imData[i];
                    }
                    cv::Mat img1(cv::Size(m.width, m.height), CV_8UC3, imdata);
                    QImage image = mat2RealQImage(img1);
                    emit transmit_to_right_image(image);
                }
            }
            if (h.type == HarbingerMessage::SYSTEM_MESSAGE)
            {
                SystemMessage m;
                char msg[sizeof (m)];
                int bytes;
                for (int i = 0; i < sizeof(m); i += bytes) {
                    if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1){
                        std::cout<<"error"<<std::endl;
                        errorServerStop();
                    }
                }
                std::memcpy(&m,msg , sizeof(m));
                QString string ("");
                bool b;
                switch (m.type)
                {
                    case SystemMessage::TEXT_ALLERT:
                        string =  m.text;
                        emit transmit_to_gui(string);                                  //TODO change to "reactToSysCommand"
                        break;
                    case SystemMessage::VIDEO_STREAM_STATUS:
                        std::cout<<"VIDEO_STREAM_STATUS"<<std::endl;
                        b = m.i[0] == 1;
                        emit transmitVideoStreamStatus(b);
                        break;
                    case SystemMessage::VIDEO_CAPTURE_STATUS:
                        b = m.i[0] == 1;
                        emit transmitOnboardVideoCaptureStatus(b);
                        break;
                    default:
                        break;
                }
            }
        }
    });
    thr.detach();
}

void StatsClient::errorServerStop()
{
    closeConnectionThreadBool.set(true);
    transmitConnectionStatus(false);
    emit transmit_to_gui("Connection error");
}

QImage StatsClient::mat2RealQImage(cv::Mat const &src)     // B<->R
{
    QImage img = QImage((uchar*) src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
    return img.copy();
}



void StatsClient::sendMessage(SystemMessage m)
{
    HarbingerMessage h;
    h.type = HarbingerMessage::SYSTEM_MESSAGE;
    sendMutex.lock();
    send(sock, &h, sizeof(h), 0);
    send(sock, &m, sizeof(m), 0);
    sendMutex.unlock();
}

void StatsClient::sendMessage(MessageWithImage m)
{
    HarbingerMessage h;
    h.type = HarbingerMessage::MESSAGE_WITH_IMAGE;
    sendMutex.lock();
    send(sock, &h, sizeof(h), 0);
    send(sock, &m, sizeof(m), 0);
    sendMutex.unlock();
}

void StatsClient::sendAllert(std::string s)
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
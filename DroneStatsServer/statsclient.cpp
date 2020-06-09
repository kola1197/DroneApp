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

void StatsClient::connectToDroneServer()
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
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
    {
       printf("\nInvalid address/ Address not supported \n");
    }
    if (::connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\nConnection Failed \n");
    }
    //sendData("greetings");
    std::thread thr([this]()
    {
        std::cout<<"reading messages"<<std::endl;
        while (true) {
            Message m;
            char msg[sizeof (m)];
            int bytes;
            for (int i = 0; i < sizeof(m); i += bytes) {
                if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1)
                    std::cout<<"error"<<std::endl;
            }
            std::memcpy(&m,msg , sizeof(m));
            if ( m.type == Type::LEFT_IMAGE )
            {
                uchar imdata [m.dataSize];
                for (int i = 0;i<m.dataSize;i++)
                {
                    imdata[i] = m.imData[i];
                }
                cv::Mat img1(cv::Size(m.width, m.height), CV_8UC3, imdata);
                QImage image = mat2RealQImage(img1);
                //counter++;
                //std::cout<<"counter = "<<counter<<std::endl;
                emit transmit_to_left_image(image);
                //cv::imshow("Left image", img1);
            }
            if ( m.type == Type::RIGHT_IMAGE )
            {
                uchar imdata [m.dataSize];
                for (int i = 0;i<m.dataSize;i++)
                {
                    imdata[i] = m.imData[i];
                }
                cv::Mat img1(cv::Size(m.width, m.height), CV_8UC3, imdata);
                QImage image = mat2RealQImage(img1);
                //std::cout<<"right image"<<std::endl;
                emit transmit_to_right_image(image);
                //cv::imshow("Right image", img1);
            }
            if ( m.type == Type::SYSTEM_MESSAGE )
            {
                QString string (m.text);
                emit transmit_to_gui(string);
            }
        }
    });
    thr.detach();
}

QImage StatsClient::mat2RealQImage(cv::Mat const &src)     // B<->R
{
    QImage img = QImage((uchar*) src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
    //QImage res = img.convertToFormat(QImage::Format_RGB32);
    return img.copy();
}

void StatsClient::sendData(char *data)
{
    send(sock , data , strlen(data) , 0 );
}


#pragma clang diagnostic pop
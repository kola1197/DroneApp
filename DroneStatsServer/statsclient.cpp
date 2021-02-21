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
    bool failed = false;
    struct sockaddr_in serv_addr;
    char *hello = "Hello from client";
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
       printf("\n Socket creation error \n");
       failed = true;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr)<=0)
    {
       printf("\nInvalid address/ Address not supported \n");
       failed = true;
    }
    if (::connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\nConnection Failed \n");
       failed = true;
    }
    connected.set(!failed);
    emit transmitConnectionStatus(true);
    closeConnectionThreadBool.set(false);
    //sendData("greetings");
    std::thread thr([this]()
    {
        std::cout<<"reading messages"<<std::endl;
        while (!closeConnectionThreadBool.get()) {
            getMessage();
        }
    });
    thr.detach();
    QTimer::singleShot(100,this, SLOT(sendPingRequest()));
}

void StatsClient::getMessage(){
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

void StatsClient::getSettingsMessage(){
    SettingsMessage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1)
            std::cout<<"error"<<std::endl;
    }
    std::memcpy(&m,msg , sizeof(m));
}

void StatsClient::getCommandMessage(){
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
            vehicleData.targetpoint.set(CvPoint3D32f{m.f[0],m.f[1],m.f[2]});
            emit(transmitTargetpointUpdated());
            break;
        default:
            break;

    }
}


void StatsClient::getPingMessage()
{
    PingMessage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1){
            std::cout<<"error"<<std::endl;
            errorServerStop();
        }
    }
    std::memcpy(&m,msg , sizeof(m));
    int64 now =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    //std::cout<<"ping: "<<m.time[0]<<"-_-"<<m.time[1]<<"-_-"<<now<<std::endl;
    QString q = "FPS: " + QString::number(fps)+"  \nPING(milliseconds):\nTo Raspberry: " + QString::number(m.time[1] - m.time[0]) + "\n" + "From Raspberry: " + QString::number(now - m.time[1]);
    emit transmitPing( q);
}

void StatsClient::getMessageWithColorImage()
{
    //std::cout<<"got color image"<<std::endl;
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
        QImage image = mat2RealQImage(img1, false);
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
        QImage image = mat2RealQImage(img1, false);
        emit transmit_to_right_image(image);
    }
}


void StatsClient::getMessageWithGrayImage()
{
    std::cout<<"got gray"<<std::endl;
    MessageWithGrayImage m;
    char msg[sizeof (m)];
    int bytes;
    for (int i = 0; i < sizeof(m); i += bytes) {
        if ((bytes = recv(sock, msg +i, sizeof(m)  - i, 0)) == -1){
            std::cout<<"error"<<std::endl;
            errorServerStop();
        }
    }
    std::memcpy(&m,msg , sizeof(m));
    if ( m.type == MessageWithGrayImage::LEFT_IMAGE)
    {
        uchar imdata [m.dataSize];
        for (int i = 0;i<m.dataSize;i++)
        {
            imdata[i] = m.imData[i];
        }
        cv::Mat img1(cv::Size(m.width, m.height), CV_8UC1, imdata);
        //std::cout<<"GOT CREY"<<std::endl;
        //cv::imwrite("../TESTGREY.jpg",img1);
        QImage image = mat2RealQImage(img1, true);
        //counter++;
        //std::cout<<"counter = "<<counter<<std::endl;
        emit transmit_to_left_image(image);
        //cv::imshow("Left image", img1);
    }
    if ( m.type == MessageWithGrayImage::RIGHT_IMAGE )
    {
        uchar imdata [m.dataSize];
        for (int i = 0;i<m.dataSize;i++)
        {
            imdata[i] = m.imData[i];
        }
        cv::Mat img1(cv::Size(m.width, m.height), CV_8UC1, imdata);
        cv::imwrite("../RightGREY.jpg",img1);
        QImage image = mat2RealQImage(img1, true);
        emit transmit_to_right_image(image);
    }
}

void StatsClient::getSystemMessage()
{
    SystemMessage m{};
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
            emit transmit_to_gui(string);
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
        case SystemMessage::FPS_COUNTER:
            fps = m.i[0];
            break;
        case SystemMessage::COORDINATES:
            //CvPoint3D32f point(m.f[0],m.f[1],m.f[2]);
            emit transmitCoordinates(CvPoint3D32f(m.f[0],m.f[1],m.f[2]));
            //std::cout<<"Point x:"<<m.f[0]<<" y:"<<m.f[1]<<" z:"<<m.f[2]<<std::endl;
            break;
        default:
            break;
    }
}

void StatsClient::sendPingRequest()
{
    //std::cout<<"PING"<<std::endl;
    PingMessage m{};
    m.time[0] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    sendMessage(m);
    QTimer::singleShot(1000,this, SLOT(sendPingRequest()));
}

void StatsClient::errorServerStop()
{
    closeConnectionThreadBool.set(true);
    connected.set(false);
    transmitConnectionStatus(false);
    emit transmit_to_gui("Connection error");
}

QImage StatsClient::mat2RealQImage(cv::Mat const &src, bool isGray)     // B<->R
{
    QImage img;
    if (isGray) {
        img = QImage((uchar *) src.data, src.cols, src.rows, src.step, QImage::Format_Grayscale8);
    } else {
        img = QImage((uchar *) src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
    }
    return img.copy();
}



/*void StatsClient::sendMessage(SystemMessage m)
{
    HarbingerMessage h;
    h.type = SYSTEM_MESSAGE;
    sendMutex.lock();
    send(sock, &h, sizeof(h), 0);
    send(sock, &m, sizeof(m), 0);
    sendMutex.unlock();
}*/

/*void StatsClient::sendMessage(MessageWithImage m)
{
    HarbingerMessage h;
    h.type = MESSAGE_WITH_IMAGE;
    sendMutex.lock();
    send(sock, &h, sizeof(h), 0);
    send(sock, &m, sizeof(m), 0);
    sendMutex.unlock();
}
*/
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

/*void StatsClient::sendMessage(PingMessage m)
{
    HarbingerMessage h;
    h.type = PING_MESSAGE;
    h.code = 239;
    sendMutex.lock();
    send(sock, &h, sizeof(h), 0);
    send(sock, &m, sizeof(m), 0);
    sendMutex.unlock();
}*/
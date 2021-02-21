//
// Created by nickolay on 30.05.2020.
//

#ifndef DRONEONBOARDCONTROLLER_STATSSERVER_H
#define DRONEONBOARDCONTROLLER_STATSSERVER_H

#include "sys/socket.h"
#include "opencv2/core.hpp"
#include "CameraModule.h"
#include "../Utils/Messages.h"
#include "OdometryModule.h"

/*enum MessageType { LEFT_IMAGE, RIGHT_IMAGE, SYSTEM_MESSAGE, EMPTY };

struct Message {
    char text[200];
    int i;
    int height;
    int width;
    int dataSize;
    //cv::Mat image;
    uchar imData [240400];//230400   //2352000
    MessageType type;
};*/

class StatsServer {
public :
    StatsServer();
    ~StatsServer();

    void startServer();
    AsyncVar<bool> serverStop{false};
    void readVoid();

    template <typename T>
    void sendMessage(T t){                       //in header because of stupid gcc compilation
        HarbingerMessage h{};
        std::string type = typeid(t).name();
        if (Messages::getMessageTypeByName(type, &h.type)) //HarbingerMessage::PING_MESSAGE;
        {
            /*if (h.type == DEBUG_MESSAGE)
            {
                //t.checksum;
                //sim::sout<<"Sending DebugMessage from "<<t.from<<" to "<<t.to<<" checksum = "<<t.checksum<<sim::endl;
                if (t.checksum != 239239239)
                {
                    sim::sout<<"Sending BROKEN DebugMessage from "<<t.from<<" to "<<t.to<<" checksum = "<<t.checksum<<sim::endl;
                }
            }*/
            sendMutex.lock();
            char hData[sizeof(h)];
            memcpy(hData, &h, sizeof(h));
            send(sock, &hData, sizeof(h), 0);
            //sim::sout<<"sizeof m"<< sizeof(m)<<sim::endl;
            char mData[sizeof(t)];
            memcpy(mData, &t, sizeof(t));
            send(sock, &mData, sizeof(t), 0);
            sendMutex.unlock();
        }
    }
private:
    OdometryModule* odometryModule;
    CameraModule camModule;
    bool stopBool = false;
    int server_fd, sock, valread;
    bool stopThread();
    int counter = 0;
    MutexBool imageSendMode {false};
    std::mutex sendMutex;
    void sendImage(std::shared_ptr<cv::Mat> image, bool left, bool isGrey);
    /*void sendMessage(SystemMessage m);
    void sendMessage(MessageWithImage m);
    void sendMessage(MessageWithGrayImage m);*/
    void sendAllert(std::string);
    int GetCPULoad();
    //void sendMessage(PingMessage m);
    void getMessage();
    std::chrono::microseconds lastDataUpdateForGroundStation;
    void getSystemMessage();

    void getMessageWithGrayImage();

    void getMessageWithColorImage();

    void getPingMessage();

    void getCommandMessage();
    void getSettingsMessage();

    void sendCoordinates();

    void updateTargetPointForGroundStation();

    void updateDataForGroundStation();

    void updateVehicleStatusForGroundStation();

    void UpdateSettingsForGroundStation();
};


#endif //DRONEONBOARDCONTROLLER_STATSSERVER_H

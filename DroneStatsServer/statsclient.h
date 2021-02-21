#ifndef STATSCLIENT_H
#define STATSCLIENT_H

#include <opencv2/core/mat.hpp>
#include <mutex>
#include "QObject"
#include <sys/socket.h>
#include <cv.h>

#include "../Utils/Messages.h"
#include "../Utils/MutexBool.h"
#include "VehicleData.h"

/*
enum MessageType { LEFT_IMAGE, RIGHT_IMAGE, SYSTEM_MESSAGE, EMPTY };

struct Message {
    char text[200];
    int i;
    int height;
    int width;
    int dataSize;
    //cv::Mat image;
    uchar imData [240400];//230400   //2352000
    MessageType type;
};
*/

class StatsClient : public QObject
{
    Q_OBJECT


public:
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
    VehicleData vehicleData;
    StatsClient();
    void connectToDroneServer(std::string ip);
    //void sendMessage(MessageWithImage m);
    //void sendMessage(SystemMessage m);
    AsyncVar<bool> connected{false};
    MutexBool closeConnectionThreadBool{false};              //on true closes connection
signals:
    void transmit_to_gui(QString value);
    void transmit_to_left_image(QImage value);
    void transmit_to_right_image(QImage value);
    void transmitOnboardVideoCaptureStatus(bool mode);
    void transmitVideoStreamStatus(bool mode);
    void transmitConnectionStatus(bool connected);
    void transmitCoordinates(CvPoint3D32f point);
    void transmitPing(QString q);
    void transmitTargetpointUpdated();
private slots:
    void sendPingRequest();
private:


    int sock = 0;
    QImage mat2RealQImage(cv::Mat const &src, bool isGray);
    int counter = 0;
    std::mutex sendMutex;
    void sendAllert(std::string s);
    void errorServerStop();
    //void sendMessage(PingMessage m);
    double fps = 0;

    void getMessage();

    void getSystemMessage();

    void getMessageWithGrayImage();
    void getMessageWithColorImage();
    void getPingMessage();

    void getCommandMessage();
    void getSettingsMessage();
};

#endif // STATSCLIENT_H

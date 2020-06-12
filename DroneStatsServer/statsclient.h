#ifndef STATSCLIENT_H
#define STATSCLIENT_H

#include <opencv2/core/mat.hpp>
#include <mutex>
#include "QObject"
#include "../Utils/Messages.h"

/*
enum Type { LEFT_IMAGE, RIGHT_IMAGE, SYSTEM_MESSAGE, EMPTY };

struct Message {
    char text[200];
    int i;
    int height;
    int width;
    int dataSize;
    //cv::Mat image;
    uchar imData [240400];//230400   //2352000
    Type type;
};
*/

class StatsClient : public QObject
{
    Q_OBJECT
public:
    StatsClient();
    void connectToDroneServer();
    void sendMessage(MessageWithImage m);
    void sendMessage(SystemMessage m);

private:
    int sock = 0;
    QImage mat2RealQImage(const cv::Mat &src);
    int counter = 0;
    bool connected = false;
    std::mutex sendMutex;
    void sendAllert(std::string s);

signals:
    void transmit_to_gui(QString value);
    void transmit_to_left_image(QImage value);
    void transmit_to_right_image(QImage value);
    void transmitOnboardVideoCaptureStatus(bool mode);
    void transmitVideoStreamStatus(bool mode);

};

#endif // STATSCLIENT_H

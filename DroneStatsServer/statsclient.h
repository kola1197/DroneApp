#ifndef STATSCLIENT_H
#define STATSCLIENT_H

#include <opencv2/core/mat.hpp>
#include "QObject"

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


class StatsClient : public QObject
{
    Q_OBJECT
public:
    StatsClient();
    void connectToDroneServer();
private:
    int sock = 0;
    QImage mat2RealQImage(const cv::Mat &src);
    int counter = 0;
    void sendData(char *data);

signals:
    void transmit_to_gui(QString value);
    void transmit_to_left_image(QImage value);
    void transmit_to_right_image(QImage value);

};

#endif // STATSCLIENT_H

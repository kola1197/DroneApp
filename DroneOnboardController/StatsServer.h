//
// Created by nickolay on 30.05.2020.
//

#ifndef DRONEONBOARDCONTROLLER_STATSSERVER_H
#define DRONEONBOARDCONTROLLER_STATSSERVER_H

#include "opencv2/core.hpp"
#include "CameraModule.h"
#include "../Utils/Messages.h"

/*enum Type { LEFT_IMAGE, RIGHT_IMAGE, SYSTEM_MESSAGE, EMPTY };

struct Message {
    char text[200];
    int i;
    int height;
    int width;
    int dataSize;
    //cv::Mat image;
    uchar imData [240400];//230400   //2352000
    Type type;
};*/

class StatsServer {
public :
    StatsServer();
    ~StatsServer();

    void startServer();

    void readVoid();


private:
    CameraModule camModule;
    bool stopBool = false;
    int server_fd, sock, valread;
    bool stopThread();
    int counter = 0;
    MutexBool imageSendMode {false};
    void sendMessage(SystemMessage m);
    std::mutex sendMutex;
    void sendImage(std::shared_ptr<cv::Mat> image, bool left);
    void sendMessage(MessageWithImage m);
    void sendAllert(std::string);
    int GetCPULoad();
};


#endif //DRONEONBOARDCONTROLLER_STATSSERVER_H

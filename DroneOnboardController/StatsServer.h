//
// Created by nickolay on 30.05.2020.
//

#ifndef DRONEONBOARDCONTROLLER_STATSSERVER_H
#define DRONEONBOARDCONTROLLER_STATSSERVER_H

#include "opencv2/core.hpp"

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

class StatsServer {
public :
    StatsServer();
    ~StatsServer();

    void startServer();

    void readVoid();


private:
    bool stopBool = false;
    int server_fd, new_socket, valread;
    bool stopThread();

    void sendSystemMessage(std::string text);
    int counter = 0;
    void sendImage(cv::Mat image, bool left);

    void sendMessage(Message *m);
};


#endif //DRONEONBOARDCONTROLLER_STATSSERVER_H

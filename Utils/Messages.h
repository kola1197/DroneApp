//
// Created by nickolay on 10.06.2020.
//

#ifndef DRONEAPP_MESSAGES_H
#define DRONEAPP_MESSAGES_H


#include <opencv2/core/hal/interface.h>

//enum Type { LEFT_IMAGE, RIGHT_IMAGE, SYSTEM_MESSAGE, EMPTY, SET_IMAGE_SEND_MODE };

struct HarbingerMessage {                                           //sends before other messages, to set resiver to it
    enum Type {TEST_MESSAGE, MESSAGE_WITH_IMAGE, SYSTEM_MESSAGE};
    Type type;
    int code;
};

struct MessageWithImage {                                           //get image from vehicle
    enum Type {LEFT_IMAGE, RIGHT_IMAGE};
    char text[200];
    int i;
    int height;
    int width;
    int dataSize;
    //cv::Mat image;
    uchar imData [240400];                                                  //230400   //2352000
    Type type;
};

struct SystemMessage {
    enum Type {START_VIDEO_STREAM, VIDEO_STREAM_STATUS, VIDEO_CAPTURE_STATUS, START_IMAGE_CAPTURE, STOP_IMAGE_CAPTURE, STOP_VIDEO_STREAM, TEXT_ALLERT};
    char text[200];
    int i[8];
    Type type;
};

class Messages {

};


#endif //DRONEAPP_MESSAGES_H

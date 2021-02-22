//
// Created by nickolay on 10.06.2020.
//

#ifndef DRONEAPP_MESSAGES_H
#define DRONEAPP_MESSAGES_H


#include <opencv2/core/hal/interface.h>
#include <string>

//enum MessageType { LEFT_IMAGE, RIGHT_IMAGE, SYSTEM_MESSAGE, EMPTY, SET_IMAGE_SEND_MODE };
enum MessageType {TEST_MESSAGE, MESSAGE_WITH_IMAGE, MESSAGE_WITH_GRAY_IMAGE, SYSTEM_MESSAGE, PING_MESSAGE, COMMAND_MESSAGE, SETTINGS_MESSAGE};

struct HarbingerMessage {                                           //sends before other messages, to set resiver to it
    //enum MessageType {TEST_MESSAGE, MESSAGE_WITH_IMAGE, MESSAGE_WITH_GRAY_IMAGE, SYSTEM_MESSAGE, PING_MESSAGE};
    MessageType type;
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

struct MessageWithGrayImage {                                           //get image from vehicle
    enum Type {LEFT_IMAGE, RIGHT_IMAGE};
    char text[200];
    int i;
    int height;
    int width;
    int dataSize;
    //cv::Mat image;
    uchar imData [100000];                                               //100000   //230400   //2352000
    Type type;
};

struct SystemMessage {
    enum Type {START_VIDEO_STREAM, VIDEO_STREAM_STATUS, VIDEO_CAPTURE_STATUS, START_IMAGE_CAPTURE, STOP_IMAGE_CAPTURE,
            STOP_VIDEO_STREAM, TEXT_ALLERT, FPS_COUNTER, COORDINATES, CONNECT_TO_PX, PX_CONNECTION_STATUS};
    char text[200];
    int i[8];
    float f[3];
    Type type;
};

struct PingMessage{
    int64 time[2];
};

struct CommandMessage{
    enum Type{SET_TARGET, START, SET_THIS_POINT_AS_ZERO};
    Type type;
    char text[200];
    int i[9];
    float f[9];
};

struct SettingsMessage{
    enum Type{VEHICLE_MODE};
    Type type;
    char text[200];
    int i[9];
};

class Messages {
public:
    static bool getMessageTypeByName(std::string name,MessageType * type) {
        if (name == typeid(SystemMessage).name())
        {
            *type = MessageType::SYSTEM_MESSAGE;
            return true;
        }
        if (name == typeid(MessageWithImage).name())
        {
            *type = MessageType::MESSAGE_WITH_IMAGE;
            return true;
        }
        if (name == typeid(MessageWithGrayImage).name())
        {
            *type = MessageType::MESSAGE_WITH_GRAY_IMAGE;
            return true;
        }
        if (name == typeid(PingMessage).name())
        {
            *type = MessageType::PING_MESSAGE;
            return true;
        }
        if (name == typeid(SettingsMessage).name())
        {
            *type = MessageType::SETTINGS_MESSAGE;
            return true;
        }
        if (name == typeid(CommandMessage).name())
        {
            *type = MessageType::COMMAND_MESSAGE;
            return true;
        }
        /*if (name == typeid(TestMessage).name())
        {
            *type = MessageType::TEST_MESSAGE;
            return true;
        }*/
        return false;
    }
};


#endif //DRONEAPP_MESSAGES_H

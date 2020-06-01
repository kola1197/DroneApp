//
// Created by nickolay on 30.05.2020.
//

#include "Message.h"
;;

Message::Message(cv::Mat im, Type t)
{
    type = t;
    image = im;
    text = "null";
}

Message::Message(std::string t)
{
    type = Type::SYSTEM_MESSAGE;
    text = t;
}

Message::Message()
{
    type = Type::EMPTY;
    text = "null";
}

Message::~Message()
{

}

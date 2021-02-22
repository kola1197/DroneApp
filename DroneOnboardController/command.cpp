#include "command.h"
#include <QLatin1String>
#include <iostream>

const char *Command::getName(Command::ChannelID value)
{
    switch (value)
    {
    case CHANNEL_YAW : return "CHANNEL_YAW"; break ;
    case CHANNEL_ROLL : return "CHANNEL_ROLL"; break ;
    case CHANNEL_PITCH : return "CHANNEL_PITCH"; break ;
    case CHANNEL_THROTTLE : return "CHANNEL_THROTTLE"; break ;
    case CHANNEL_4 : return "CHANNEL_4"; break ;
    case CHANNEL_5 : return "CHANNEL_5"; break ;
    case CHANNEL_6 : return "CHANNEL_6"; break ;
    case CHANNEL_7 : return "CHANNEL_7"; break ;
    default : return "Not in range"; break ;

    }
    return "Not in range";
}

void Command::print()
{
    for (int i = 0; i < CHANNEL_CONTROL_LAST; i++)
    {
        std::cout<<(QLatin1String("%s %d\n"), getName((ChannelID)i), axis[i])<<std::endl;
    }
}

std::string Command::getDataToSend()
{
    std::string result;
    for (int i = 0; i < CHANNEL_CONTROL_LAST; i++)
    {
        result+='_';
        result+=std::to_string(axis[i]);
    }

    return result;
}

Command::Command()
{
    axis[CHANNEL_YAW]      = 1500;
    axis[CHANNEL_ROLL]     = 1500;
    axis[CHANNEL_PITCH]    = 1500;
    axis[CHANNEL_THROTTLE] = 1100;

    axis[CHANNEL_4] = 900;
    axis[CHANNEL_5] = 1500;
    axis[CHANNEL_6] = 1500;
    axis[CHANNEL_7] = 1500;
}

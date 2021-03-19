#ifndef COMMANDINPUT_H
#define COMMANDINPUT_H

#include "joystickinput.h"
#include "command.h"



class CommandInput
{
public:
    CommandInput();
    Command getCommand();
    Command failSafe = Command();
    void setCurrentSendMode(int i);
    int getCurrentSendMode();
    void Stop();
private:
    //Command output = Command();
    int currentSendMode = 1;   // 1 - joystick, 0 - failsafe, -1 - AI
    JoyStickInput jsInput;
};


#endif // COMMANDINPUT_H

#include "commandinput.h"
#include "command.h"

CommandInput::CommandInput()
{
    jsInput.setRTLTFullMode();
//    std::thread th([&]()
//                   {
//                       jsInput.start();
//                   });
    //std::this_thread::sleep_for(std::chrono::seconds(10));
    jsInput.start();
    //th.join();
    std::cout<<"started jsInput"<<std::endl;
}

Command CommandInput::getCommand()
{

    Command result = failSafe;
    if (currentSendMode == 1)
    {
        result = jsInput.getOutput();
    }
    if (currentSendMode == -1)
    {
        //result = jsInput.getOutput();
    }
    return result;
}

void CommandInput::setCurrentSendMode(int i)
{
    if ( i * i < 2 )
    {
        currentSendMode = i;
    }
}

void CommandInput::Stop()
{
    jsInput.stop();
}

int CommandInput::getCurrentSendMode()
{
    return currentSendMode;
}

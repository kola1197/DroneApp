//
// Created by nickolay on 02.04.2020.
//

#ifndef AUTOPILOT_STOPPABLETHREAD_H
#define AUTOPILOT_STOPPABLETHREAD_H


#include <thread>
#include <iostream>
#include <assert.h>
#include <chrono>
#include <future>

/*
 * Class that encapsulates promise and future object and
 * provides API to set exit signal for the thread
 */
class StoppableThread
{
    std::promise<void> exitSignal;
    std::future<void> futureObj;
public:
    StoppableThread() :
            futureObj(exitSignal.get_future())
    {

    }
    StoppableThread(StoppableThread && obj) : exitSignal(std::move(obj.exitSignal)), futureObj(std::move(obj.futureObj))
    {
        std::cout << "Move Constructor is called" << std::endl;
    }

    StoppableThread & operator=(StoppableThread && obj)
    {
        std::cout << "Move Assignment is called" << std::endl;
        exitSignal = std::move(obj.exitSignal);
        futureObj = std::move(obj.futureObj);
        return *this;
    }

    // Task need to provide defination  for this function
    // It will be called by thread function
    virtual void run() = 0;


    // Thread function to be executed by thread
    void operator()()
    {
        run();
    }

    //Checks if thread is requested to stop
    bool stopRequested()
    {
        // checks if value in future object is available
        return !(futureObj.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout);
    }
    // Request the thread to stop by setting value in promise object
    void stop()
    {
        exitSignal.set_value();
    }
};
#endif //AUTOPILOT_STOPPABLETHREAD_H

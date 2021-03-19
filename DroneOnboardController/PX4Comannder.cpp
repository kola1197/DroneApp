//
// Created by nickolay on 21.02.2021.
//

#include <string>
#include "PX4Comannder.h"
#include <unistd.h>
#include <iostream>
#include <thread>
#include <pthread.h>
#include <wait.h>
#include "MAVConnector.h"

PX4Comannder::PX4Comannder(){}

void PX4Comannder::startDronekit()
{
    std::thread thr([this]() {
    pid_t child_pid;

    /* Duplicate this process.  */
    child_pid = fork ();

    if (child_pid != 0){
        /* This is the parent process.  */

        int ret = waitpid(child_pid, NULL, 0);

        if (ret == -1){
            printf ("an error occurred in waitpid\n");
            abort ();
        }
    }
    else {
        std::cout << "turning on drone kit" << std::endl;
        char *argv[2];
        argv[0] = "python3";
        argv[1] = "../Python/Main.py";//"/home/nickolay/Code/DroneApp/DroneOnboardController/Python/Main.py";
        argv[2] = NULL;

        execvp("python3", argv);        /* The execvp function returns only if an error occurs.  */
        //printf ("an error occurred in execl\n");
        //abort ();
    }
    });
    thr.detach();
    //std::cout<<"detached"<<std::endl;
}

bool PX4Comannder::connectToPX4()
{
    bool result = mavConnector.ConnectToServer();
    connected.set(result);
    return result;
}

void PX4Comannder::sendCommnads(int thr, int pitch, int roll, int yaw)
{
    std::string data = commandInput.getCommand().getDataToSend();
    char char_array[data.length() + 1];
    strcpy(char_array, data.c_str());
    mavConnector.SendData(char_array);
}

//
// Created by nickolay on 21.02.2021.
//

#include <string>
#include "PX4Comannder.h"
#include <unistd.h>
#include <iostream>
#include "MAVConnector.h"

PX4Comannder::PX4Comannder(){}

void PX4Comannder::startDronekit()
{
    std::cout<<"turning on drone kit"<<std::endl;
    std::string path = "/usr/bin/python";
    std::string script = "/home/user/script.py";
    std::string arg = "arg";
    char *argv [2];
    execvp("/home/nickolay/Code/DroneApp/DroneOnboardController/Python/main.py",argv);
    //std::cerr<<"cannot exec"<<endl;
}

bool PX4Comannder::connectToPX4()
{
    mavConnector.ConnectToServer();
}
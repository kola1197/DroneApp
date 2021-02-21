//
// Created by nickolay on 21.02.2021.
//

#include <string>
#include "PX4Comannder.h"
#include <unistd.h>

void PX4Comannder::startDronekit()
{
    std::string path = "/usr/bin/python";
    std::string script = "/home/user/script.py";
    std::string arg = "arg";
    char *argv [2];
    execvp("home/pi/dronekit.py",argv);
    //std::cerr<<"cannot exec"<<endl;
}

void PX4Comannder::connectToPX4()
{

}
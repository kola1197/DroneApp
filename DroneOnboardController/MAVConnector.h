//
// Created by nickolay on 12.04.2020.
//

#ifndef AUTOPILOT_MAVCONNECTOR_H
#define AUTOPILOT_MAVCONNECTOR_H
#include <python2.7/Python.h>


class MAVConnector {
public:
    MAVConnector();
    void test();
    int sock = 0;
    bool ConnectToServer();

    void SendData(char *data);
};


#endif //AUTOPILOT_MAVCONNECTOR_H

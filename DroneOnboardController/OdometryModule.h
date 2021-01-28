//
// Created by nickolay on 25.01.2021.
//

#ifndef DRONEAPP_ODOMETRYMODULE_H
#define DRONEAPP_ODOMETRYMODULE_H

#include "StatsServer.h"
#include "../Utils/AsyncVar.h"

class OdometryModule {
public:
    OdometryModule(CameraModule* _camModule);
    OdometryModule();
    void startThread();
private:
    CameraModule* camModule;
    AsyncVar<bool> threadActive{true};
    void updateCoordinats();

};


#endif //DRONEAPP_ODOMETRYMODULE_H

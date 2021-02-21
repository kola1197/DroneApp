//
// Created by nickolay on 21.02.2021.
//

#ifndef DRONEAPP_PX4COMANNDER_H
#define DRONEAPP_PX4COMANNDER_H
#include "../Utils/AsyncVar.h"
enum VehicleMode{QUADROCOPTER = 0, CAR = 1};

class PX4Comannder {
public:
    PX4Comannder();
    void startDronekit();
    AsyncVar<VehicleMode> vehicleMode{QUADROCOPTER};
private:
    void connectToPX4();
};


#endif //DRONEAPP_PX4COMANNDER_H

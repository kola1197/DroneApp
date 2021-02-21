//
// Created by nickolay on 21.02.2021.
//

#ifndef DRONEAPP_VEHICLEDATA_H
#define DRONEAPP_VEHICLEDATA_H


#include <cv.h>
#include "../Utils/AsyncVar.h"
enum VehicleMode{QUADROCOPTER = 0, CAR = 1};

class VehicleData {
public:
    AsyncVar<CvPoint3D32f> targetpoint{CvPoint3D32f{0,0,0}};
    AsyncVar<VehicleMode> vehicleMode{QUADROCOPTER};
};


#endif //DRONEAPP_VEHICLEDATA_H

//
// Created by nickolay on 21.02.2021.
//

#ifndef DRONEAPP_VEHICLEDATA_H
#define DRONEAPP_VEHICLEDATA_H


#include <cv.h>
#include "../Utils/AsyncVar.h"

class VehicleData {
public:
    AsyncVar<CvPoint3D32f> targetpoint{CvPoint3D32f{0,0,0}};
};


#endif //DRONEAPP_VEHICLEDATA_H

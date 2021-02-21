//
// Created by nickolay on 21.02.2021.
//

#ifndef DRONEAPP_PX4COMANNDER_H
#define DRONEAPP_PX4COMANNDER_H


class PX4Comannder {
public:
    void startDronekit();

private:
    void connectToPX4();
};


#endif //DRONEAPP_PX4COMANNDER_H

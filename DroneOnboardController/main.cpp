#include <iostream>
//#include <QApplication>
#include "StatsServer.h"
#include "depthmapcalibrator.h"

/*int calibDM(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DepthMapCalibrator dm;
    dm.show();
    return a.exec();
}*/

int main(int argc, char *argv[]) {
    //calibDM(argc, argv);
    StatsServer server;
    server.startServer();
    return 0;
}

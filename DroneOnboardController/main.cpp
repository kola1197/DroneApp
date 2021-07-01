#include <iostream>
#include <fstream>
#include <limits>

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

void odometryParametrsTest(){
    std::ofstream fout;
    double minDist = 8888888;
    double minI;
    bool minThreshold = true;
    double minPlanarDist = 8888888;
    double minPlanarI;
    bool minPlanarThreshold = true;
    typedef std::numeric_limits< double > dbl;

    fout.precision(dbl::max_digits10);
    fout.open("Odometry_test.txt",std::ios_base::out);
    for (int i=23;i<24;i++) {
        std::cout<<" fast_threshold: "<<i<<" nonmaxSupression: true"<<std::endl;
        StatsServer server;
        std::chrono::microseconds startTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
        std::vector<double> v = server.odometryTest(i, true, &fout);
        std::chrono::microseconds endTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
        double deltaTime = (endTime - startTime).count();
        double planarDist = sqrt(v[3] * v[3] + v[5] * v[5]);
        fout << " fast_threshold: "<<v[0]<<" nonmaxSupression: " <<v[1]<<"\n";
        fout << "Distance - " << v[2]<<" Planar dist - "<<planarDist<<" Time: "<<deltaTime<<"\n";
        fout << "Coordinates x: "<<v[3]<<" y: "<<v[4]<<" z: "<<v[5]<<"\n";
        fout << "\n";
        if (minDist > v[2]){
            minDist = v[2];
            minThreshold = true;
            minI = i;
        }
        if (minPlanarDist > planarDist){
            minPlanarDist = planarDist;
            minPlanarThreshold = true;
            minPlanarI = i;
        }
    }
    /*for (int i=10;i<36;i++) {
        std::cout<<" fast_threshold: "<<i<<" nonmaxSupression: false"<<std::endl;
        StatsServer server;
        std::chrono::microseconds startTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
        std::vector<double> v = server.odometryTest(i, false, &fout);
        std::chrono::microseconds endTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());       // timeShot[0]
        double deltaTime = (endTime - startTime).count();
        double planarDist = sqrt(v[3] * v[3] + v[5] * v[5]);
        fout << " fast_threshold: "<<v[0]<<" nonmaxSupression: " <<v[1]<<"\n";
        fout << "Distance - " << v[2]<<" Planar dist - "<<planarDist<<" Time: "<<deltaTime<<"\n";
        fout << "Coordinates x: "<<v[3]<<" y: "<<v[4]<<" z: "<<v[5]<<"\n";
        fout << "\n";
        if (minDist > v[2]){
            minDist = v[2];
            minThreshold = false;
            minI = i;
        }
        if (minPlanarDist > planarDist){
            minPlanarDist = planarDist;
            minPlanarThreshold = false;
            minPlanarI = i;
        }
    }*/
    fout << "  ----------------------------  ";
    fout << "\n";
    fout << "Min distance - "<< minDist <<" Threshold: "<<minI<<"\n";
    fout << "Min planar distance - "<< minPlanarDist <<" Threshold: "<<minPlanarI<<"\n";
    std::cout<<"Min distance - "<< minDist <<" Threshold: "<<minI<<"\n";
    std::cout<< "Min planar distance - "<< minPlanarDist <<" Threshold: "<<minPlanarI<<"\n";
    fout.close();
}

void ramTest()
{
    unsigned long memSize = 0;
    unsigned long memFree = 0;
    std::string token;
    std::ifstream file("/proc/meminfo");
    int counter = 0;
    while(file >> token) {
        if(token == "MemTotal:") {
            unsigned long mem;
            if(file >> mem) {
                memSize = mem;
            }
            counter++;
        }
        if(token == "MemAvailable:") {
            unsigned long mem;
            if(file >> mem) {
                memFree = mem;
            }
            counter++;
        }
        // ignore rest of the line
        if (counter==2) {
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    std::cout<<"MemSize: "<<memSize<<" MemFree: "<<memFree<<std::endl;
}

int main(int argc, char *argv[]) {
    //calibDM(argc, argv);
    //odometryParametrsTest();

    //ramUsageCheck();

    StatsServer server;
    server.startServer();
    return 0;
}

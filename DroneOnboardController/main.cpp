#include <iostream>
#include "StatsServer.h"

int main() {
    std::cout<<"start"<<std::endl;
    StatsServer server;
    server.startServer();
    return 0;
}

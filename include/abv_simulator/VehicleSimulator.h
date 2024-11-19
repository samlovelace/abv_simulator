#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include "UdpServer.h"

class VehicleSimulator
{
public:
    VehicleSimulator(/* args */);
    ~VehicleSimulator();

    void onRecieved(const std::string& message);
    void update(); 
    void listen(); 

private:
    std::unique_ptr<UdpServer> mUdpServer; 
};
#endif //VEHICLESIMULATOR_H






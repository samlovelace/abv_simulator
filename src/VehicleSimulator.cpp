
#include "abv_simulator/VehicleSimulator.h"


VehicleSimulator::VehicleSimulator(/* args */) : 
    mUdpServer(std::make_unique<UdpServer>(6969, std::bind(&VehicleSimulator::onRecieved, this, std::placeholders::_1)))
{
}

VehicleSimulator::~VehicleSimulator()
{
}

void VehicleSimulator::listen()
{
    mUdpServer->start(); 
}

void VehicleSimulator::onRecieved(const std::string& message)
{
    printf("Recieved %s\n", message.c_str()); 

}

void VehicleSimulator::update()
{
}

#include "abv_simulator/VehicleSimulator.h"
#include <thread> 
#include <memory>
#include "abv_simulator/RateController.hpp"

int main()
{
    rclcpp::init(0, nullptr); 
    std::unique_ptr<VehicleSimulator> abv = std::make_unique<VehicleSimulator>(); 

    abv->listen(); 
    RateController rate(50); 
    printf("######################################################\n");

    while(true)
    {
        rate.start(); 
        abv->update(rate.getDeltaTime());
        rate.block(); 
    }

    rclcpp::shutdown(); 
}
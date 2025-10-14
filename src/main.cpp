
#include "abv_simulator/VehicleSimulator.h"
#include <thread> 
#include <memory>


int main()
{
    rclcpp::init(0, nullptr); 
    std::unique_ptr<VehicleSimulator> abv = std::make_unique<VehicleSimulator>(); 

    abv->listen(); 
    printf("######################################################\n"); 

    while(true)
    {
        abv->update(); 

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
    }

    rclcpp::shutdown(); 
}
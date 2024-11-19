
#include "abv_simulator/VehicleSimulator.h"
#include <thread> 
#include <memory>


int main()
{
    std::unique_ptr<VehicleSimulator> abv = std::make_unique<VehicleSimulator>(); 

    abv->listen(); 

    while(true)
    {
        abv->update(); 

        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
    }
}
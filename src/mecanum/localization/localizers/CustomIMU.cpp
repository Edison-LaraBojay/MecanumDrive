#include <CustomIMU.hpp>
#include <main.h>

CustomIMU::CustomIMU(int port) : imu(port) { // initializes imu port

}

// returns heading in radians
double CustomIMU::getHeading(){
    return (imu.get_heading() * M_PI / 180.0); // converts to deg -> rad (M_PI is pi if that wasn't clear...)
}

void CustomIMU::resetYaw(){
    imu.set_heading(0);
}
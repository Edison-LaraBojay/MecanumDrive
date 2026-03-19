#include <main.h>

class CustomIMU {

    public:
    CustomIMU(int port);

    double getHeading();
    void resetYaw();

    private:
    pros::Imu imu;

};
#include <main.h>

class Encoder {
    double FORWARD = 1;
    double REVERSE = -1;

    Encoder(int port);

    void setDirection(double setMultiplier);
    void reset();
    void update();
    double getMultiplier() const;
    double getDeltaPosition() const;

    private:
    pros::Rotation rotationSensor;
    double previousPosition;
    double currentPosition;
    double multiplier;
};
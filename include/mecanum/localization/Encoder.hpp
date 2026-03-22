#include <main.h>
//it is yelling at us here saying a member with an in-class initializer must be const
class Encoder {
    public:
    static double FORWARD = 1;
    static double REVERSE = -1;

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
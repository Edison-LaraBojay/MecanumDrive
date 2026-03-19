#include <Encoder.hpp>

Encoder::Encoder(int port) : rotationSensor(port), previousPosition(0), currentPosition(0), multiplier(FORWARD){ 
    reset();
}

void Encoder::setDirection(double setMultiplier){
    multiplier = setMultiplier;
}

void Encoder::reset(){
    rotationSensor.set_position(0);
    previousPosition = rotationSensor.get_position();
    currentPosition = rotationSensor.get_position();
}

void Encoder::update(){
    // rotationSensor.get_position() returns in centidegrees (100 per degree)
    previousPosition = currentPosition;
    currentPosition = rotationSensor.get_position();
}

double Encoder::getMultiplier() const {
    return multiplier;
}

// delta is in centidegrees
double Encoder::getDeltaPosition() const {
    return getMultiplier() * (currentPosition - previousPosition);
}





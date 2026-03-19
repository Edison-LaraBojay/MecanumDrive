#include <main.h>

class NanoTimer {
    public:
    NanoTimer() : lastTime(pros::micros()) {};
    
    int getElapsedTime() const {
        return pros::micros() - lastTime;
    }

    void resetTimer() {
        lastTime = pros::micros();
    }

    private:
    int lastTime;
};
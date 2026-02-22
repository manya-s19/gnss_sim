#include "Receiver.h"
#include <cmath>

Receiver::Receiver(double x, double y, double z)
    : x(x), y(y), z(z), clockBias(0.0001) {}

double Receiver::distanceTo(double satX, double satY, double satZ) {
    return sqrt(
        pow(satX - x, 2) +
        pow(satY - y, 2) +
        pow(satZ - z, 2)
    );
}
double Receiver::getX() const {
    return x;
}

double Receiver::getY() const {
    return y;
}

double Receiver::getZ() const {
    return z;
}

double Receiver::getClockBias() const {
    return clockBias;
}


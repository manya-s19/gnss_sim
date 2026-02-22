#pragma once

class Receiver {
private:
    double x;
    double y;
    double z;
    double clockBias;

public:
    Receiver(double x, double y, double z);

    double getX() const;
    double getY() const;
    double getZ() const;

    double getClockBias() const;
    double distanceTo(double satX, double satY, double satZ);
};

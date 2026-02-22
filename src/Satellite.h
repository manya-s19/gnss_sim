#ifndef SATELLITE_H
#define SATELLITE_H

class Satellite {
private:
    double altitude;
    double radius;
    double x, y, z;
    double phase;     // NEW
    double omega;     // angular velocity
    double inclination;

public:
    Satellite(double alt, double initialPhase, double incl);

    void update(double time);

    double getX() const;
    double getY() const;
    double getZ() const;
};

#endif
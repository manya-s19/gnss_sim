#include "Satellite.h"
#include <cmath>

const double EARTH_RADIUS = 6371000.0;
const double MU = 3.986e14; // Earth's gravitational parameter

Satellite::Satellite(double orbitalRadius, double initialPhase, double incl)
    : altitude(orbitalRadius), phase(initialPhase), inclination(incl) {

    // FIX #2: treat the parameter as the true orbital radius (not altitude above surface)
    radius = orbitalRadius;

    // Angular velocity for circular orbit
    omega = std::sqrt(MU / std::pow(radius, 3));

    x = radius;
    y = 0.0;
    z = 0.0;
}

void Satellite::update(double time) {
    double angle = omega * time + phase;

    // FIX #1: y and z were identical — correct 3D orbital mechanics:
    //   x = r * cos(angle)                          (in-plane, equatorial direction)
    //   y = r * sin(angle) * cos(inclination)       (in-plane, cross-equatorial)
    //   z = r * sin(angle) * sin(inclination)       (out-of-plane, polar component)
    x = radius * std::cos(angle);
    y = radius * std::sin(angle) * std::cos(inclination);
    z = radius * std::sin(angle) * std::sin(inclination);
}

double Satellite::getX() const { return x; }
double Satellite::getY() const { return y; }
double Satellite::getZ() const { return z; }
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include "Satellite.h"
#include "Receiver.h"
#include "Visualizer.h"

int main() {

    const double earthRadius  = 6371000.0;
    const double angularSpeed = 0.001;

    // 24-satellite GPS-like constellation (6 planes x 4 sats)
    std::vector<Satellite> satellites;

    const int numPlanes    = 6;
    const int satsPerPlane = 4;
    const double inclination = 55.0 * M_PI / 180.0;

    for (int p = 0; p < numPlanes; p++) {
        double planeRotation = p * (2 * M_PI / numPlanes);
        for (int s = 0; s < satsPerPlane; s++) {
            double phase = s * (2 * M_PI / satsPerPlane);
            satellites.emplace_back(
                26571000.0,
                phase + planeRotation,
                inclination
            );
        }
    }

    runVisualizer(satellites, earthRadius, angularSpeed);

    return 0;
}
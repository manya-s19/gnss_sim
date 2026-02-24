#include <iostream>
#include <vector>
#include <cmath>
#include "Satellite.h"
#include "Visualizer_gl.h"

int main() {
    const double earthRadius  = 6371000.0;
    const double angularSpeed = 0.001;

    std::vector<Satellite> satellites;
    const double inclination = 55.0 * M_PI / 180.0;
    for (int p = 0; p < 6; p++) {
        double planeRotation = p * (2 * M_PI / 6);
        for (int s = 0; s < 4; s++) {
            double phase = s * (2 * M_PI / 4);
            satellites.emplace_back(26571000.0, phase + planeRotation, inclination);
        }
    }

    runGLVisualizer(satellites, earthRadius, angularSpeed);
    return 0;
}
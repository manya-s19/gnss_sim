#pragma once
#include <vector>
#include <array>
#include "Satellite.h"

struct ScenarioState {
    std::vector<std::array<double,2>> truePath;
    std::vector<std::array<double,2>> estPath;
    std::vector<bool> spoofDetected;
    std::vector<bool> inNoFly;
    std::array<double,2> pearsonLatLon;
    double noFlyRadiusDeg;
    bool spoofMode;
};

// Split-screen: normal on left, spoofed on right
void runVisualizer(const ScenarioState& normal, const ScenarioState& spoofed);
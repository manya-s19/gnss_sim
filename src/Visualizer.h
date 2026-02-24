#pragma once
#include <vector>
#include <array>
#include "Satellite.h"

struct ScenarioState {
    std::vector<std::array<double,3>> truePath;     // drone's real positions
    std::vector<std::array<double,3>> estPath;      // estimated positions
    std::vector<bool> spoofDetected;                // detection at each step
    std::array<double,3> pearsonPos;                // no-fly zone center
    double noFlyRadius;
    bool spoofMode;
};

void runVisualizer(std::vector<Satellite>& satellites,
                   const ScenarioState& scenario);
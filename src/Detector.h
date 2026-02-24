#pragma once
#include <vector>
#include <deque>
#include <string>
#include <array>

struct DetectionResult {
    bool spoofingDetected;
    double confidence;
    double residualScore;
    double velocityScore;
    double clockScore;
    std::string reason;
};

class Detector {
private:
    std::deque<std::array<double,3>>      posHistory;
    std::deque<double>                    clockHistory;
    std::deque<std::vector<double>>       pseudoHistory;  // NEW

    double maxPhysicalSpeed;
    double residualThreshold;
    double clockJumpThreshold;

    double computeResidualScore(
        const std::vector<std::vector<double>>& satPositions,
        const std::vector<double>& pseudoranges,
        double estX, double estY, double estZ, double clockBias);

public:
    Detector();

    DetectionResult analyze(
        double estX, double estY, double estZ, double clockBias,
        double dt,
        const std::vector<std::vector<double>>& satPositions,
        const std::vector<double>& pseudoranges
    );
};
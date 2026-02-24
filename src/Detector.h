#pragma once
#include <vector>
#include <deque>
#include <string>

struct DetectionResult {
    bool spoofingDetected;
    double confidence;          // 0.0 to 1.0
    double residualScore;       // how large the pseudorange residuals are
    double velocityScore;       // how fast the position jumped
    double clockScore;          // how erratic the clock bias is
    std::string reason;         // human-readable explanation
};

class Detector {
private:
    // History for detecting jumps
    std::deque<std::array<double,3>> posHistory;
    std::deque<double> clockHistory;

    double maxPhysicalSpeed;    // m/s — max believable receiver speed
    double residualThreshold;   // meters — max believable pseudorange residual
    double clockJumpThreshold;  // meters — max believable clock bias jump

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
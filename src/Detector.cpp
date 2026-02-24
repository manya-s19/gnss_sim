#include "Detector.h"
#include <cmath>
#include <sstream>
#include <algorithm>

Detector::Detector()
    : maxPhysicalSpeed(1000.0)      // 1000 m/s max (well above any ground vehicle)
    , residualThreshold(50.0)       // 50m residual is suspicious
    , clockJumpThreshold(100.0)     // 100m clock jump is suspicious
{}

double Detector::computeResidualScore(
    const std::vector<std::vector<double>>& satPositions,
    const std::vector<double>& pseudoranges,
    double estX, double estY, double estZ, double clockBias)
{
    double totalResidual = 0.0;
    int n = satPositions.size();

    for (int i = 0; i < n; i++) {
        double dx = estX - satPositions[i][0];
        double dy = estY - satPositions[i][1];
        double dz = estZ - satPositions[i][2];
        double range = sqrt(dx*dx + dy*dy + dz*dz);
        double residual = fabs(pseudoranges[i] - (range + clockBias));
        totalResidual += residual;
    }

    return totalResidual / n;   // mean absolute residual in meters
}

DetectionResult Detector::analyze(
    double estX, double estY, double estZ, double clockBias,
    double dt,
    const std::vector<std::vector<double>>& satPositions,
    const std::vector<double>& pseudoranges)
{
    DetectionResult result;
    result.spoofingDetected = false;
    result.confidence       = 0.0;
    result.residualScore    = 0.0;
    result.velocityScore    = 0.0;
    result.clockScore       = 0.0;
    result.reason           = "OK";

    // ===== 1. Residual check =====
    result.residualScore = computeResidualScore(
        satPositions, pseudoranges, estX, estY, estZ, clockBias);

    // ===== 2. Velocity check =====
    if (!posHistory.empty()) {
        auto& prev = posHistory.back();
        double dx = estX - prev[0];
        double dy = estY - prev[1];
        double dz = estZ - prev[2];
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        double speed = (dt > 0) ? dist / dt : 0.0;
        result.velocityScore = speed;
    }

    // ===== 3. Clock bias jump check =====
    if (!clockHistory.empty()) {
        double clockJump = fabs(clockBias - clockHistory.back());
        result.clockScore = clockJump;
    }

    // ===== Update history =====
    posHistory.push_back({estX, estY, estZ});
    if (posHistory.size() > 10) posHistory.pop_front();
    clockHistory.push_back(clockBias);
    if (clockHistory.size() > 10) clockHistory.pop_front();

    // ===== Scoring =====
    // Each metric contributes 0-1 to overall confidence
    double residualConf = std::min(1.0, result.residualScore / residualThreshold);
    double velocityConf = std::min(1.0, result.velocityScore / (maxPhysicalSpeed * dt + 1.0));
    double clockConf    = std::min(1.0, result.clockScore    / clockJumpThreshold);

    // Weighted combination
    result.confidence = 0.5 * residualConf + 0.3 * velocityConf + 0.2 * clockConf;

    if (result.confidence > 0.5) {
        result.spoofingDetected = true;
        std::ostringstream oss;
        oss << "SPOOFING DETECTED (confidence=" << result.confidence << ") —";
        if (residualConf > 0.5) oss << " high residuals (" << result.residualScore << "m)";
        if (velocityConf > 0.5) oss << " impossible speed (" << result.velocityScore << "m/s)";
        if (clockConf    > 0.5) oss << " clock jump (" << result.clockScore << "m)";
        result.reason = oss.str();
    }

    return result;
}
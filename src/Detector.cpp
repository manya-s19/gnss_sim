#include "Detector.h"
#include <cmath>
#include <sstream>
#include <algorithm>

Detector::Detector()
    : maxPhysicalSpeed(300.0)       // 300 m/s max drone speed
    , residualThreshold(50.0)
    , clockJumpThreshold(100.0)
{}

double Detector::computeResidualScore(
    const std::vector<std::vector<double>>& satPositions,
    const std::vector<double>& pseudoranges,
    double estX, double estY, double estZ, double clockBias)
{
    double total = 0.0;
    int n = satPositions.size();
    for (int i = 0; i < n; i++) {
        double dx = estX - satPositions[i][0];
        double dy = estY - satPositions[i][1];
        double dz = estZ - satPositions[i][2];
        double range = sqrt(dx*dx + dy*dy + dz*dz);
        total += fabs(pseudoranges[i] - (range + clockBias));
    }
    return total / n;
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

    result.residualScore = computeResidualScore(
        satPositions, pseudoranges, estX, estY, estZ, clockBias);

    // ===== Velocity check — key spoofing indicator =====
    // If position jumped further than physically possible, flag it
    double velocityConf = 0.0;
    if (!posHistory.empty()) {
        auto& prev = posHistory.back();
        double dx = estX - prev[0];
        double dy = estY - prev[1];
        double dz = estZ - prev[2];
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        double speed = (dt > 0) ? dist / dt : 0.0;
        result.velocityScore = speed;
        velocityConf = std::min(1.0, speed / maxPhysicalSpeed);
    }

    // ===== Clock jump check =====
    double clockConf = 0.0;
    if (!clockHistory.empty()) {
        double jump = fabs(clockBias - clockHistory.back());
        result.clockScore = jump;
        clockConf = std::min(1.0, jump / clockJumpThreshold);
    }

    // ===== Pseudorange consistency check =====
    // Compare current pseudoranges against what we'd expect from
    // the previous position — spoofed signals are inconsistent with motion
    double consistencyConf = 0.0;
    if (!posHistory.empty() && !pseudoHistory.empty()) {
        auto& prevPos = posHistory.back();
        auto& prevPR  = pseudoHistory.back();

        // How much did pseudoranges change vs how much position changed
        double prDelta = 0.0;
        int n = std::min(pseudoranges.size(), prevPR.size());
        for (int i = 0; i < n; i++)
            prDelta += fabs(pseudoranges[i] - prevPR[i]);
        prDelta /= n;

        double posDelta = sqrt(
            pow(estX-prevPos[0],2) +
            pow(estY-prevPos[1],2) +
            pow(estZ-prevPos[2],2));

        // In normal operation pseudoranges change proportionally to position
        // Spoofed: position jumps but pseudoranges stay suspiciously similar
        if (posDelta > 100.0 && prDelta < 1.0)
            consistencyConf = std::min(1.0, posDelta / 10000.0);
    }

    // Update history
    posHistory.push_back({estX, estY, estZ});
    if (posHistory.size() > 10) posHistory.pop_front();
    clockHistory.push_back(clockBias);
    if (clockHistory.size() > 10) clockHistory.pop_front();
    pseudoHistory.push_back(pseudoranges);
    if (pseudoHistory.size() > 10) pseudoHistory.pop_front();

    double residualConf = std::min(1.0, result.residualScore / residualThreshold);

    // Weighted — consistency check is most powerful here
    result.confidence = 0.1 * residualConf
                      + 0.3 * velocityConf
                      + 0.1 * clockConf
                      + 0.5 * consistencyConf;

    if (result.confidence > 0.4) {
        result.spoofingDetected = true;
        std::ostringstream oss;
        oss << "SPOOFING DETECTED (confidence=" << (int)(result.confidence*100) << "%)";
        if (consistencyConf > 0.3)
            oss << " — pseudoranges unchanged despite position shift";
        if (velocityConf > 0.3)
            oss << " — impossible speed (" << (int)result.velocityScore << " m/s)";
        result.reason = oss.str();
    }

    return result;
}
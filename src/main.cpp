#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <array>
//version for drone sim
#include "Satellite.h"
#include "Receiver.h"
#include "Spoofer.h"
#include "Detector.h"

// ===== WGS84 ECEF conversion =====
std::array<double,3> lla_to_ecef(double lat_deg, double lon_deg, double alt_m) {
    const double a  = 6378137.0;
    const double e2 = 0.00669437999014;
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
    double N = a / sqrt(1.0 - e2 * sin(lat)*sin(lat));
    double x = (N + alt_m) * cos(lat) * cos(lon);
    double y = (N + alt_m) * cos(lat) * sin(lon);
    double z = (N*(1.0-e2) + alt_m) * sin(lat);
    return {x, y, z};
}

// ===== Least squares solver (unchanged) =====
std::vector<double> solvePositionLeastSquares(
    const std::vector<std::vector<double>>& satPositions,
    const std::vector<double>& pseudoranges,
    double initX, double initY, double initZ)
{
    double x = initX, y = initY, z = initZ, clockBias = 0.0;
    const int maxIterations = 20;
    const int n = satPositions.size();

    for (int iter = 0; iter < maxIterations; iter++) {
        double HtH[4][4] = {0};
        double Htr[4]    = {0};

        for (int i = 0; i < n; i++) {
            double dx = x - satPositions[i][0];
            double dy = y - satPositions[i][1];
            double dz = z - satPositions[i][2];
            double range = sqrt(dx*dx + dy*dy + dz*dz);
            if (range < 1.0) continue;
            double residual = pseudoranges[i] - (range + clockBias);
            double H[4] = {dx/range, dy/range, dz/range, 1.0};
            for (int row = 0; row < 4; row++) {
                Htr[row] += H[row] * residual;
                for (int col = 0; col < 4; col++)
                    HtH[row][col] += H[row] * H[col];
            }
        }

        double A[4][5];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) A[i][j] = HtH[i][j];
            A[i][4] = Htr[i];
        }
        for (int i = 0; i < 4; i++) {
            int maxRow = i; double maxVal = fabs(A[i][i]);
            for (int k = i+1; k < 4; k++)
                if (fabs(A[k][i]) > maxVal) { maxVal = fabs(A[k][i]); maxRow = k; }
            if (maxRow != i)
                for (int j = 0; j < 5; j++) std::swap(A[i][j], A[maxRow][j]);
            double pivot = A[i][i];
            if (fabs(pivot) < 1e-10) continue;
            for (int j = i; j < 5; j++) A[i][j] /= pivot;
            for (int k = 0; k < 4; k++) {
                if (k == i) continue;
                double factor = A[k][i];
                for (int j = i; j < 5; j++) A[k][j] -= factor * A[i][j];
            }
        }
        double dxu = A[0][4], dyu = A[1][4], dzu = A[2][4], dbu = A[3][4];
        const double maxStep = 1e5;
        x += std::max(-maxStep, std::min(maxStep, dxu));
        y += std::max(-maxStep, std::min(maxStep, dyu));
        z += std::max(-maxStep, std::min(maxStep, dzu));
        clockBias += dbu;
        if (fabs(dxu)<1e-4 && fabs(dyu)<1e-4 && fabs(dzu)<1e-4) break;
    }
    return {x, y, z, clockBias};
}

int main(int argc, char* argv[]) {

    bool spoofMode = (argc > 1 && std::string(argv[1]) == "spoof");

    std::cout << "\n";
    std::cout << "==========================================\n";
    std::cout << "  GNSS DRONE NO-FLY ZONE SIMULATOR\n";
    std::cout << "  Scenario: CN Tower → Toronto Pearson\n";
    std::cout << "  Mode: " << (spoofMode ? "SPOOFING ACTIVE" : "Normal") << "\n";
    std::cout << "==========================================\n\n";

    // ===== Drone flight path (5 waypoints, 200m altitude) =====
    std::vector<std::array<double,3>> waypoints = {
        lla_to_ecef(43.6426, -79.3871, 200),  // CN Tower (downtown)
        lla_to_ecef(43.6500, -79.4500, 200),  // midpoint 1
        lla_to_ecef(43.6600, -79.5200, 200),  // midpoint 2 — entering restricted
        lla_to_ecef(43.6700, -79.5800, 200),  // midpoint 3 — inside restricted
        lla_to_ecef(43.6777, -79.6248, 200),  // Toronto Pearson Airport
    };

    // No-fly zone: 5km radius around Pearson
    auto pearson = lla_to_ecef(43.6777, -79.6248, 173);
    const double noFlyRadius = 5000.0;   // 5 km

    // Spoofer: makes drone think it's still at CN Tower (safe zone)
    auto fakePos = lla_to_ecef(43.6426, -79.3871, 200);
    Spoofer spoofer(fakePos[0], fakePos[1], fakePos[2]);

    // ===== GPS constellation =====
    std::vector<Satellite> satellites;
    const double inclination = 55.0 * M_PI / 180.0;
    for (int p = 0; p < 6; p++) {
        double planeRot = p * (2*M_PI/6);
        for (int s = 0; s < 4; s++) {
            double phase = s * (2*M_PI/4);
            satellites.emplace_back(26571000.0, phase+planeRot, inclination);
        }
    }

    Detector detector;
    const double clockBiasTrue = 0.12;
    double simTime = 0.0;

    // ===== CSV =====
    std::ofstream file("output.csv");
    file << "step,true_lat,true_lon,est_lat,est_lon,"
         << "error_m,residual,confidence,spoofed,in_nofly\n";

    // ===== Fly through each waypoint segment =====
    int totalSteps = 0;
    int detectionCount = 0;

    for (int wp = 0; wp < (int)waypoints.size(); wp++) {

        auto& pos = waypoints[wp];
        double rx = pos[0], ry = pos[1], rz = pos[2];

        simTime += 360.0;

        // Update satellites
        for (auto& sat : satellites) sat.update(simTime);

        // Build pseudoranges
        Receiver trueReceiver(rx, ry, rz);
        std::vector<double> pseudoranges;
        std::vector<std::vector<double>> satPositions;
        double recMag = sqrt(rx*rx + ry*ry + rz*rz);

        for (auto& sat : satellites) {
            double sx=sat.getX(), sy=sat.getY(), sz=sat.getZ();
            double dot = sx*rx + sy*ry + sz*rz;
            double satMag = sqrt(sx*sx+sy*sy+sz*sz);
            if (dot/(satMag*recMag) > 0.0) {
                pseudoranges.push_back(trueReceiver.distanceTo(sx,sy,sz) + clockBiasTrue);
                satPositions.push_back({sx, sy, sz});
            }
        }

        if ((int)satPositions.size() < 4) continue;

        // Apply spoofing if active
        if (spoofMode)
            pseudoranges = spoofer.spoofPseudoranges(satPositions, pseudoranges, clockBiasTrue);

        // Solve
        auto sol = solvePositionLeastSquares(satPositions, pseudoranges, rx, ry, rz);
        double estX=sol[0], estY=sol[1], estZ=sol[2], clockBias=sol[3];

        double error = sqrt(pow(estX-rx,2)+pow(estY-ry,2)+pow(estZ-rz,2));

        // Check no-fly zone (using TRUE position)
        double dx=rx-pearson[0], dy=ry-pearson[1], dz=rz-pearson[2];
        double distToPearson = sqrt(dx*dx+dy*dy+dz*dz);
        bool inNoFly = distToPearson < noFlyRadius;

        // Detection
        DetectionResult det = detector.analyze(
            estX, estY, estZ, clockBias, 360.0, satPositions, pseudoranges);

        if (det.spoofingDetected) detectionCount++;
        totalSteps++;

        // Approximate lat/lon back from ECEF for display
        // (simple spherical approximation for printing)
        double trueLat = atan2(rz, sqrt(rx*rx+ry*ry)) * 180.0/M_PI;
        double trueLon = atan2(ry, rx) * 180.0/M_PI;
        double estLat  = atan2(estZ, sqrt(estX*estX+estY*estY)) * 180.0/M_PI;
        double estLon  = atan2(estY, estX) * 180.0/M_PI;

        // Console
        std::cout << "Waypoint " << wp+1 << "/5";
        if (wp == 0) std::cout << " [CN Tower]";
        if (wp == 4) std::cout << " [Pearson Airport]";
        std::cout << "\n";
        std::cout << "  True pos:  " << trueLat << "°N, " << trueLon << "°W\n";
        std::cout << "  Est pos:   " << estLat  << "°N, " << estLon  << "°W\n";
        std::cout << "  Error:     " << error << " m\n";
        std::cout << "  No-fly:    " << (inNoFly ? "YES ⚠️" : "no") << "\n";
        std::cout << "  Residual:  " << det.residualScore << " m\n";
        std::cout << "  Confidence:" << det.confidence << "\n";

        if (det.spoofingDetected) {
            std::cout << "  *** " << det.reason << " ***\n";
            if (spoofMode && inNoFly)
                std::cout << "  !!! DRONE IN NO-FLY ZONE — SPOOFING MASKED TRUE POSITION !!!\n";
        }
        std::cout << "\n";

        file << wp+1 << ","
             << trueLat << "," << trueLon << ","
             << estLat  << "," << estLon  << ","
             << error   << "," << det.residualScore << ","
             << det.confidence << ","
             << (det.spoofingDetected ? 1 : 0) << ","
             << (inNoFly ? 1 : 0) << "\n";
    }

    std::cout << "==========================================\n";
    std::cout << "  SUMMARY\n";
    std::cout << "  Spoofing detected at: "
              << detectionCount << "/" << totalSteps << " waypoints\n";
    if (spoofMode)
        std::cout << "  Without detection, drone would have entered\n"
                  << "  Pearson Airport no-fly zone undetected!\n";
    std::cout << "==========================================\n";

    file.close();
    return 0;
}
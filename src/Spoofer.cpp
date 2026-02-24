#include "Spoofer.h"
#include <cmath>

Spoofer::Spoofer(double fakeX, double fakeY, double fakeZ, double power)
    : fakeX(fakeX), fakeY(fakeY), fakeZ(fakeZ), power(power) {}

std::vector<double> Spoofer::spoofPseudoranges(
    const std::vector<std::vector<double>>& satPositions,
    const std::vector<double>& realPseudoranges,
    double clockBias)
{
    std::vector<double> spoofed;

    for (int i = 0; i < (int)satPositions.size(); i++) {
        double sx = satPositions[i][0];
        double sy = satPositions[i][1];
        double sz = satPositions[i][2];

        // Fake distance: from satellite to the fake receiver position
        double dx = fakeX - sx;
        double dy = fakeY - sy;
        double dz = fakeZ - sz;
        double fakeDist = sqrt(dx*dx + dy*dy + dz*dz) + clockBias;

        // Blend real and fake based on spoofer power
        double blended = (1.0 - power) * realPseudoranges[i] + power * fakeDist;
        spoofed.push_back(blended);
    }

    return spoofed;
}
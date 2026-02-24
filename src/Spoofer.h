#pragma once
#include <vector>

// A Spoofer injects fake pseudoranges to make the receiver
// think it's at a different location (the "fake" position).
class Spoofer {
private:
    double fakeX, fakeY, fakeZ;   // where the spoofer wants receiver to think it is
    double power;                  // 0.0 = off, 1.0 = fully spoofed

public:
    Spoofer(double fakeX, double fakeY, double fakeZ, double power = 1.0);

    // Takes real pseudoranges and corrupts them to match the fake position
    std::vector<double> spoofPseudoranges(
        const std::vector<std::vector<double>>& satPositions,
        const std::vector<double>& realPseudoranges,
        double clockBias
    );

    double getFakeX() const { return fakeX; }
    double getFakeY() const { return fakeY; }
    double getFakeZ() const { return fakeZ; }
};
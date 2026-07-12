// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <openvr_driver.h>

class KalmanFilterXYZ {
public:
    KalmanFilterXYZ();
    ~KalmanFilterXYZ();
    double GetQ() { return Q; };
    double GetR() { return R; };
    void SetQ(double q) { Q = q; };
    void SetR(double r) { R = r; };
    void SetAdaptiveGain(double a) { A = a; };
    void reset();
    vr::HmdVector3d_t update(const vr::HmdVector3d_t& measurement);
private:
    double Q = 0.000150;
    double R = 0.01;
    double A = 0.0;
    double P;
    double K;
    vr::HmdVector3d_t X;
    bool primed = false;
    void calc(const double innovation);
};
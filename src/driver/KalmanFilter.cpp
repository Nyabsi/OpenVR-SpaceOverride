// SPDX-License-Identifier: AGPL-3.0-only

#include "KalmanFilter.h"

#include <cmath>

KalmanFilterXYZ::KalmanFilterXYZ() : P(1), K(0), X{ 0, 0, 0 }
{
}

KalmanFilterXYZ::~KalmanFilterXYZ()
{
}

void KalmanFilterXYZ::reset()
{
    P = 1;
    K = 0;
    X = { 0, 0, 0 };
    primed = false;
}

void KalmanFilterXYZ::calc(const double innovation)
{
    double q = Q;
    double excess = innovation - 2.0 * std::sqrt(R);
    if (A > 0.0 && excess > 0.0)
        q += A * excess * excess;
    K = (P + q) / (P + q + R);
    P = R * (P + q) / (R + P + q);
}

vr::HmdVector3d_t KalmanFilterXYZ::update(const vr::HmdVector3d_t& measurement)
{
    if (!primed) {
        X = measurement;
        P = R;
        primed = true;
        return X;
    }

    double dx = measurement.v[0] - X.v[0];
    double dy = measurement.v[1] - X.v[1];
    double dz = measurement.v[2] - X.v[2];

    calc(std::sqrt(dx * dx + dy * dy + dz * dz));

    X.v[0] += dx * K;
    X.v[1] += dy * K;
    X.v[2] += dz * K;
    return X;
}
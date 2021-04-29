#pragma once

#include <cmath>

static double sigmoid(double x, double kappa = 1.0)
{
    return 1.0 / (1.0 + std::exp(-x * kappa));
}

static double sinus_step(double x, double kappa = 1.0)
{
    if (x <= 0.0)
        return 0.0;
    if (x >= 1.0 / kappa)
        return 1.0;
    return std::sin((x * kappa * M_PI - M_PI_2)) * 0.5 + 0.5;
}
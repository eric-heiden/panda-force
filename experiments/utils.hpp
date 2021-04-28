#pragma once

#include <cmath>

static double sigmoid(double x, double kappa = 1.0)
{
    return 1.0 / (1.0 + std::exp(-x * kappa));
}
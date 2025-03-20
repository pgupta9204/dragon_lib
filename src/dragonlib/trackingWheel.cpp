#include "main.h"
#include "dragonlib/api.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

double TrackingWheel::getTotalDistanceTravelled() {
    double revolutions = this->rotationSensor.get_position() / 36000;
    return revolutions * M_PI * this->wheelDiameter;
}
#pragma once
#include "main.h"
#include "api.h"
#include "ports.hpp"
#include "lemlib/api.hpp"

class Distance {
public:

    // Sensors

    pros::Distance distancer;
    pros::Distance distancel;
    pros::Distance distanceb;
    pros::Distance distancef;

    Distance();

    // Functions
    void resetcoord(int quadrant, lemlib::Chassis& chassis);
    double mmToIn(double mm);
    double safeRead(pros::Distance& sensor);


private:

};

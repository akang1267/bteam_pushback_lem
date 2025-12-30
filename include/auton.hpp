#pragma once
#include "main.h"
#include "robot.hpp"
#include "distance.hpp"
#include "lemlib/api.hpp"

void solo_awp_right(lemlib::Chassis& chassis, Robot robot, Distance distance);
void elims_left(lemlib::Chassis& chassis, Robot robot, Distance distance);
void pid_tune(lemlib::Chassis& chassis);
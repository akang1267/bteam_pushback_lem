#include "main.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "ports.hpp"
#include "distance.hpp"

Distance::Distance ()
  : distancer(ports::distancerP),      // from ports.hpp
    distancel(ports::distancelP),
    distanceb(ports::distancebP),    // example: ADI port definitions
    distancef(ports::distancefP) {
}



const double FIELD_WIDTH  = 144.0;
const double FIELD_HEIGHT = 144.0;

//to be edited
const double FRONT_OFFSET = 4.5;
const double BACK_OFFSET  = 5.0;
const double LEFT_OFFSET  = 6;
const double RIGHT_OFFSET = 6;



double Distance::mmToIn(double mm) { 
    return mm / 25.4; 
}

double Distance::safeRead(pros::Distance& sensor) {
    int raw = sensor.get();

    // Reject invalid mm readings
    if (raw <= 0 || raw > 2000) return -1;

    double inches = mmToIn(raw);

    // Reject readings under 8 inches
    if (inches < 1) return -1;

    return inches;
}

void Distance::resetcoord(int quadrant, lemlib::Chassis& chassis) {
    double front = safeRead(distancef);
    double back  = safeRead(distanceb);
    double left  = safeRead(distancel);
    double right = safeRead(distancer);

    // Default to current pose if a reading is invalid
    lemlib::Pose current = chassis.getPose();
    double xPos = current.x;
    double yPos = current.y;

    // ---------------- FIELD-CENTERED X POSITION ----------------
    // Walls:
    // Bottom wall = y = -72
    // Top wall    = y = +72
    if (front > 0 && back > 0) {
        double x_from_back  = -72 + (back  + BACK_OFFSET);
        double x_from_front =  72 - (front + FRONT_OFFSET);
        xPos = (x_from_back + x_from_front) / 2.0;
    } else if (back > 0) {
        xPos = -72 + (back + BACK_OFFSET);
    } else if (front > 0) {
        xPos =  72 - (front + FRONT_OFFSET);
    }

    // ---------------- FIELD-CENTERED Y POSITION ----------------
    // Walls:
    // Left wall  = x = -72
    // Right wall = x = +72
    if (left > 0 && right > 0) {
        double y_from_left  = -72 + (left  + LEFT_OFFSET);
        double y_from_right =  72 - (right + RIGHT_OFFSET);
        yPos = (y_from_left + y_from_right) / 2.0;
    } else if (left > 0) {
        yPos = -72 + (left + LEFT_OFFSET);
    } else if (right > 0) {
        yPos =  72 - (right + RIGHT_OFFSET);
    }

    // Apply quadrant sign logic AFTER computing raw values
    switch (quadrant) {
        case 1:
            xPos = fabs(xPos);
            yPos = fabs(yPos);
            break;
        case 2:
            xPos = -fabs(xPos);
            yPos =  fabs(yPos);
            break;
        case 3:
            xPos = -fabs(xPos);
            yPos = -fabs(yPos);
            break;
        case 4:
            xPos =  fabs(xPos);
            yPos = -fabs(yPos);
            break;
        default:
            break;
    }



    // printf("Sensors  F:%.2f  B:%.2f  L:%.2f  R:%.2f\n", front, back, left, right);
    // printf("Result   X: %.2f  Y: %.2f  H: %.2f\n", xPos, yPos, chassis.getPose().theta);

    // pros::lcd::print(0, "X: %.2f  Y: %.2f", xPos, yPos);
    // pros::lcd::print(1, "F:%.1f B:%.1f L:%.1f R:%.1f", front, back, left, right);
    
    chassis.setPose(xPos, yPos, chassis.getPose().theta);
}

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

void Distance::resetcoord(int quadrant, int angle, lemlib::Chassis& chassis) {
    double front = safeRead(distancef) + FRONT_OFFSET;
    double back  = safeRead(distanceb) + BACK_OFFSET;
    double left  = safeRead(distancel) + LEFT_OFFSET;
    double right = safeRead(distancer) + RIGHT_OFFSET;

    // Default to current pose if a reading is invalid
    lemlib::Pose current = chassis.getPose();
    double xPos = current.x;
    double yPos = current.y;

    double HALF_FIELD = 72;

    bool red = false;
    bool blue = false;
    bool leftd = false;
    bool rightd = false;

    switch (angle)
    {
    case 0:
        blue = true;
        break;
    
    case 90:
        right = true;
        break;
    case 180:
        red = true;
        break;
    case 270:
        left = true;
        break;
    default:
        break;
    }

    // QUAD 1

    if (quadrant == 1){
        if(red){
            xPos = (HALF_FIELD - left);
            yPos = (HALF_FIELD - back);

        }
        else if (blue){
            xPos = (HALF_FIELD - right);
            yPos = (HALF_FIELD - front);
        }
        else if (rightd){
            xPos = (HALF_FIELD - front);
            yPos = (HALF_FIELD - left);
        }

        else if (leftd){
            xPos = (HALF_FIELD - back);
            yPos = (HALF_FIELD - right);
        }
    
    }

    // QUAD 2

    if (quadrant == 2){
        if(red){
            xPos = -(HALF_FIELD - right);
            yPos = (HALF_FIELD - back);
        }
        else if (blue){
            xPos = -(HALF_FIELD - left);
            yPos = (HALF_FIELD - front);
        }
        else if (rightd){
            xPos = -(HALF_FIELD - back);
            yPos = (HALF_FIELD - left);
        }

        else if (leftd){
            xPos = -(HALF_FIELD - front);
            yPos = (HALF_FIELD - right);
        }
    
    }

    // QUAD 3

    if (quadrant == 3){
        if(red){
            xPos = -(HALF_FIELD - right);
            yPos = -(HALF_FIELD - front);
        }
        else if (blue){
            xPos = -(HALF_FIELD - left);
            yPos = -(HALF_FIELD - back);
        }
        else if (rightd){
            xPos = -(HALF_FIELD - back);
            yPos = -(HALF_FIELD - right);
        }

        else if (leftd){
            xPos = -(HALF_FIELD - front);
            yPos = -(HALF_FIELD - left);
        }
    
    }

    // QUAD 4

    if (quadrant == 4){
        if(red){
            xPos = (HALF_FIELD - left);
            yPos = -(HALF_FIELD - front);

        }
        else if (blue){
            xPos = (HALF_FIELD - right);
            yPos = -(HALF_FIELD - back);
        }
        else if (rightd){
            xPos = (HALF_FIELD - front);
            yPos = -(HALF_FIELD - right);
        }

        else if (leftd){
            xPos = (HALF_FIELD - back);
            yPos = -(HALF_FIELD - left);
        }
    
    }


    // printf("Sensors  F:%.2f  B:%.2f  L:%.2f  R:%.2f\n", front, back, left, right);
    // printf("Result   X: %.2f  Y: %.2f  H: %.2f\n", xPos, yPos, chassis.getPose().theta);

    // pros::lcd::print(0, "X: %.2f  Y: %.2f", xPos, yPos);
    // pros::lcd::print(1, "F:%.1f B:%.1f L:%.1f R:%.1f", front, back, left, right);
    
    chassis.setPose(xPos, yPos, chassis.getPose().theta);
}

#include "main.h"
#include "robot.hpp"
#include "auton.hpp"
#include "distance.hpp"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"

void solo_awp_right(lemlib::Chassis& chassis,Robot robot, Distance distance){
    chassis.moveToPoint(0, 24, 1000, {.minSpeed = 10, .earlyExitRange = 1});

    chassis.turnToHeading(90, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    robot.toggle_matchload();
    //toggle amtchlod

    chassis.moveToPoint(8, 24, 1000);
    robot.set_intakef(12000);
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.moveToPoint(-21, 24, 1500, {.forwards = false}); //backwards
    chassis.waitUntil(22);
    robot.set_intakeb(12000);
    //score
    pros::delay(500);
    chassis.moveToPoint(-13, 23, 1000);

    robot.toggle_matchload();

    chassis.turnToHeading(208, 1000, {.minSpeed = 1, .earlyExitRange = 5});
    robot.set_both(0);
    chassis.moveToPoint(-22, 10, 1500);
    robot.set_intakef(12000);
    chassis.waitUntil(14);
    robot.toggle_matchload();
    chassis.turnToHeading(185, 1000, {.minSpeed = 1, .earlyExitRange = 5});

    chassis.moveToPoint(-23, -32, 1500);
    chassis.waitUntil(1);
    robot.toggle_matchload();
    chassis.waitUntil(29);
    robot.toggle_matchload();

    chassis.turnToHeading(125, 1000);
    chassis.waitUntil(10);
    robot.set_intakef(0);

    chassis.moveToPoint(-36, -19.5, 2500, {.forwards = false, .minSpeed = 70});
    chassis.waitUntil(5);
    robot.set_intakef(-12000);
    pros::delay(500);
    robot.set_intakef(12000);
    chassis.waitUntilDone();
    pros::delay(2000);
    robot.set_intakef(-12000);
    pros::delay(300);
    robot.set_intakef(12000);

    chassis.moveToPoint(1, -51.3, 2000, {.minSpeed = 10 , .earlyExitRange = 2});
    chassis.turnToHeading(90, 1000, {.minSpeed = 10, .earlyExitRange = 5});
    chassis.moveToPoint(13, -51.3, 1000);
    pros::delay(500);
    chassis.moveToPoint(-19, -52, 1000, {.forwards = false});
    chassis.waitUntilDone();
    robot.set_both(12000);

}

void elims_left(lemlib::Chassis& chassis, Robot robot, Distance distance){
    pros::delay(3000);
    chassis.moveToPose(-3, 19, -11, 1000, {.minSpeed = 70, .earlyExitRange = 2}); 
    robot.set_intakef(12000);
    chassis.waitUntil(20);
    robot.toggle_matchload();
    chassis.moveToPoint(-5.5, 25.5, 1000, {.maxSpeed = 100});

    chassis.turnToHeading(-135, 1000, {.minSpeed = 10, .earlyExitRange = 5});
    chassis.moveToPoint(6, 36, 2000, {.forwards = false});
    robot.set_intakef(-12000);
    pros::delay(200);
    robot.set_intakef(12000);
    pros::delay(2000);
    chassis.moveToPoint(-28, 9, 1000, {.minSpeed = 1,.earlyExitRange = 2});
    chassis.turnToHeading(-180, 1000);
    chassis.moveToPoint(-28, -9, 1500);
    chassis.moveToPoint(-28.5, 24, 1500, {.forwards = false});
    chassis.waitUntil(23);
    robot.set_both(12000);

    //placeholder for test
    distance.resetcoord(1, chassis);
}

void pid_tune(lemlib::Chassis& chassis){
    chassis.moveToPoint(0, 24, 3000);
    pros::delay (300);

    chassis.turnToHeading(90, 2000);
    pros::delay (300);
    chassis.turnToHeading(270, 2000);
    pros::delay (300);
    chassis.turnToHeading(180, 2000);
}
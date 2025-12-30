#pragma once
#include "api.h"
#include "lemlib/api.hpp"
#include "ports.hpp"

class Robot {
public:

    // Controller
    pros::Controller controller {pros::E_CONTROLLER_MASTER}; 

    // Motors
    pros::Motor intakef;
    pros::Motor intakeb;

    // Pistons
    pros::ADIDigitalOut matchload;
    pros::ADIDigitalOut descore;

    // Constructor 
    Robot();

    // Functions
    void set_both(int32_t voltage);
    void set_intakef(int32_t voltage);
    void set_intakeb(int32_t voltage);

    void toggle_matchload();
    void toggle_descore();

    void update_robot();


private:
    bool matchload_state = false;
    bool descore_state = false;
};

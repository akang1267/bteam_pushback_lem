#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "robot.hpp"

Robot::Robot()
  : intakef(ports::intakefP),      // from ports.hpp
    intakeb(ports::intakebP),
    matchload(ports::matchloadP),    // example: ADI port definitions
    descore(ports::descoreP) {
}

void Robot::set_both(int32_t voltage) {
  intakef.move_voltage(voltage);
  intakeb.move_voltage(voltage);
}

void Robot::set_intakeb(int32_t voltage) {
  intakeb.move_voltage(voltage);
}

void Robot::set_intakef(int32_t voltage) {
  intakef.move_voltage(voltage);
}

void Robot::toggle_matchload() {
  matchload_state = !matchload_state;
  matchload.set_value(matchload_state);
}

void Robot::toggle_descore() {
  descore_state = !descore_state;
  descore.set_value(descore_state);
}

void Robot::update_robot(){
    int8_t L1_pressing = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1); 
    int8_t L2_pressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);

    int8_t R1_pressing = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);  
    int8_t R2_pressing = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    int8_t A_pressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);  

    if(L2_pressed){
        toggle_descore();
    }
    
    else if(A_pressed){
        toggle_matchload();
    }

    else if(R2_pressing){
        set_both(-12000);
    }

    else if (R1_pressing){ 
        set_intakef(12000);
    }

    else if(L1_pressing){
        set_both(12000);
    }

    else{
        set_both(0);
    }
}
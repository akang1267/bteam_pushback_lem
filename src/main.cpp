#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

constexpr int8_t leftF = -17; 
constexpr int8_t leftM = -15;
constexpr int8_t leftB = 16;

constexpr int8_t rightF = 18;
constexpr int8_t rightM = 20;
constexpr int8_t rightB = -19;

constexpr int8_t intakefP = 7;
constexpr int8_t intakebP = 8;

constexpr char descoreP = 'A';
constexpr char matchloadP = 'C';

bool descore_state = false;
bool middle_state = false;
bool matchload_state = false;

pros::Motor intakef (intakefP);
pros::Motor intakeb (intakebP);

pros::ADIDigitalOut descore (descoreP);
pros::ADIDigitalOut matchload (matchloadP);



// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({leftF, leftM, leftB},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({rightF, rightM, rightB}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

//MADE-FUNCTIONS****

void set_both(int32_t voltage){
    intakef.move_voltage(voltage);
    intakeb.move_voltage(voltage);
}

void set_intakeb(int32_t voltage){
    intakeb.move_voltage(voltage);
}

void set_intakef(int32_t voltage){
    intakef.move_voltage(voltage);
}

void toggle_matchload(){
    matchload_state = !matchload_state;
    matchload.set_value(matchload_state);
}

void toggle_descore(){
    descore_state = !descore_state;
    descore.set_value(descore_state);
}

void example_auton(){
    chassis.moveToPoint(0, 24, 1000);
}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    example_auton();
}

/**
 * Runs in driver control
 */

int8_t L1_pressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1); 
int8_t L2_pressing = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

int8_t R1_pressing = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);  
int8_t R2_pressing = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

int8_t X_pressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);  

void update_robot(){

    if(L1_pressed){
        toggle_descore();
    }

    if(L2_pressing){
        set_both(-1200);
    }

    if(R1_pressing){
        set_intakef(1200);
    }

    if(R2_pressing){
        set_both(1200);
    }

    if(X_pressed){
        toggle_matchload();
    }

    set_both(0);

}

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        update_robot();
        // delay to save resources
        pros::delay(10);
    }
}

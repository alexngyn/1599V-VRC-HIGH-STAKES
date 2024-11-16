#include "setup.h"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);

pros::MotorGroup dt_left (LEFT_MOTOR_PORTS, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup dt_right (RIGHT_MOTOR_PORTS, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup dt_motors (DT_MOTOR_PORTS, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::adi::Pneumatics clamp_solenoid (CLAMP_SOLENOID_PORT, false);
pros::adi::Pneumatics doinker_solenoid(DOINKER_PORT, false);
//pros::adi::DigitalOut indicator_g (INDICATOR_G_PORT);

pros::Motor intake_motor (INTAKE_PORT, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::Motor arm_motor (ARM_PORT, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);

pros::Optical optical_sensor (OPTICAL_SENSOR_PORT);

pros::IMU inertial_sensor (INERTIAL_SENSOR_PORT);

pros::Rotation arm_rotational_sensor (ARM_ROTATIONAL_SENSOR_PORT);

color sideColor = sideColorInit;

Arm arm_controller(
    arm_motor,
    arm_rotational_sensor,
    1,
    lemlib::PID {3.0, 30, 0.155, true}
);

// drivetrain settingss
lemlib::Drivetrain drivetrain {
    &dt_left, // left drivetrain motors
    &dt_right, // right drivetrain motors
    13.25, // track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    450, // drivetrain rpm is 480
    4 // omni chase power is 2. If we had traction wheels, it would have been 8
};

// lateral motion controller

lemlib::ControllerSettings linearController (
    10, // proportional gain (kP)
    0, // integral gain (kI)
    55, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController (
    2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew) prevent wheel cuz we dont have encoder
);

// sensors for odometry
// note that in this example we use internal motor encoders, so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1, set to nullptr as we don't have one
    nullptr, // vertical tracking wheel 2, set to nullptr as we don't have one
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &inertial_sensor // inertial sensor
};

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

//converts color (enum) to a string
std::string colorToString(color color) {
    switch (color) {
        case red: return "red";
        case blue: return "blue";
        default: return "unknown";
    }
}
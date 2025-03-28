#include "setup.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);

pros::MotorGroup dt_left (LEFT_MOTOR_PORTS, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup dt_right (RIGHT_MOTOR_PORTS, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup dt_motors (DT_MOTOR_PORTS, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::adi::Pneumatics clamp_solenoid (CLAMP_SOLENOID_PORT, false);
pros::adi::Pneumatics doinker_solenoid(DOINKER_PORT, false);
pros::adi::Pneumatics intake_solenoid(INTAKE_SOLENOID_PORT, false);
pros::adi::DigitalOut indicator (INDICATOR_PORT);
pros::adi::AnalogIn selector (SELECTION_PORT);

pros::Motor intake_motor (INTAKE_PORT, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::Motor arm_motor (ARM_PORT, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);

// pros::Optical optical_sensor (OPTICAL_SENSOR_PORT);
pros::Vision vision_sensor(VISION_SENSOR_PORT);

// vision::signature SIG_1 (1, -4479, -3039, -3758, 7423, 10461, 8942, 3.000, 0);
// vision::signature SIG_2 (2, 7617, 11241, 9430, -1281, 255, -514, 3.000, 0);

pros::vision_signature_s_t REDSIG =
    pros::Vision::signature_from_utility(1, -4479, -3039, -3758, 7423, 10461, 8942, 3.000, 0);
pros::vision_signature_s_t BLUESIG =
    pros::Vision::signature_from_utility(2, 7617, 11241, 9430, -1281, 255, -514, 3.000, 0);

pros::IMU inertial_sensor (INERTIAL_SENSOR_PORT);

pros::Rotation arm_rotational_sensor (ARM_ROTATIONAL_SENSOR_PORT);

pros::Rotation horizontal_encoder (HORIZONTAL_ENCODER_PORT);
pros::Rotation vertical_encoder (VERTICAL_ENCODER_PORT);

color sideColor = color::unknown;

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
    6 // omni chase power is 2. If we had traction wheels, it would have been 8
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
    4, // proportional gain (kP)
    0, // integral gain (kI)
    22, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew) prevent wheel cuz we dont have encoder
);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.625);

// sensors for odometry
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
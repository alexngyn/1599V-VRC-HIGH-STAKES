#include "setup.h"
#include "intake.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);
//pros::Controller partner (pros::E_CONTROLLER_PARTNER);

pros::MotorGroup dt_left (LEFT_MOTOR_PORTS, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup dt_right (RIGHT_MOTOR_PORTS, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::adi::Pneumatics clamp_solenoid (CLAMP_SOLENOID_PORT, false);
pros::adi::Pneumatics doinker_solenoid(DOINKER_PORT, false);
pros::adi::Pneumatics intake_solenoid(INTAKE_SOLENOID_PORT, false);
pros::adi::DigitalOut indicator (INDICATOR_PORT);
pros::adi::AnalogIn selector (SELECTION_PORT);

pros::Motor intake_motor (INTAKE_PORT, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup arm_motors (ARM_PORTS, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);

pros::Optical optical_sensor (OPTICAL_SENSOR_PORT);
pros::Optical distance_sensor (DISTANCE_SENSOR_PORT);

pros::IMU inertial_sensor (INERTIAL_SENSOR_PORT);

pros::Rotation arm_rotational_sensor (ARM_ROTATIONAL_SENSOR_PORT);

//pros::Rotation horizontal_encoder (HORIZONTAL_ENCODER_PORT);
pros::Rotation vertical_encoder (VERTICAL_ENCODER_PORT);

color sideColor = color::unknown;

Arm arm_controller(
    &arm_motors,
    &arm_rotational_sensor,
    lemlib::PID {1.5, 0.2, 1, true} // 3.0, 30, 0.155
);

Intake intake_controller(
    intake_motor,
    distance_sensor,
    optical_sensor,
    arm_controller
);

// drivetrain settings
lemlib::Drivetrain drivetrain {
    &dt_left, // left drivetrain motors
    &dt_right, // right drivetrain motors
    11.85, // track width 11.83
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    450, // drivetrain rpm is 480
    4 // omni chase power is 2. If we had traction wheels, it would have been 8
};

// lateral motion controller

lemlib::ControllerSettings linearController ( // 26,0,150      ,7
    10, // proportional gain (kP)
    0, // integral gain (kI)
    55, // derivative gain (kD)
    3, // anti windup
    0.8, // small error range, in inches
    100, // small error range timeout, in milliseconds
    2, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController ( // 5.2,0,45
    3, // proportional gain (kP)
    0, // integral gain (kI)
    20, // derivative gain (kD)
    3, // anti windup
    2, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    5, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    10 // maximum acceleration (slew) prevent wheel cuz we dont have encoder
);


// horizontal tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -3.25);

// sensors for odometry
lemlib::OdomSensors sensors {
    &vertical_tracking_wheel, // vertical tracking wheel 1, &vertical_tracking_wheel
    nullptr, // vertical tracking wheel 2, set to nullptr as we don't have one
    nullptr, // horizontal tracking wheel 1,  set to nullptr as we don't have one
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &inertial_sensor // inertial sensor
};

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);
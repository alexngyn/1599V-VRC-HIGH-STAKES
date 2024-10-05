#include "main.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);

pros::Motor dt_motor_lf (DT_WHEEL_LF_PORT, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor dt_motor_lm (DT_WHEEL_LM_PORT, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor dt_motor_lb (DT_WHEEL_LB_PORT, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor dt_motor_rf (DT_WHEEL_RF_PORT, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor dt_motor_rm (DT_WHEEL_RM_PORT, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor dt_motor_rb (DT_WHEEL_RB_PORT, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group dt_left ({dt_motor_lf, dt_motor_lm, dt_motor_lb});
pros::Motor_Group dt_right ({dt_motor_rf, dt_motor_rm, dt_motor_rb});

pros::Motor arm_motor_left (ARM_L_PORT, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor arm_motor_right (ARM_R_PORT, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group arm ({arm_motor_right, arm_motor_left});

pros::Motor intake_motor (INTAKE_PORT, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

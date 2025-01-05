#pragma once

#include "api.h"  // IWYU pragma: keep
#include "pros/vision.hpp"
//#include "setup.h"

#define targetAngleInit 16

class Intake {
    public:
        Intake(pros::Motor motor, pros::Vision vision, lemlib::PID pid);
        ~Intake() {this->task.remove();};

        enum class state {
            STOP,
            INTAKE,
            REDIRECT
        };

        Intake::state currState = Intake::state::STOP;

        void setVelocity(double angle);
        void toggleSort();
        void redirect();

    private:

        pros::Motor motor;
        pros::Vision vision;
        double ratio;
        lemlib::PID PID;
        int velocity;

        pros::Task task = pros::Task {[&] {
            while (true) {                                                                                                                                                              
                if (currState == state::INTAKE){
                    this->motor.move_velocity(velocity);
                    
                    int prevUnjam = 0;
                    // unjam
                    if ((intake_motor.get_actual_velocity() < (0.1*intake_motor.get_target_velocity())) && (pros::millis() - prevUnjam < 2000)) {
                        intake_motor.move_velocity(-600);
                        pros::delay(500);
                        intake_motor.move_velocity(600);
                        pros::delay(400);
                        prevUnjam = pros::millis();
                    }

                    void colorSortVision() {
                        pros::vision_object_s_t rtn = this->vision.get_by_size(0);

                        intake_motor.move_velocity(600);

                        if (((sideColor == color::blue && rtn.signature == 2) || //red
                            (sideColor == color::red && rtn.signature == 1)) //blue
                            && ejectEnabled && rtn.height > 150) {
                            
                            double initPos = intake_motor.get_position();

                            while ((initPos + 720) - intake_motor.get_position() > 10) {
                                pros::delay(10);
                            }

                            intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                            intake_motor.brake();
                            pros::delay(200);
                            intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        }
                    }


                } else if (currState == state::REDIRECT){
                    pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);

                    intake_motor.move_velocity(600);

                    if ((rtn.signature == 2 || //red
                        rtn.signature == 1) //blue
                        && rtn.height > 150) {
                        
                        double initPos = intake_motor.get_position();

                        while ((initPos + 530) - intake_motor.get_position() > 10) {
                            pros::delay(10);
                        }

                        intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        intake_motor.brake();
                        pros::delay(100);
                        // // intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

                        intake_motor.move_relative(-1200, 400);
                        pros::delay(1000);

                        // intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                    } 
                    pros::delay(10);

                    // if (((sideColor == color::blue && rtn.signature == 2) || //red
                    //     (sideColor == color::red && rtn.signature == 1)) //blue
                    //     && ejectEnabled && rtn.height > 100) {

                    //     double initPos = intake_motor.get_position();

                    //     lemlib::PID intakePID(8, 0, 0, 1, false);

                    //     while ((initPos + 360) - intake_motor.get_position() > 10) {
                    //         intake_motor.move_velocity(intakePID.update((initPos + 480) - intake_motor.get_position()));
                    //         pros::delay(10);
                    //     }

                    //     intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    //     intake_motor.brake();
                    //     pros::delay(50);
                    //     intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                    //     intake_motor.move_velocity(-400);
                    //     pros::delay(200);
                    // } 

                    // intake_motor.move_velocity(600);
                    // intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    // while (optical_sensor.get_proximity() < 240){
                    //     pros::delay(10);
                    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == true) {break;}
                    // }
                    // pros::delay(95);
                    // intake_motor.brake();
                    // intake_motor.move_velocity(0);
                    // pros::delay(500);
                }
                pros::delay(20);
            }
        }
        pros::Task logTask = pros::Task {[&] {
            while (true) {                                                                                                                                                              
                intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                while (true) {
                    if(!ejectEnabled && printLoop) {master.print(0,0, "Color sort: off");} else if (printLoop) {master.print(0, 0, "Color sort: %s", colorToString(sideColor));}
                    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                      master.clear_line(0);
                      ejectEnabled = !ejectEnabled;
                  }
                  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) { sideColor == red ? sideColor = color::blue : sideColor = color::red;}
                  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                      intake_motor.move_velocity(600);
                      if (ejectEnabled) {colorSortVision();}
                      unjam();
                  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                      intake_motor.move_velocity(-400);
                  } else {
                      intake_motor.move_velocity(0);
                  }
                  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { redirect(); }
                  pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
                  printLoop = !printLoop;
              }

                pros::delay(20);
            }
        }
    };;
};
#pragma once
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "arm.h"

#define INTAKE_SPEED 600
#define OUTTAKE_SPEED -400
#define INTAKE_LDB_SPEED 300

class Intake {
    public:

        Intake( pros::Motor& motor, pros::Optical& holdSensor, pros::Optical& sortSensor, Arm& arm);
        ~Intake() {this->task.remove();};

        enum IntakeState { STOPPED , INTAKING , OUTTAKE };
        enum SortState { RED , BLUE , OFF };

        void set(IntakeState state);
        void toggleState();
        void setState(SortState state);
        SortState getState();
        void hold(bool async = true, int timeout = 10000);
        void holdldb(bool async = true, int timeout = 10000);
        void waitUntilDone(int timeout = 10000);
        
    private:
        pros::Motor& motor;
        pros::Optical& holdSensor;
        pros::Optical& sortSensor;
        Arm& arm;

        IntakeState state = STOPPED;
        SortState sort = OFF;

        void ejectRingUp();
        void ejectRingDown();
        void colorSort();

        pros::Task task = pros::Task {[&] {
            Arm::position prevArmState;
            while (true) {                                                                                                                                                              
                if(this->state == STOPPED){
                    this->motor.brake();
                } else if(this->state == INTAKING){
                    // this->motor.move_velocity(INTAKE_SPEED);

                    // while (this->arm.getTargetPosition() == Arm::position::INTAKE && this->state == INTAKING) {
                    //     this->motor.move_velocity(INTAKE_SPEED);

                    //     while (this->holdSensor.get_proximity() < 80){
                    //         pros::delay(10);
                    //     }

                    //     this->motor.move_velocity(INTAKE_LDB_SPEED);
                    // }
                    if (this->arm.getTargetPosition() == Arm::position::INTAKE) {
                        this->motor.move_velocity(INTAKE_LDB_SPEED);
                    } else {
                        this->motor.move_velocity(INTAKE_SPEED);
                    }
                    if (!(sort==OFF)) { colorSort(); }
                    
                } else if(this->state == OUTTAKE){
                    this->motor.move_velocity(OUTTAKE_SPEED);
                }

                if ((this->arm.getTargetPosition() == Arm::position::UP || this->arm.getTargetPosition() == Arm::position::SCORE_NEUTRAL ||
                    this->arm.getTargetPosition() == Arm::position::SCORE_ALLIANCE) && prevArmState == Arm::position::INTAKE) {
                    
                    this->motor.move_velocity(-600);
                    pros::delay(150);
                    this->motor.move_velocity(0);
                }
                prevArmState = this->arm.getTargetPosition();

                pros::delay(10);
            }
        }};;
};
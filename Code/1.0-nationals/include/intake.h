#pragma once
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "arm.h"

#define INTAKE_SPEED 600
#define OUTTAKE_SPEED -400

class Intake {
    public:

        Intake( pros::Motor& motor , pros::Optical& topSort, Arm& arm);
        ~Intake() {this->task.remove();};

        enum IntakeState { STOPPED , INTAKING , OUTTAKE };
        enum SortState { RED , BLUE , OFF };

        void set(IntakeState state);
        void toggleState();
        void setState(SortState state);
        SortState getState();
        void hold(bool async = true);
        void waitUntilDone();
        
    private:
        pros::Motor& motor;
        pros::Optical& topSort;
        Arm& arm;

        IntakeState state = STOPPED;
        SortState sort = OFF;

        void ejectRing();
        void colorSort();

        pros::Task task = pros::Task {[&] {
            while (true) {                                                                                                                                                              
                if(this->state == STOPPED){
                    this->motor.brake();
                } else if(this->state == INTAKING){
                    this->motor.move_velocity(INTAKE_SPEED);
                    if (this->motor.get_efficiency() < 5 && this->arm.getTargetPosition() == Arm::position::INTAKE) { this->state = STOPPED; }
                } else if(this->state == OUTTAKE){
                    this->motor.move_velocity(OUTTAKE_SPEED);
                }
                pros::delay(10);

                if (!(sort==OFF)) { colorSort(); }
            }
        }};;
};
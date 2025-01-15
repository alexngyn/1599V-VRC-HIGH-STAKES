#include "intake.h"

Intake::Intake(pros::Motor& motor, pros::Optical& topSort, Arm& arm): motor(motor) , topSort(topSort) , arm(arm){};

void Intake::toggleState(){
    if (sort == SortState::OFF){
        sort = BLUE;
    }
    else if (sort == SortState::BLUE){
        sort = RED;
    }
    else if (sort == SortState::RED){
        sort = OFF;
    }
}

void Intake::setState(SortState state){
    this->sort = state;
}

Intake::SortState Intake::getState(){
    return this->sort;
}

void Intake::ejectRing(){
    double initPos = motor.get_position();

    while ((initPos + 480) - motor.get_position() > 10) {
        motor.move_velocity(600);
        pros::delay(10);
    
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motor.brake();
    pros::delay(50);
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
}

void Intake::colorSort(){ // private function
    if (sort == SortState::BLUE){
        if (topSort.get_hue()>200 && topSort.get_hue()<=270 &&topSort.get_proximity()<100){   //TUNE PROXIMITY // >0
            if (arm.getTargetAngle() == INTAKE){
                arm.moveTo(RETRACT);
                ejectRing();
                arm.moveTo(INTAKE);
            } else { ejectRing(); }
        }
    }
    else if (sort == SortState::RED){
        if (topSort.get_hue()<30 && topSort.get_hue()>=0 &&topSort.get_proximity()<100) {  //TUNE PROXIMITY //<40
            if (arm.getTargetAngle() == INTAKE){
                arm.moveTo(RETRACT);
                ejectRing();
                arm.moveTo(INTAKE);
            } else {ejectRing();}
        }
    }
}

void Intake::set(IntakeState state){
    this->state = state;
}
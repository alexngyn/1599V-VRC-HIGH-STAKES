#include "intake.h"

Intake::Intake(pros::Motor& motor, pros::Optical& topSort, Arm& arm): motor(motor) , topSort(topSort) , arm(arm){};

void Intake::toggleState(){
    if (sort == SortState::OFF){
        this->sort = BLUE;
    }
    else if (sort == SortState::BLUE){
        this->sort = RED;
    }
    else if (sort == SortState::RED){
        this->sort = OFF;
    }
}

void Intake::setState(SortState state){
    this->sort = state;
}

Intake::SortState Intake::getState(){
    return this->sort;
}

void Intake::ejectRing(){
    double initPos = this->motor.get_position();

    while ((initPos + 200) - this->motor.get_position() > 10) {
        this->motor.move(127);
        pros::delay(10);
    }

    this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    this->motor.brake();
    pros::delay(500);
    this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Intake::colorSort(){ // private function
    if (this->sort == SortState::RED){
        if (this->topSort.get_hue()>210 && this->topSort.get_hue()<=250 && this->topSort.get_proximity()<100){   //TUNE PROXIMITY // >0
            if (this->arm.getTargetPosition() == Arm::position::INTAKE){
                this->arm.moveTo(Arm::position::RETRACT);
                ejectRing();
                this->arm.moveTo(Arm::position::INTAKE);
            } else { ejectRing(); }
        }
    }
    else if (this->sort == SortState::BLUE){
        if ((this->topSort.get_hue()<10 || this->topSort.get_hue()>=350) && this->topSort.get_proximity()<100) {  //TUNE PROXIMITY //<40
            if (this->arm.getTargetPosition() == Arm::position::INTAKE){
                this->arm.moveTo(Arm::position::RETRACT);
                ejectRing();
                this->arm.moveTo(Arm::position::INTAKE);
            } else { ejectRing(); }
        }
    }
}

void Intake::hold(){
    this->state = INTAKING;

    while (this->topSort.get_proximity() < 100){
        pros::delay(20);
    }

    this->state = STOPPED;
}

void Intake::set(IntakeState state){
    this->state = state;
}
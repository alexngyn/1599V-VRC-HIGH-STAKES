#include "intake.h"
#include "setup.h"

/**
    * @brief Construct a new Intake:: Intake object
    * 
    * @param motor The motor that controls the intake
    * @param holdSensor The sensor that detects when a ring is in the intake
    * @param sortSensor The sensor that detects the color of the ring
    * @param arm The arm object that controls lady brown
    */

Intake::Intake(pros::Motor& motor,
               pros::Optical& holdSensor, 
               pros::Optical& sortSensor, 
               Arm& arm): 
               motor(motor), 
               holdSensor(holdSensor), 
               sortSensor(sortSensor), 
               arm(arm){};

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

// void Intake::ejectRing(){
//     double initPos = this->motor.get_position();

//     while ((initPos + 160) - this->motor.get_position() > 10) {
//         this->motor.move(127);
//         pros::delay(10);
//     }

//     this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     this->motor.brake();
//     pros::delay(500);
//     this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
// }

void Intake::ejectRingUp(){
    master.rumble("-");
    // double initPos = this->motor.get_position();

    this->motor.move_velocity(-600);
    pros::delay(200);
    this->motor.move_velocity(600);
    pros::delay(100);
    // this->motor.brake();
    // pros::delay(1000);
}

void Intake::ejectRingDown(){
    master.rumble("-");
    // double initPos = this->motor.get_position();

    this->motor.move_velocity(600);
    pros::delay(80);
     this->motor.move_velocity(-600);
    pros::delay(100);
    this->motor.move_velocity(600);
    pros::delay(100);
}

void Intake::colorSort(){ // private function
    // pros::c::optical_rgb_s_t rgb_value;
    // rgb_value = this->sortSensor.get_rgb();
    if (this->sortSensor.get_proximity() > 100) {
    if (this->sort == SortState::RED){
        if (this->sortSensor.get_hue()>100 && this->sortSensor.get_hue()<300) {  
            if (this->arm.getTargetPosition() == Arm::position::INTAKE){
                this->arm.moveTo(Arm::position::RETRACT);
                ejectRingDown();
                this->arm.moveTo(Arm::position::INTAKE);
            } else { ejectRingUp(); }
        }
    }
    else if (this->sort == SortState::BLUE){
        if (this->sortSensor.get_hue()<40 || this->sortSensor.get_hue()>=320) { 
            if (this->arm.getTargetPosition() == Arm::position::INTAKE){
                this->arm.moveTo(Arm::position::RETRACT);
                ejectRingDown();
                this->arm.moveTo(Arm::position::INTAKE);
            } else { ejectRingUp(); }
        }
    }
    }
}

void Intake::hold(bool async, int timeout){
    this->state = INTAKING;

    if (async) {
        pros::Task holdTask = pros::Task {[&] {
            while (this->holdSensor.get_proximity() < 80){
                pros::delay(10);
            }
            this->state = STOPPED;
        }};
    } else {
        while (this->holdSensor.get_proximity() < 80){
            pros::delay(10);
        }
        this->state = STOPPED;
    }
}

void Intake::holdldb(bool async, int timeout){
    this->state = INTAKING;

    if (!(this->arm.getTargetPosition() == Arm::position::INTAKE)) {
        this->arm.moveTo(Arm::position::INTAKE, true);
    }

    pros::Task holdTask = pros::Task {[&] {
            // while (this->holdSensor.get_proximity() < 100){
            //     pros::delay(20);
            // }

            // while (motor.get_efficiency() > 0){
            //     pros::delay(20);
            // }

            // while (true){
            //     if (this->motor.get_efficiency() < 0) {
            //         pros::delay(200);
            //         if (this->motor.get_efficiency() < 0) {
            //             break;
            //         }
            //     } pros::delay(20);
            // }

            // while (this->motor.get_efficiency() > 0) {
            //     pros::delay(20);

            //     if (this->motor.get_efficiency() < 5) {
            //         pros::delay(200);

            //         if (this->motor.get_efficiency() < 5) {
            //             break;
            //         }
            //     }
            // }

            while (this->motor.get_efficiency() > 3) {
                pros::delay(20);

                if (this->motor.get_efficiency() < 3) {
                    pros::delay(300);
                }
            }

            // this->state = OUTTAKE;
            // pros::delay(200);
            // double initPos = this->motor.get_position();

            // double error = (initPos + 40) - this->motor.get_position();

            // while (error > 0) {
            //     error = (initPos + 40) - this->motor.get_position();
            //     this->motor.move(20);
            //     pros::delay(10);
            // }

            //this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            //this->motor.brake();
            //pros::delay(500);
            //this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);


            this->state = STOPPED;
        }};

    if (!async) {waitUntilDone(timeout);}
}

void Intake::waitUntilDone(int timeout) {
    int initialTime = pros::millis();
    while (this->state != Intake::INTAKING) {
        pros::delay(50);
        if (pros::millis() - initialTime > timeout) { break; }
    }
}

void Intake::set(IntakeState state){
    this->state = state;
}
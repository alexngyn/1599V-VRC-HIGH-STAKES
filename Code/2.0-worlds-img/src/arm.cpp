#include "arm.h"
#include "pros/rtos.hpp"

/**
 * @brief Construct a new Arm object
 *
 * @param motors The motor group that controls the arm
 * @param rotation The rotation sensor that measures the arm's angle
 * @param pid The PID constants for the arm
 */

Arm::Arm(pros::MotorGroup* motors, pros::Rotation* rotation, lemlib::PID pid)
    : motors(motors),
      rotation(rotation),
      pid(pid) {
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD, 0);
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD, 1);
}  

void Arm::init() {
    this->task = new pros::Task([&] {
        double error;
        double vel;
        while (true) {  
            //std::printf("%d %f %f %f \n", pros::millis(), this->getAngle(), this->targetAngle, vel);

            this->targetAngle = angleStringToAngle(); 

            if (this->targetAngle == this->targetAngle) { // check if not NAN
                error = this->targetAngle - this->getAngle();

                if (std::fabs(error) <= 1 && this->targetPosition == position::INTAKE) {
                    this->currentState = Arm::state::HOLD;
                } else if (std::fabs(error) <= 2.5 && this->targetPosition == position::RETRACT) {
                    this->currentState = Arm::state::HOLD;
                } else if (std::fabs(error) <= 4 && (this->targetPosition != position::INTAKE || this->targetPosition != position::RETRACT)) {
                    this->currentState = Arm::state::HOLD;
                } else {
                    this->currentState = Arm::state::MOVING;
                }

                if (this->getAngle() > -90) {this->targetAngle=-110;}// soft limits 

                if (this->currentState == Arm::state::MOVING) {
                    vel = this->pid.update(error);

                    //if ((vel > 0 && this -> getAngle() < 180) || (vel < 0 && this -> getAngle() > 180)) { vel *= UpwardGain; } else {vel *= DownwardGain; }

                } else if (this->currentState == Arm::state::HOLD) {
                    vel = 0;
                }

                //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, error);
                //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, vel);
                // pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Arm: act pos %.1f | tgt pos %.1f | vel %.1f \n", this->getAngle(), this->targetAngle, vel);

                this->motors->move(vel);
            } // soft limits
            
            pros::delay(20);
        }
    } );
}

void Arm::setCustomSpeed(int speed) {
    if (speed != 0) {
        if (this->getAngle() > -90) { this->motors->brake();} else {
        this->motors->move_velocity(speed);}
        this->targetPosition = position::CUSTOM;
        this->targetAngle = NAN;
    } else if (targetAngle != targetAngle) { // if target angle is nan and speed is 0
        this->motors->brake();
        this->targetPosition = position::CUSTOM;
        this->targetAngle = this->getAngle();
    }
}

void Arm::moveTo(double angle, bool async, int timeout) {
    this->targetPosition = position::CUSTOM;
    this->targetAngle = angle;
    if (!async) { waitUntilDone(timeout); }
}

void Arm::moveTo(position pos, bool async, int timeout) {
    this->targetPosition = pos;
    if (!async) { waitUntilDone(timeout); }
}

void Arm::changeAngle(double deltaAngle) {
    this->targetPosition = position::CUSTOM;
    this->moveTo(this->targetAngle + deltaAngle);
}

double Arm::getAngle() {
    return this->rotation->get_position() * 0.01;
}

double Arm::getTargetAngle() {
    return this->targetAngle;
}

Arm::position Arm::getTargetPosition() {
    return this->targetPosition;
}

void Arm::waitUntilDone(int timeout) {
    int initialTime = pros::millis();
    while (this->currentState == Arm::state::MOVING) {
        pros::delay(10);
        if (pros::millis() - initialTime > timeout) { break; }
    }
}

double Arm::angleStringToAngle() {
    switch (this->targetPosition) {
        case position::RETRACT: return -340;
        case position::INTAKE: return -308;//08
        case position::UP: return -220;
        case position::SCORE_NEUTRAL: return -182;//190
        case position::SCORE_ALLIANCE: return -130;
        case position::TIP: return -100;
        //case position::CLIMB: return -170;
        default: return this->targetAngle;
    }
}

void Arm::togglePosition(position position1, position position2, 
                         position position3, position position4, position position5) {

    position prevPosition = this->targetPosition;
    // if (prevPosition == position::SCORE_NEUTRAL && position2 == position::SCORE_ALLIANCE) {
    //     this->targetPosition = position::SCORE_ALLIANCE;
    // } else if (prevPosition == position1) {
    //     this->targetPosition = position2;
    // } else if (prevPosition == position2) {
    //     if (position3 != position::NaV) {this->targetPosition = position3;}
    //     else {this->targetPosition = position1;}
    // } else if (prevPosition == position3) {
    //     if (position4 != position::NaV) {this->targetPosition = position4;}
    //     else {this->targetPosition = position1;}
    // } else if (prevPosition == position4) {
    //     if (position5 != position::NaV) {this->targetPosition = position5;}
    //     else {this->targetPosition = position1;}
    // } else if (prevPosition == position::CUSTOM ) {
    //     this->targetPosition = position1;
    // } else {
    //     this->targetPosition = position1;
    // }

    if (prevPosition == position::SCORE_NEUTRAL && position2 == position::SCORE_ALLIANCE) {
        this->targetPosition = position::SCORE_ALLIANCE;
    } else if (prevPosition == position1) {
        this->targetPosition = position2;
    } else if (prevPosition == position2 && position3 != position::NaV) {
        this->targetPosition = position3;
    } else if (prevPosition == position3 && position4 != position::NaV) {
        this->targetPosition = position4;
    } else if (prevPosition == position4 && position5 != position::NaV) {
        this->targetPosition = position5;
    } else {
        this->targetPosition = position1;
    }

    // Run unjam function if moving from INTAKE to UP or SCORE positions
    // if ((this->targetPosition == position::UP || this->targetPosition == position::SCORE_NEUTRAL || this->targetPosition == position::SCORE_ALLIANCE) && prevPosition == position::INTAKE) {
    //     // Implement the unjam logic here

    //     this->motors->move_relative(-10, 100); // Move back slightly
    //     pros::delay(100);
    //     this->motors->move_relative(10, 100); // Move forward slightly
    //     pros::delay(100);
    // }
}
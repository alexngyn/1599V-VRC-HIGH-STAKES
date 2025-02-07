#include "arm.h"

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
    pros::Task task = pros::Task {[&] {
    double error;
    double vel;
    while (true) {  
        this->targetAngle = angleStringToAngle(); 

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

        //if (this->getAngle() > -100) {this->targetAngle=-200;}// soft limits 

        if (this->currentState == Arm::state::MOVING) {
            vel = this->pid.update(error);

            //if ((vel > 0 && this -> getAngle() < 180) || (vel < 0 && this -> getAngle() > 180)) { vel *= UpwardGain; } else {vel *= DownwardGain; }
            
        } else if (this->currentState == Arm::state::HOLD) {
            vel = 0;
        }

        //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, error);
        //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, vel);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Arm: act pos %.1f | tgt pos %.1f | vel %.1f \n", this->getAngle(), this->targetAngle, vel);

        // FILE* usd_arm_file = fopen("/usd/log_arm.txt", "w");
        // fprintf(usd_arm_file, "%.1f,%.1f,%.1f\n", this->getAngle(), this->targetAngle, vel);
        // fclose(usd_arm_file);

        //printf("%d,%.1f,%.1f,%.1f,%.1f\n", pros::millis(), this->getAngle(), this->targetAngle, error, vel);

        this->motors->move(vel);

        pros::delay(20);
        }
    }};;
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
        case position::INTAKE: return -308;
        case position::UP: return -200;
        case position::SCORE_NEUTRAL: return -190;
        case position::SCORE_ALLIANCE: return -130;
        case position::CLIMB: return -170;
        default: return targetAngle;
    }
}

void Arm::togglePosition(position position1, position position2, 
                         position position3, position position4, position position5) {
    if (this->targetPosition == position1) {
        this->targetPosition = position2;
    } else if (this->targetPosition == position2) {
        if (position3 != position::CUSTOM) {this->targetPosition = position3;}
        else {this->targetPosition = position1;}
    } else if (this->targetPosition == position3) {
        if (position4 != position::CUSTOM) {this->targetPosition = position4;}
        else {this->targetPosition = position1;}
    } else if (this->targetPosition == position4) {
        if (position5 != position::CUSTOM) {this->targetPosition = position5;}
        else {this->targetPosition = position1;}
    } else {
        this->targetPosition = position1;
    }
}
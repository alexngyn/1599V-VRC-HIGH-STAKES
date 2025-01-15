#pragma once

#include "api.h"  // IWYU pragma: keep
//#include "setup.h"

#define SCORE_ALLIANCE 240
#define SCORE_NEUTRAL 140
#define UP 80
#define INTAKE 30
#define RETRACT 10

class Arm {
    public:
        Arm(pros::MotorGroup* motors, pros::Rotation* rotation, lemlib::PID pid);
        ~Arm() {this->task.remove();};

        enum class state {
            MOVING,
            HOLD
        };

        void moveTo(double angle);
        void changeAngle(double deltaAngle);
        double getAngle();
        double getTargetAngle();
        void home();

    private:
        pros::Rotation* rotation;
        pros::MotorGroup* motors;
        lemlib::PID pid;

        double targetAngle = 0; // init angle
        //float UpwardGain = 2.0;
        //float DownwardGain = 1.5;

        Arm::state currentState = Arm::state::HOLD;

        pros::Task task = pros::Task {[&] {
            while (true) {                                                                                                                                                              
                pros::delay(10);
                double error = lemlib::angleError(this->targetAngle, this->getAngle());

                //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, error);

                if (std::fabs(error) <= 10) {
                    this->currentState = Arm::state::HOLD;
                } else {
                    this->currentState = Arm::state::MOVING;
                }

                if (this->currentState == Arm::state::MOVING) {
                    double vel = this->pid.update(error);

                    //if ((vel > 0 && this -> getAngle() < 180) || (vel < 0 && this -> getAngle() > 180)) { vel *= UpwardGain; } else {vel *= DownwardGain; }

                    //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, vel);
                     this->motors->move(vel);
                } else if (this->currentState == Arm::state::HOLD) {
                    this->motors->move(0);
                }
            }
        }};;
};
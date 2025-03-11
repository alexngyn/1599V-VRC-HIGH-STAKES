#pragma once

#include "api.h"  // IWYU pragma: keep
//#include "intake.h"
//#include "setup.h"

class Arm {
    public:
        Arm(pros::MotorGroup* motors, pros::Rotation* rotation, lemlib::PID pid);

        enum class state {
            MOVING,
            HOLD
        };

        enum class position {
            RETRACT,
            INTAKE,
            UP,
            SCORE_NEUTRAL,
            SCORE_ALLIANCE,
            // CLIMB,
            TIP,
            CUSTOM,
            NaV // not a value
        };

        void moveTo(double angle, bool async = true, int timeout = 10000);
        void moveTo(position angle, bool async = true, int timeout = 10000);
        void changeAngle(double deltaAngle);
        double getAngle();
        double angleStringToAngle();
        double getTargetAngle();
        position getTargetPosition();
        void waitUntilDone(int timeout = 10000);
        void togglePosition(position position1, position position2, position position3 = position::NaV, 
                            position position4 = position::NaV,  position position5 = position::NaV);
        void init();
        void setCustomSpeed(int speed);

    private:
        void softLimits();
        pros::Rotation* rotation;
        pros::MotorGroup* motors;
        lemlib::PID pid;
        //Intake intake;

        position targetPosition = position::RETRACT;
        double targetAngle = this->angleStringToAngle(); 
        pros::Task* task;
        //float UpwardGain = 2.0;
        //float DownwardGain = 1.5;

        Arm::state currentState;
};
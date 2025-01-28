#pragma once

#include "api.h"  // IWYU pragma: keep
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
            CUSTOM
        };

        void moveTo(double angle, bool async = true);
        void moveTo(position angle, bool async = true);
        void changeAngle(double deltaAngle);
        double getAngle();
        double angleStringToAngle();
        double getTargetAngle();
        position getTargetPosition();
        void waitUntilDone();
        void togglePosition(position position1, position position2, position position3 = position::CUSTOM, 
                            position position4 = position::CUSTOM,  position position5 = position::CUSTOM);
        void init();

    private:
        pros::Rotation* rotation;
        pros::MotorGroup* motors;
        lemlib::PID pid;

        position targetPosition = position::RETRACT;
        double targetAngle = this->angleStringToAngle(); 
        //float UpwardGain = 2.0;
        //float DownwardGain = 1.5;

        Arm::state currentState;
};
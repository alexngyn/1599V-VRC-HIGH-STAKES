#include "intake.h"

Intake::Intake(pros::Motor motor, pros::Vision vision, lemlib::PID pid)
    : motor(std::move(motor)),
      vision(std::move(vision)),
      PID(pid)
      {}
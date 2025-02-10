#include "telemetry.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "setup.h"
#include "drive.h"

void flushBufferToFile(std::string name, std::vector<std::string>& buffer) {
    std::string path = "/usd/log_" + name + ".txt";
    FILE* file = fopen(path.c_str(), "w");

    for (const auto& data : buffer) {
        fprintf(file, "%s", data.c_str());
    }

    fclose(file);
    buffer.clear();
}

void writeToBuffer(const std::string& data, std::string name, std::vector<std::string>& buffer) {
    buffer.push_back(data + "\n");
    if (buffer.size() >= 500) { //20ms*25size is every 500ms
        flushBufferToFile(name, buffer);
        pros::delay(60000);
    }
}

void sdTelemetry() {
    std::vector<std::string> poseBuffer;
    std::vector<std::string> tempBuffer;


    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        writeToBuffer(std::to_string(pros::millis()) + "," +
                            std::to_string(pose.x) + "," + 
                            std::to_string(pose.y) + "," + 
                            std::to_string(pose.theta), "pose", poseBuffer);

        writeToBuffer(std::to_string(pros::millis()) + "," +
                            std::to_string(dt_left.get_temperature(0)) + "," + 
                            std::to_string(dt_left.get_temperature(1)) + "," + 
                            std::to_string(dt_left.get_temperature(2)) + "," +
                            std::to_string(dt_right.get_temperature(0)) + "," +
                            std::to_string(dt_right.get_temperature(1)) + "," +
                            std::to_string(dt_right.get_temperature(2)) + "," +
                            std::to_string(intake_motor.get_temperature()) + "," +
                            std::to_string(arm_motors.get_temperature(0)) + "," +
                            std::to_string(arm_motors.get_temperature(1)), "temp", tempBuffer);

        pros::delay(20);
    }
}

void screenTelemetry() {
    pros::lcd::initialize(); // initialize brain screen
    pros::c::optical_rgb_s_t rgb_value;

    pros::delay(20); // wait for initialization
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "x: %.3f    y: %.3f   theta: %.3f", pose.x, pose.y, pose.theta); // prints the x position
        //pros::screen::print(pros::E_TEXT_MEDIUM, 2, "", pose.y); // prints the y position

        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Intake: act vel %d | tgt vel  %d | eff %.1f \n", int(intake_motor.get_actual_velocity()), int(intake_motor.get_target_velocity()), intake_motor.get_efficiency());

        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "optical: %d %d %d %d", int(optical_sensor.get_hue()), int(optical_sensor.get_saturation()), int(optical_sensor.get_brightness()), int(optical_sensor.get_proximity()));

        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "left temp: %d %d %d", 
                            int(dt_left.get_temperature(0)),
                            int(dt_left.get_temperature(1)), 
                            int(dt_left.get_temperature(2)));

        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "right temp: %d %d %d", 
                            int(dt_right.get_temperature(0)),
                            int(dt_right.get_temperature(1)), 
                            int(dt_right.get_temperature(2)));

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "intake temp: %d", int(intake_motor.get_temperature()));
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "arm temp: %d %d", int(arm_motors.get_temperature(0)), int(arm_motors.get_temperature(1)));
        
        switch (intake_controller.getState()) {
            case Intake::SortState::BLUE: master.print(1, 1, "%s", "BLUE"); break;
            case Intake::SortState::RED: master.print(1, 1, "%s", "RED "); break;
            case Intake::SortState::OFF: master.print(1, 1, "%s", "OFF "); break;
            default: break;
        }

        pros::delay(100);

        master.print(2, 1, "%d", scaleValue); break;

        pros::delay(100);
    }
}
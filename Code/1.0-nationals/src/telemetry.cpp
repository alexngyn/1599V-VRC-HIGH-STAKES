#include "telemetry.h"
#include "pros/rtos.hpp"
#include "setup.h"

void flushBufferToFile(std::string name, std::vector<std::string>& buffer) {
    std::string path = "/usd/log_" + name + ".txt";
    FILE* file = fopen(path.c_str(), "w");
    if (file == nullptr) { return; }

    for (const auto& data : buffer) {
        fprintf(file, "%s", data.c_str());
    }

    fclose(file);
    buffer.clear();
}

void writeToBuffer(const std::string& data, std::string name, std::vector<std::string>& buffer) {
    buffer.push_back(data + "\n");
    if (buffer.size() >= 25) { //20ms*25size is every 500ms
        flushBufferToFile(name, buffer);
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
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "x: %.1f", pose.x); // prints the x position
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "y: %.1f", pose.y); // prints the y position
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "theta: %.1f", pose.theta); // prints the heading

        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "left temp: %d %d %d", 
                            dt_left.get_temperature(0),
                            dt_left.get_temperature(1), 
                            dt_left.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "right temp: %d %d %d", 
                            dt_right.get_temperature(0),
                            dt_right.get_temperature(1), 
                            dt_right.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "intake temp: %d", intake_motor.get_temperature());
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "arm temp: %d %d", arm_motors.get_temperature(0), arm_motors.get_temperature(1));

        switch (intake_controller.getState()) {
            case Intake::SortState::BLUE: master.print(1, 1, "%s", "BLUE"); break;
            case Intake::SortState::RED: master.print(1, 1, "%s", "RED "); break;
            case Intake::SortState::OFF: master.print(1, 1, "%s", "OFF "); break;
            default: break;
        }

        pros::delay(200);
    }
}
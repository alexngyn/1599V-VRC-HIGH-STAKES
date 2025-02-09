#include "telemetry.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "setup.h"

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
        pros::delay(100000);
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
        
        rgb_value = optical_sensor.get_rgb();
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "optical: %lf %lf %lf %d", rgb_value.red, rgb_value.green, rgb_value.blue, int(optical_sensor.get_proximity()));

        //pros::lcd::print(0, "x: %.3f    y: %.3f   theta: %.3f", pose.x, pose.y, pose.theta); // prints the position
        // pros::lcd::print(3, "left temp: %d %d %d", 
        //                     dt_left.get_temperature(0),
        //                     dt_left.get_temperature(1), 
        //                     dt_left.get_temperature(2));
        // pros::lcd::print(4, "right temp: %d %d %d", 
        //                     dt_right.get_temperature(0),
        //                     dt_right.get_temperature(1), 
        //                     dt_right.get_temperature(2));
        // pros::lcd::print(5, "intake temp: %d", intake_motor.get_temperature());
        // pros::lcd::print(6, "arm temp: %d %d", arm_motors.get_temperature(0), arm_motors.get_temperature(1));
        
        
        //pros::lcd::print(2, "Theta: %f", pose.theta);
        //pros::lcd::print(0, "%s %s auton", sideColor == color::red ? "red" : "blue"); // 0-2 0-14

        switch (intake_controller.getState()) {
            case Intake::SortState::BLUE: master.print(1, 1, "%s", "BLUE"); break;
            case Intake::SortState::RED: master.print(1, 1, "%s", "RED "); break;
            case Intake::SortState::OFF: master.print(1, 1, "%s", "OFF "); break;
            default: break;
        }

        pros::delay(200);
    }
}
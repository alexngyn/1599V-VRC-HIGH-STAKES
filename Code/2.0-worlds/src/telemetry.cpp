#include "telemetry.h"
#include "setup.h"
#include "drive.h"
#include <map>

void flushBufferToFile(std::string name, std::vector<std::string>& buffer) {
    std::string path = "/usd/log_" + name + ".txt";
    FILE* file = fopen(path.c_str(), "a"); //w or a

    for (const auto& data : buffer) {
        fprintf(file, "%s", data.c_str());
    }

    fclose(file);
    buffer.clear();
}

void writeToBuffer(const std::string& data, std::string name, std::vector<std::string>& buffer) {
    buffer.push_back(data + "\n");
    if (buffer.size() >= 50) { //20ms*25size is every 500ms
        flushBufferToFile(name, buffer);
        // pros::delay(60000);
    }
}

void sdTelemetry() {
    std::vector<std::string> poseBuffer;
    std::vector<std::string> tempBuffer;
    fclose(fopen("pose_log.txt", "w")); // create new file
    fclose(fopen("temp_log.txt", "w"));

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
    // brain screeen
    // pros::lcd::initialize(); // initialize brain screen
    //pros::c::optical_rgb_s_t rgb_value;

    // pros::delay(20); // wait for initialization
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "x: %.3f    y: %.3f   theta: %.3f", pose.x, pose.y, pose.theta); // prints the x position
        //pros::screen::print(pros::E_TEXT_MEDIUM, 2, "", pose.y); // prints the y position

        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Intake: act vel %d | tgt vel  %d | eff %.1f \n", int(intake_motor.get_actual_velocity()), int(intake_motor.get_target_velocity()), intake_motor.get_efficiency());

        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "optical: %03d %03.1f %03.1f %03d", int(optical_sensor.get_hue()), optical_sensor.get_saturation(), optical_sensor.get_brightness(), optical_sensor.get_proximity());
        // pros::screen::print(pros::E_TEXT_MEDIUM, 3, "optical: %d", optical_sensor.get_proximity());
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

        pros::delay(100);
    }
}

void controllerTelemetry() {
    pros::delay(20); // wait for initialization
    while (true) {
        //      controller screen
        switch (intake_controller.getState()) {
            case Intake::SortState::BLUE: master.print(1, 1, "%s", "BLUE"); break;
            case Intake::SortState::RED: master.print(1, 1, "%s", "RED "); break;
            case Intake::SortState::OFF: master.print(1, 1, "%s", "OFF "); break;
            default: break;
        }

        pros::delay(100);

        master.print(2, 1, "%03d", scaleValue);

        pros::delay(100);
    }
}

static pros::Task* screenTelemetryTask = nullptr;

/*

LV_IMG_DECLARE(lf);
LV_IMG_DECLARE(sc);
// LV_IMG_DECLARE(typ);
LV_IMG_DECLARE(abt);
// LV_IMG_DECLARE(allience);
LV_IMG_DECLARE(banana);
LV_IMG_DECLARE(ssis);

// Map image names to LVGL image descriptors
std::map<std::string, const lv_img_dsc_t*> imageMap = {
    {"sc", &sc},
    // {"lf", &lf},
    // {"typ", &typ},
    // {"abt", &abt},
    // {"ssis", &ssis},
    // {"banana", &banana},
    // {"allience", &allience}

};

void printImage(const char* img_name) {
    auto it = imageMap.find(img_name);
    if (it == imageMap.end()) {
        std::cerr << "Error: Image '" << img_name << "' not found!\n";
        return;
    }

    lv_obj_t* image = lv_img_create(lv_scr_act());
    lv_img_set_src(image, it->second);
    lv_obj_set_size(image, 480, 240);
    lv_obj_align(image, LV_ALIGN_CENTER, 0, 0);
}

static int imgIndex = 0;
static std::vector<const char*> imgNames;

// void switchImage() {
//     if (imgNames.empty()) {
//         for (const auto& pair : imageMap) {
//             imgNames.push_back(pair.first.c_str());
//         }
//     }
    
//     printImage(imgNames[imgIndex]);
//     imgIndex = (imgIndex + 1) % imgNames.size();
// }

void switchScreen() {
    if (imgNames.empty()) {
        for (const auto& pair : imageMap) { imgNames.push_back(pair.first.c_str()); }
    }

    if (imgIndex == imgNames.size()) {
        
        printImage(imgNames[imgIndex]);
        imgIndex = 0;
        //lv_obj_clean(lv_scr_act());
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
        screenTelemetryTask->resume();
    } else {
        if (screenTelemetryTask->get_state() == pros::E_TASK_STATE_RUNNING) { screenTelemetryTask->suspend(); }
        printImage(imgNames[imgIndex]);
        // imgIndex = (imgIndex + 1) % imgNames.size();
        imgIndex++;
    }
}

void screenImage() {
    printImage("sc");
    // screenTelemetryTask->suspend();
    pros::screen::touch_callback(switchScreen, pros::E_TOUCH_PRESSED);
}

*/

void telemetryInit() {
    screenTelemetryTask = new pros::Task(screenTelemetry); // screenTelemetryTask->suspend();
    // screenImage();
    pros::Task controllerScreenTask(controllerTelemetry);
    pros::Task sdTask(sdTelemetry);
}
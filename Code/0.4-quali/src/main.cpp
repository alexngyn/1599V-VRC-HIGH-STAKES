/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "autonomous.h"
#include "pros/motors.h"
#include "setup.h"

void check_device_plugged_in(int port, std::string deviceName)
{
	if (pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::none ||
		pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::undefined)
	{
		master.rumble("---");
	}
}

void initialize() {
    // partner.clear();
    //partner.print(0, 0, "init start"); // 0-2 0-14
    chassis.calibrate(); // calibrate the chassis
    arm_rotational_sensor.reset(); // reset the arm sensor

    //console.println("Checking device status...");
	// check_device_plugged_in(inertial_sensor.get_port(), "IMU");
	// check_device_plugged_in(dt_left.get_port(0), "Left Motor Group 1");
	// check_device_plugged_in(dt_left.get_port(1), "Left Motor Group 2");
	// check_device_plugged_in(dt_left.get_port(2), "Left Motor Group 3");
	// check_device_plugged_in(dt_right.get_port(0), "Right Motor Group 1");
	// check_device_plugged_in(dt_right.get_port(1), "Right Motor Group 2");
	// check_device_plugged_in(dt_right.get_port(2), "Right Motor Group 3");
    // check_device_plugged_in(intake_motor.get_port(), "Intake Motor");
	// check_device_plugged_in(arm_motor.get_port(), "Arm Motor");
	//check_device_plugged_in(horizontal_encoder.get_port(), "Horizontal Encoder");
    
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    pros::lcd::initialize(); // initialize brain screen
    
    pros::Task positionprint([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose(); // get chassis position
            partner.print(0, 0, "%.1f %.1f %.1f", pose.x, pose.y, pose.theta); // 0-2 0-14
            pros::delay(50);
            partner.print(1, 0, "%.1f %s", arm_controller.getAngle(), arm_controller.isInPosition() ? "stopped" : " moving"); // 0-2 0-14
            pros::delay(50);
            partner.print(2, 0, "%s %s ej:%s", sideColor == red ? "red" : "blue", ejectEnabled ? "on" : "off"); // 0-2 0-14
            pros::delay(50);
            pros::lcd::print(3,"opotical distance: %i", optical_sensor.get_proximity());
            pros::delay(500);
        }
    });
    
    pros::lcd::initialize(); // initialize brain screen
    pros::Task screen([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose(); // get chassis position
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::delay(50);
        }
    });

    // pros::lcd::print(0, "%s %s auton", sideColor == red ? "red" : "blue"); // 0-2 0-14

    pros::delay(500);

    //lemlib::Pose testpose = chassis.getPose();
    // if (testpose.theta != testpose.theta) {
    //     console.println("Initializing failed...");
    //     rd_view_alert(getSelectorView(selector), "Initializing failed...");
    // } else {
    //     console.println("Initializing successful...");
    // }

    //pros::Task update_ui_thread(update_ui);
    //pros::Task lights([&] { led1.rotate(); pros::delay(100); });
    //imageTest();

    //partner.print(0, 0, "init done"); // 0-2 0-14
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    
    // partner.print(0, 0, "auton start"); // 0-2 0-14
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //partner.print(0, 0, "auton done"); // 0-2 0-14
    skills();

}

void printDistance(){
    while (true){
        pros::lcd::print(0,"distance: %i", optical_sensor.get_proximity());
        pros::delay(500);
    }
}
struct rgb {
  double r;
  double g;
  double b;
};

uint32_t ledbuffer[LED_1_LENGTH];
std::vector<uint32_t> ledbuffer_v;

std::uint32_t rgb_to_hex(int r, int g, int b) {
    return (((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff));
}

rgb hex_to_rgb(std::uint32_t color) {
    rgb in;
    in.r = (color >> 16) & 0xff;
    in.g = (color >> 8) & 0xff;
    in.b = color & 0xff;
    return in;
}

uint32_t interpolate_rgb(std::uint32_t start_color, std::uint32_t end_color, int step,
                                       int fade_width) {
    rgb startComponents = hex_to_rgb(start_color);
    rgb endComponents = hex_to_rgb(end_color);

    double red_diff = endComponents.r - startComponents.r;
    double green_diff = endComponents.g - startComponents.g;
    double blue_diff = endComponents.b - startComponents.b;

    double red_step = red_diff / fade_width;
    double green_step = green_diff / fade_width;
    double blue_step = blue_diff / fade_width;

    rgb solved;

    solved.r = (startComponents.r + red_step * step);
    solved.g = (startComponents.g + green_step * step);
    solved.b = (startComponents.b + blue_step * step);
    return rgb_to_hex(solved.r, solved.g, solved.b);
}

void gradient(std::uint32_t start_color, std::uint32_t end_color, int fade_width) {
    for (int i = 0; i < fade_width; i++) {
    	ledbuffer_v[i] = interpolate_rgb(start_color, end_color, i, fade_width);
    }
	for (int i = fade_width; i < fade_width*2; i++) {
    	ledbuffer_v[i] = interpolate_rgb(end_color, start_color, i, fade_width);
    }
}

void ledsetup() {

    // for(int i = 0;i<LED_1_LENGTH;i++){
	// 	ledbuffer_v.push_back(0x00FF00);
	// }
	// pros::c::adi_led_t led = pros::c::adi_led_init(LED_1_PORT);
	// pros::delay(500);

	// gradient(0xFFDA29, 0xC40233, 30);

	// std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);
	// pros::c::adi_led_set(led, ledbuffer, LED_1_LENGTH);
	// while (true) {
	// 	// std::rotate(ledbuffer.begin(), ledbuffer.end() - 1, ledbuffer.end());
	// 	std::rotate(ledbuffer_v.begin(), ledbuffer_v.begin() + 1, ledbuffer_v.end());
	// 	std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);
	// 	pros::c::adi_led_set(led, ledbuffer, LED_1_LENGTH);
	// 	pros::delay(200);

    // }
    led led_1 (LED_1_PORT, LED_1_LENGTH);
    //pros::delay(1000);
    //led led_2 (LED_2_PORT, LED_2_LENGTH);

    // pros::Task lights([&] { 
    //     led_1.rotate(); 
    //     //led_2.rotate(); 
    //     pros::delay(200);
    // });
}

void opcontrol() {
    master.clear();
    // console.println("Running op...");
    partner.print(0, 0, "op start"); // 0-2 0-14
    arm_controller.moveToAngle(12);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	pros::Task drive_thread(drive);
	pros::Task intake_thread(intake);
    pros::Task clamp_thread(clamp);
    pros::Task topmech_thread(topmech);
    pros::Task doinker_thread(doinker);
    ledsetup();
}
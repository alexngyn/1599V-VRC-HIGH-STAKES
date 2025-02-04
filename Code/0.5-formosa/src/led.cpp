#include "led.h"
#include "setup.h"

uint32_t ledbuffer[LED_1_LENGTH];
std::vector<uint32_t> ledbuffer_v;
// uint32_t ledbuffer2[LED_2_LENGTH];
// std::vector<uint32_t> ledbuffer2_v;

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

void gradient(std::uint32_t start_color, std::uint32_t end_color, int fade_width, std::vector<uint32_t>* ledbufferv) {
    for (int i = 0; i < fade_width; i++) {
        (*ledbufferv)[i] = interpolate_rgb(start_color, end_color, i, fade_width);
    }
    for (int i = fade_width; i < fade_width * 2; i++) {
        (*ledbufferv)[i] = interpolate_rgb(end_color, start_color, i - fade_width, fade_width);
    }
}

void ledsetup() {
    int start_time = pros::millis();

    for(int i = 0;i<LED_1_LENGTH;i++){
		ledbuffer_v.push_back(0xFF0000);
	}
	pros::c::adi_led_t led = pros::c::adi_led_init(LED_1_PORT);

    // for(int i = 0;i<LED_2_LENGTH;i++){
	// 	ledbuffer_v.push_back(0xFF0000);
	// }
	pros::c::adi_led_t led2 = pros::c::adi_led_init(LED_2_PORT);
	pros::delay(200);

	gradient(0xFFDA29, 0xC40233, 30, &ledbuffer_v);
    // gradient(0xFFDA29, 0xC40233, 30, &ledbuffer2_v);
    //std::vector<uint32_t> ledbuffer2_v(ledbuffer_v);

	std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);
	pros::c::adi_led_set(led, ledbuffer, LED_1_LENGTH);

    pros::delay(50);

    // std::copy(ledbuffer2_v.begin(), ledbuffer2_v.end(), ledbuffer2);
	pros::c::adi_led_set(led2, ledbuffer, LED_2_LENGTH);

    // pros::delay(50);

	while (pros::millis() - start_time < 75000) { // 1:30
		std::rotate(ledbuffer_v.begin(), ledbuffer_v.begin() + 1, ledbuffer_v.end());
		std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);
		pros::c::adi_led_set(led, ledbuffer, LED_1_LENGTH);
		pros::delay(50);

        // std::rotate(ledbuffer2_v.begin(), ledbuffer2_v.begin() + 1, ledbuffer2_v.end());
		// std::copy(ledbuffer2_v.begin(), ledbuffer2_v.end(), ledbuffer2);
		pros::c::adi_led_set(led2, ledbuffer, LED_2_LENGTH);
		pros::delay(50);

        //if (true) {pros::lcd::print(4, "Theta: %f", intake_motor.get_temperature());}  intake_motor.is_over_temp()
        if (intake_motor.get_temperature() > 45) { pros::c::adi_led_set_all(led2, ledbuffer, LED_2_LENGTH, 0xFFFF00); 
            while (intake_motor.get_temperature() > 45) { pros::delay(200); } }
        
    }

    for (int i = 0; i < floor(static_cast<float>(LED_1_LENGTH) / 3); i++) {
        (ledbuffer_v)[i*3] = 0xFF4D00;
    }

    for (int i=0; i<3; i++) {
        std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);
        pros::c::adi_led_set(led, ledbuffer, LED_2_LENGTH);
        pros::delay(25);
        pros::c::adi_led_set(led2, ledbuffer, LED_2_LENGTH);
        pros::delay(25);
        pros::c::adi_led_clear_all(led, ledbuffer, LED_1_LENGTH);
        pros::delay(25);
        pros::c::adi_led_clear_all(led2, ledbuffer, LED_2_LENGTH);
        pros::delay(25);
    }

    while (pros::millis() - start_time < 85000) {pros::delay(100); } // 1:20

    for (int i=0; i<10; i++) {
        std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);
        pros::c::adi_led_set(led, ledbuffer, LED_2_LENGTH);
        pros::delay(50);
        pros::c::adi_led_set(led2, ledbuffer, LED_2_LENGTH);
        pros::delay(50);
        pros::c::adi_led_clear_all(led, ledbuffer, LED_1_LENGTH);
        pros::delay(50);
        pros::c::adi_led_clear_all(led2, ledbuffer, LED_2_LENGTH);
        pros::delay(50);
    }
    
    pros::c::adi_led_set_all(led, ledbuffer, LED_1_LENGTH, 0xFF4D00);
    pros::delay(50);
    pros::c::adi_led_set_all(led2, ledbuffer, LED_2_LENGTH, 0xFF4D00);

    while (pros::millis() - start_time < 102000) {pros::delay(100); } // 1:42
    while (true) {
        pros::c::adi_led_set_all(led, ledbuffer, LED_1_LENGTH, 0xFF4D00);
        pros::delay(50);
        pros::c::adi_led_set_all(led2, ledbuffer, LED_2_LENGTH, 0xFF4D00);
        pros::delay(450);
        pros::c::adi_led_clear_all(led, ledbuffer, LED_1_LENGTH);
        pros::delay(50);
        pros::c::adi_led_clear_all(led2, ledbuffer, LED_2_LENGTH);
        pros::delay(450);
    }
}
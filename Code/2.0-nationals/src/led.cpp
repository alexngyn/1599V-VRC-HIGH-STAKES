#include "led.h"
#include "setup.h"
#include <cstdint>
#include <sys/types.h>

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

void setAllToBuffer(pros::c::adi_led_t& led1, uint32_t* ledbuffer1, int length1, uint8_t delay1,
            pros::c::adi_led_t& led2, uint32_t* ledbuffer2, int length2, uint8_t delay2) {
    pros::c::adi_led_set(led1, ledbuffer1, length1);
    pros::delay(delay1);

    pros::c::adi_led_set(led2, ledbuffer2, length2);
    pros::delay(delay2);
}

void setAllToColor(pros::c::adi_led_t& led1, int length1, uint8_t delay1, uint32_t color1,
            pros::c::adi_led_t& led2, int length2, uint8_t delay2, uint32_t color2) {
    pros::c::adi_led_set_all(led1, ledbuffer, length1, color1);
    pros::delay(delay1);

    pros::c::adi_led_set_all(led2, ledbuffer, length2, color2);
    pros::delay(delay2);
}

void clearAll(pros::c::adi_led_t& led1, uint32_t* ledbuffer1, int length1, uint8_t delay1,
            pros::c::adi_led_t& led2, uint32_t* ledbuffer2, int length2, uint8_t delay2) {
    pros::c::adi_led_clear_all(led1, ledbuffer1, length1);
    pros::delay(delay1);

    pros::c::adi_led_clear_all(led2, ledbuffer2, length2);
    pros::delay(delay2);
}

void ledsetup() {
    int start_time = pros::millis();

    for(int i = 0;i<LED_1_LENGTH;i++){
		ledbuffer_v.push_back(0xFF0000);
	}
	pros::c::adi_led_t led = pros::c::adi_led_init(LED_1_PORT);

	pros::c::adi_led_t led2 = pros::c::adi_led_init(LED_2_PORT);
	pros::delay(200);

	gradient(0xFFDA29, 0xC40233, 30, &ledbuffer_v);

	std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);
	
    setAllToBuffer(led, ledbuffer, LED_1_LENGTH, 50, 
                led2, ledbuffer, LED_2_LENGTH, 50);

	pros::c::adi_led_set(led2, ledbuffer, LED_2_LENGTH);

    // pros::delay(50);

	while (pros::millis() - start_time < 600000000) { // 0:15-1:15
		std::rotate(ledbuffer_v.begin(), ledbuffer_v.begin() + 1, ledbuffer_v.end());
		std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);

        setAllToBuffer(led, ledbuffer, LED_1_LENGTH, 50, 
                led2, ledbuffer, LED_2_LENGTH, 50);

        if (intake_motor.get_temperature() > 45) { 
            setAllToColor(led, LED_1_LENGTH, 25, 0xFFFF00, //yellow
                    led2, LED_2_LENGTH, 0, 0xFFFF00);
            while (intake_motor.get_temperature() > 45) { pros::delay(200); } 
        }
        
    }

    for (int i = 0; i < floor(static_cast<float>(LED_1_LENGTH) / 3); i++) { // set every third led to red 
        (ledbuffer_v)[i*3] = 0xFF4D00;
    }

    for (int i=0; i<3; i++) { // flash red 3 times
        std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);

        setAllToBuffer(led, ledbuffer, LED_1_LENGTH, 25, 
                led2, ledbuffer, LED_2_LENGTH, 25);

        clearAll(led, ledbuffer, LED_1_LENGTH, 25, 
                led2, ledbuffer, LED_2_LENGTH, 25);
    }

    while (pros::millis() - start_time < 70000) {pros::delay(100); } // 1:25 release mogo

    for (int i=0; i<10; i++) { // flash red slowly
        std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);

        setAllToBuffer(led, ledbuffer, LED_1_LENGTH, 50, 
                led2, ledbuffer, LED_2_LENGTH, 50);

        clearAll(led, ledbuffer, LED_1_LENGTH, 50, 
                led2, ledbuffer, LED_2_LENGTH, 50);
    }
    
    setAllToColor(led, LED_1_LENGTH, 25, 0xFF4D00, 
                led2, LED_2_LENGTH, 0, 0xFF4D00);

    while (pros::millis() - start_time < 102000) {pros::delay(100); } // 1:57
    while (true) { // 
        std::copy(ledbuffer_v.begin(), ledbuffer_v.end(), ledbuffer);

        setAllToBuffer(led, ledbuffer, LED_1_LENGTH, 25, 
                led2, ledbuffer, LED_2_LENGTH, 25);

        clearAll(led, ledbuffer, LED_1_LENGTH, 25, 
                led2, ledbuffer, LED_2_LENGTH, 25);
    }
}
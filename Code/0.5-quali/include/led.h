#pragma once

#include "api.h"  // IWYU pragma: keep

class led {
public:
    // Constructor
    led(const char port, int length);

    // Destructor to clean up dynamic memory
    ~led();

    // Member variables
    const char port;
    const int length;
    uint32_t* ledbuffer;
    std::vector<uint32_t> ledbuffer_v;
    pros::c::adi_led_t ledstrip;

    struct rgb {
        double r;
        double g;
        double b;
    };

    std::uint32_t rgb_to_hex(int r, int g, int b);
    rgb hex_to_rgb(std::uint32_t color);
    uint32_t interpolate_rgb(std::uint32_t start_color, std::uint32_t end_color, int step, int fade_width);
    void gradient(std::uint32_t start_color, std::uint32_t end_color, int fade_width);
    void update();
    void rotate();
};
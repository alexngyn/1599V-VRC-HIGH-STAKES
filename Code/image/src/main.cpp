/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "pros/rtos.hpp"
#include <map>
// #include "gif-pros/gifclass.hpp"

pros::Controller master (pros::E_CONTROLLER_MASTER);

static pros::Task* init_task = nullptr;

// Declare your images
LV_IMG_DECLARE(lf);
LV_IMG_DECLARE(sc);
LV_IMG_DECLARE(typ);
LV_IMG_DECLARE(abt);
LV_IMG_DECLARE(allience);
LV_IMG_DECLARE(banana);
LV_IMG_DECLARE(ssis);

// Map image names to LVGL image descriptors
std::map<std::string, const lv_img_dsc_t*> imageMap = {
    {"sc", &sc},
    {"lf", &lf},
    {"typ", &typ},
    {"abt", &abt},
    {"ssis", &ssis},
    {"banana", &banana},
    {"allience", &allience}

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

void switchImage() {
    if (imgNames.empty()) {
        for (const auto& pair : imageMap) {
            imgNames.push_back(pair.first.c_str());
        }
    }
    
    printImage(imgNames[imgIndex]);
    imgIndex = (imgIndex + 1) % imgNames.size();
}

void initialize() {
    // pros::delay(100);
    printImage("lf");
    pros::screen::touch_callback(switchImage, pros::E_TOUCH_PRESSED);

    // if (init_task == nullptr) { init_task = new pros::Task([&]() {
    //     while (true) {
    //         if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { switchImage(); }
            
    //         pros::delay(20);
    //     }
    // });}

    // this block can go out of scope
    // static Gif gif("/usd/funny.gif", lv_scr_act());
    
    pros::delay(1000);
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    
}

void opcontrol() {
    // Gif* gif = new Gif("/usd/funny.gif", lv_scr_act()); 
    // gif->resume();
    // lv_obj_t* obj = lv_obj_create(lv_scr_act(), NULL);
    // lv_obj_set_size(obj, 480, 240);
    // lv_obj_set_style(obj, &lv_style_transp); // make the container invisible
    // lv_obj_align(obj, NULL, LV_ALIGN_CENTER, 0, 0);

    // Gif gif("/usd/mygif.gif", obj);

    // pros::delay(10000);
}
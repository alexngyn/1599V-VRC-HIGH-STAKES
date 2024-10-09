// File: src/uiLIB/views/odom.cpp
// @Brief: Odometry view implementation
// @Credit: Using Robodash's View, also partially using Lemlib's Pose functions
// @Description: Image of field, robot that updates position, and stats of robot's pose on the right
#include "views/odomView.hpp"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/widgets/lv_img.h"
#include "liblvgl/widgets/lv_label.h"
#include "main.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "lemlib/asset.hpp"
#include "assets/fieldResized.c"
#include "assets/robot.c"
#include "assets/disconnected.c"
#include "assets/autonomous.c"
#include "assets/op.c"
#include "assets/disabled.c"
#include "assets/enabled.c"
#include "assets/robot.c"
#include "liblvgl/font/lv_font.h"
#include "lemlib/chassis/chassis.hpp"


rd::OdomView::OdomView() {
    lv_point_t line_points[]= {{0, 0}, {0, 0}};
    

    //creating view named odomview
    this->view = rd_view_create("odomview");
    lv_style_init(&styleTitle);
    lv_style_init(&styleLabel);
    lv_style_init(&styleLine);
    lv_style_init(&styleLabelVar);
    //setting variables
    this->x = 0;
    this->y = 0;
    this->theta = 0;
    this->timer = 0;
    
    //set style
    lv_style_set_text_font(&styleTitle, &lv_font_montserrat_20);
    lv_style_set_text_font(&styleLabel, &lv_font_montserrat_18);
    lv_style_set_text_color(&styleLabelVar, lv_palette_main(LV_PALETTE_GREY));

    //creating images
    field = lv_img_create(view->obj);
    lv_img_set_src(field, &fieldResized);
    robot = lv_img_create(view->obj);
    lv_img_set_src(robot, &robotL);
    lv_obj_set_pos(robot,50,50);
    lv_obj_move_foreground(robot);
    //creating changing images

    // MatchState = lv_img_create(view->obj);
    // lv_img_set_src(MatchState, &disconnected);
    // lv_obj_set_x(MatchState, 314);
    // lv_obj_set_y(MatchState, 189);
    // DisabledState = lv_img_create(view->obj);
    // lv_img_set_src(DisabledState, &disconnected);
    // lv_obj_set_x(DisabledState, 364);
    // lv_obj_set_y(DisabledState, 189);
    
    //creating labels
    labelTitle = lv_label_create(view->obj);
    lv_label_set_text(labelTitle, "Odometry");
    lv_obj_add_style(labelTitle,&styleTitle,LV_PART_MAIN);
    lv_obj_set_x(labelTitle, 310);
    lv_obj_set_y(labelTitle, 16);
    labelX = lv_label_create(view->obj);
    lv_obj_add_style(labelX,&styleLabel,LV_PART_MAIN);
    lv_label_set_text(labelX, "X");
    lv_obj_set_x(labelX, 302);
    lv_obj_set_y(labelX, 84);
    labelY = lv_label_create(view->obj);
    lv_obj_add_style(labelY,&styleLabel,LV_PART_MAIN);
    lv_label_set_text(labelY, "Y");
    lv_obj_set_x(labelY, 302);
    lv_obj_set_y(labelY, 109);
    labelTheta = lv_label_create(view->obj);
    lv_obj_add_style(labelTheta,&styleLabel,LV_PART_MAIN);
    lv_label_set_text(labelTheta, "θ");
    lv_obj_set_x(labelTheta, 302);
    lv_obj_set_y(labelTheta, 134);

    //creating variable labels
    labelXVar= lv_label_create(view->obj);
    lv_obj_add_style(labelXVar,&styleLabelVar,LV_PART_MAIN);
    lv_label_set_text(labelXVar, "0");
    lv_obj_set_x(labelXVar, 322);
    lv_obj_set_y(labelXVar, 84);
    lv_label_set_recolor(labelXVar,true) ;
    labelYVar = lv_label_create(view->obj);
    lv_obj_add_style(labelYVar,&styleLabelVar,LV_PART_MAIN);
    lv_label_set_text(labelYVar, "0");
    lv_obj_set_x(labelYVar, 322);
    lv_obj_set_y(labelYVar, 109);
    lv_label_set_recolor(labelYVar,true) ;
    labelThetaVar = lv_label_create(view->obj);
    lv_label_set_recolor(labelThetaVar,true) ;
    lv_obj_add_style(labelThetaVar,&styleLabelVar,LV_PART_MAIN);
    lv_label_set_text(labelThetaVar, "0");
    lv_obj_set_x(labelThetaVar, 322);
    lv_obj_set_y(labelThetaVar, 134);
    

    // labelXVar = lv_label_create(view->obj);
    // lv_label_set_text(labelXVar, "0");
    // labelYVar = lv_label_create(view->obj);
    // lv_label_set_text(labelYVar, "0");
    // labelThetaVar = lv_label_create(view->obj);
    // lv_label_set_text(labelThetaVar, "0");

    // //creating timer label
    // timerVarMin = lv_label_create(view->obj);
    // timerVarSec = lv_label_create(view->obj);
    // lv_label_set_text(timerVarMin, "0:");
    // lv_label_set_text(timerVarSec, "00");
    
    //creating lines styles

    lv_style_set_line_width(&styleLine, 30);
    lv_style_set_line_rounded(&styleLine, true);
    lv_style_set_line_color(&styleLine, lv_palette_main(LV_PALETTE_BLUE));
    //creating lines
    // line1 = lv_line_create(view->obj);
    // lv_line_set_points(line1, line_points, 2);
    // lv_obj_add_style(line1,&styleLine,0);
    

}
void rd::OdomView::update(lemlib::Chassis* chassis){
    this->x = chassis->getPose().x;
    this->y = chassis->getPose().y;
    this->theta = chassis->getPose().theta;
    timer = pros::millis();
    timerMin = (timer/1000)/60;
    timerSec = (timer/1000)%60;
}

void rd::OdomView::updateLabels(){
    lv_label_set_text(labelXVar, (std::to_string(x)).c_str());
    lv_label_set_text(labelYVar, (std::to_string(y)).c_str());
    lv_label_set_text(labelThetaVar, (std::to_string(theta)).c_str());
    lv_label_set_text(timerVarMin, (std::to_string(timerMin)).c_str());
    lv_label_set_text(timerVarSec, (std::to_string(timerSec)).c_str());
    
}
void rd::OdomView::updateImage(){
    lv_obj_set_pos(robot, (x+180)*0.69, (y*-1+180)*0.69);
    lv_img_set_angle(robot, theta*100);

}

void rd::OdomView::focus(){
    rd_view_focus(this->view);
}
#include "gui.h"
#include "setup.h"
//#include "assets/imageTest.c"
//#include "assets/fieldResized.c"
//#include "assets/license.c"

// Robodash selector initialization...
// rd::Selector selector({{"Red Left", &pidtune},
// 				   {"Red Right", &auton_red_non_rush},
// 				   {"Blue Left", &auton_red_non_rush},
// 					   {"Skills", &skills}});

//rd::Image img("assets/license.c");

// void imageTest() {
// 	rd_view_t *view = rd_view_create("imageTest");
// 	lv_obj_t *image = lv_img_create(view->obj);
// 	lv_img_set_src(image, &fieldResized);

//    //lv_label_t *label = lv_label_create(rd_view_obj(view));
//    //lv_label_set_text(label, "example");
//    //lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
// }


// void rd::imageTest::focus(){
//     rd_view_focus(this->view);
// }

// Robodash console initialization...
// rd::Console console;

/*
// Creates the custom pages
rd_view_t *view = rd_view_create("Info");
rd_view_t *auto_override = rd_view_create("Override");

int update_ui()
{
	// Creates the UI elements
	lv_obj_t *override_btn = lv_btn_create(rd_view_obj(auto_override));
	lv_obj_center(override_btn);
	lv_obj_add_event_cb(override_btn, [](lv_event_t *e)
						{
		if (lv_event_get_code(e) == LV_EVENT_CLICKED){
			rd_view_alert(view, "Running autonomous...");
			selector.run_auton();
		} }, LV_EVENT_ALL, NULL);
	lv_obj_t *override_label = lv_label_create(override_btn);
	lv_label_set_text(override_label, "Run Selected Autonomous");
	lv_obj_center(override_label);

	lv_obj_t *calibrate_btn = lv_btn_create(rd_view_obj(view));	  // Creates Button
	lv_obj_align(calibrate_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -20); // Moves Button

	lv_obj_t *calibrate_label = lv_label_create(calibrate_btn); // Creates Label
	lv_label_set_text(calibrate_label, "Calibrate Inertial");	// Sets the text
	lv_obj_center(calibrate_label);								// Centers it on the parent button
	lv_obj_add_event_cb(calibrate_btn, [](lv_event_t *e) {		// Adds event listener
		if (lv_event_get_code(e) == LV_EVENT_CLICKED)			// If the button is clicked
		{
			rd_view_alert(view, "Calibrating IMU..."); // Alert the user.
			inertial_sensor.reset();							   // Calibrate IMU.
		}
	},
						LV_EVENT_ALL, NULL);

	lv_obj_t *reset_pose_btn = lv_btn_create(rd_view_obj(view));
	lv_obj_align(reset_pose_btn, LV_ALIGN_TOP_RIGHT, -20, 60);

	lv_obj_t *reset_pose_label = lv_label_create(reset_pose_btn);
	lv_label_set_text(reset_pose_label, "Reset Pose");
	lv_obj_center(reset_pose_label);
	lv_obj_add_event_cb(reset_pose_btn, [](lv_event_t *e)
						{
		if (lv_event_get_code(e) == LV_EVENT_CLICKED)
		{
			rd_view_alert(view, "Calibrating and Reseting...");
			chassis.calibrate();
			chassis.resetLocalPosition(); 
		} }, LV_EVENT_ALL, NULL);

	lv_obj_t *imu_heading_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(imu_heading_label, LV_ALIGN_TOP_LEFT, 5, 5);

	lv_obj_t *drive_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(drive_temp_label, LV_ALIGN_TOP_LEFT, 5, 25);

		lv_obj_t *intake_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(intake_temp_label, LV_ALIGN_TOP_LEFT, 5, 40);

		lv_obj_t *lift_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(lift_temp_label, LV_ALIGN_TOP_LEFT, 5, 55);

	lv_obj_t *lem_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(lem_label, LV_ALIGN_TOP_LEFT, 5, 65);

	lv_obj_t *horizontal_one_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(horizontal_one_label, LV_ALIGN_TOP_LEFT, 5, 175);

	lv_obj_t *lift_encoder_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(lift_encoder_label, LV_ALIGN_TOP_LEFT, 5, 200);
	// Creates the UI elements

	// Update Loop
	while (true)
	{
		// Updates the labels to show the most recent data.
		lv_label_set_text(imu_heading_label, ("IMU Heading: " + std::to_string(round(inertial_sensor.get_heading()))).c_str());
		lv_label_set_text(drive_temp_label, ("Average drive temp: " + std::to_string((dt_left.get_temperature() + dt_right.get_temperature()) / 2)).c_str());
		
		lv_label_set_text(intake_temp_label, ("Intake temp: " + std::to_string(intake_motor.get_temperature(0))).c_str());
		lv_label_set_text(lift_temp_label, ("Average lift temp: " + std::to_string(arm_motor.get_temperature(0))).c_str());
		//lv_label_set_text(horizontal_one_label, ("Horizontal tracking 1: " + std::to_string(horizontal_tracking_wheel.getDistanceTraveled())).c_str());
		//lv_label_set_text(lift_encoder_label, ("Lift encoder ANGLE: " + std::to_string((lift_encoder.get_position() / 100) * 0.2)).c_str());
		master.print(0, 0, "Connection:%d ", master.is_connected());
		pros::delay(100); // Waits 100ms before updating again to make it readable.
	}
	// Update Loop
}

*/

/*
void graph_pid() {
    int setpoint = target_heading;
    int correction = IMU_CORRECTION;
    vex::inertial sensor = imu;

    B_SCRN.clearScreen();

    int p_val = setpoint - sensor.rotation() * correction, i_val = 0, d_val;
    float scalar =  B_SCRN_Y_MID / setpoint;

    for (int i = 0; true; i++) {
        if (i > B_SCRN_X)
            i = 0;

        // Update pid vars
        d_val = p_val;
        p_val = setpoint - sensor.rotation() * correction;
        i_val += p_val;

        // Convert to graphable values
        int g_setpoint = setpoint * scalar;
        int g_sensor_val = sensor.rotation() * correction * scalar;
        int g_p_val = p_val * scalar;
        int g_i_val = i_val * scalar;
        int g_d_val = d_val * scalar;

        // Clear current column
        clear_column(i);

        // Graph lines
        draw_colored_pixel(i, g_setpoint, vex::white);
        draw_colored_pixel(i, g_sensor_val, vex::red);
        draw_colored_pixel(i, g_p_val, vex::blue);
        draw_colored_pixel(i, g_i_val, vex::yellow);
        draw_colored_pixel(i, g_d_val, vex::green);

        vex::wait(20, vex::msec);
    }
}
*/

// #include "liblvgl/widgets/lv_label.h"
// #include "main.h"
// #include "pidView.hpp"
// #include "api.h"
// #include "liblvgl/core/lv_obj.h"
// #include "liblvgl/misc/lv_color.h"

// bool opened = false;

// void rd::PIDView::eventHandlerBigTune(lv_event_t * event){
//     lv_obj_t * obj = lv_event_get_target(event);
//     lv_event_code_t code = lv_event_get_code(event);
//     if(code == LV_EVENT_CLICKED){
//         if(obj == imgButtonPDecBig){
//             kP -= 0.1;
//         }
//         else if(obj == imgButtonPIncBig){
//             kP += 0.1;
//         }
//         else if(obj == imgButtonIDecBig){
//             kI -= 0.1;
//         }
//         else if(obj == imgButtonIIncBig){
//             kI += 0.1;
//         }
//         else if(obj == imgButtonDDecBig){
//             kD -= 0.1;
//         }
//         else if(obj == imgButtonDIncBig){
//             kD += 0.1;
//         }
//     }


// }
// void rd::PIDView::eventHandlerSmallTune(lv_event_t * event){
//     lv_obj_t * obj = lv_event_get_target(event);
//     lv_event_code_t code = lv_event_get_code(event);
//     if(code == LV_EVENT_CLICKED){
//         if(obj == imgButtonPDecSmall){
//             kP -= 0.01;
//         }
//         else if(obj == imgButtonPIncSmall){
//             kP += 0.01;
//         }
//         else if(obj == imgButtonIDecSmall){
//             kI -= 0.01;
//         }
//         else if(obj == imgButtonIIncSmall){
//             kI += 0.01;
//         }
//         else if(obj == imgButtonDDecSmall){
//             kD -= 0.01;
//         }
//         else if(obj == imgButtonDIncSmall){
//             kD += 0.01;
//         }
//     }
// }

// void rd::PIDView::eventHandlerType(lv_event_t * event){
//     FILE * fileName = fopen("/usd/Autonwinop.txt", "w");
//     lv_obj_t * obj = lv_event_get_target(event);
//     lv_event_code_t code = lv_event_get_code(event);
//     if(code == LV_EVENT_CLICKED && opened == false){
        
//         fprintf(fileName, "%f %f %f", kP, kI, kD);
//     }
//     else if(code == LV_EVENT_CLICKED && opened == true){
//         fclose(fileName);

//     }
// }

// rd::PIDView::PIDView() {
//     //creating view named pidview
//     setup New(&kPP,&kII,&kDD);
//     this->view = rd_view_create("pidview");

//     //setting variables
//     this->kP = 0;
//     this->kI = 0;
//     this->kD = 0;

//     //creating chart
//     chart = lv_chart_create(view->obj);
//     lv_obj_set_size(chart, 200, 200);
//     lv_obj_align(chart, LV_ALIGN_CENTER, 0, 0);
//     lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
//     lv_chart_set_range(chart, 0, 100,0);
//     lv_chart_set_point_count(chart, 100);
//     lv_chart_set_div_line_count(chart, 0, 0);
    
//     //creating chart series
//     seriesP = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
//     seriesD = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE),LV_CHART_AXIS_PRIMARY_Y);
//     seriesTarg = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN),LV_CHART_AXIS_PRIMARY_Y);
//     seriesCur = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_YELLOW),LV_CHART_AXIS_PRIMARY_Y);

//     //creating still labels and type button
//     labelTitle = lv_label_create(view->obj);
//     lv_label_set_text(labelTitle, "PID");
//     labelType = lv_label_create(view->obj);
//     lv_label_set_text(labelType, "PID Tuner");//add string selection later if needed


//     labelP = lv_label_create(view->obj);
//     lv_label_set_text(labelP, "P");
//     labelI = lv_label_create(view->obj);
//     lv_label_set_text(labelI, "I");
//     labelD = lv_label_create(view->obj);
//     lv_label_set_text(labelD, "D");

//     //creating type buttons
    

//     //makes bigger PID buttons
//     makeBigTune();
//     makeSmallTune();





// }
// void rd::PIDView::makeSmallTune(){
//     //creating the PID buttons
//     imgButtonPDecSmall = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonPDecSmall, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_imgbtn_set_src(imgButtonPDecSmall, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_obj_set_size(imgButtonPDecSmall, 25, 25);
//     lv_obj_align(imgButtonPDecSmall, LV_ALIGN_CENTER, -25, 0);
//     lv_obj_add_event_cb(imgButtonPDecSmall, eventHandlerSmallTune, LV_EVENT_CLICKED, NULL);

//     imgButtonPIncSmall = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonPIncSmall, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_imgbtn_set_src(imgButtonPIncSmall, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_obj_set_size(imgButtonPIncSmall, 25, 25);
//     lv_obj_align(imgButtonPIncSmall, LV_ALIGN_CENTER, 25, 0);
//     lv_obj_add_event_cb(imgButtonPIncSmall, eventHandlerSmallTune, LV_EVENT_CLICKED, NULL);

//     imgButtonIDecSmall = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonIDecSmall, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_imgbtn_set_src(imgButtonIDecSmall, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_obj_set_size(imgButtonIDecSmall, 25, 25);
//     lv_obj_align(imgButtonIDecSmall, LV_ALIGN_CENTER, -25, 50);
//     lv_obj_add_event_cb(imgButtonIDecSmall, eventHandlerSmallTune, LV_EVENT_CLICKED, NULL);

//     imgButtonIIncSmall = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonIIncSmall, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_imgbtn_set_src(imgButtonIIncSmall, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_obj_set_size(imgButtonIIncSmall, 25, 25);
//     lv_obj_align(imgButtonIIncSmall, LV_ALIGN_CENTER, 25, 50);
//     lv_obj_add_event_cb(imgButtonIIncSmall, eventHandlerSmallTune, LV_EVENT_CLICKED, NULL);
    
//     imgButtonDIncSmall = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonDIncSmall, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_imgbtn_set_src(imgButtonDIncSmall, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_obj_set_size(imgButtonDIncSmall, 25, 25);
//     lv_obj_align(imgButtonDIncSmall, LV_ALIGN_CENTER, 25, 100);
//     lv_obj_add_event_cb(imgButtonDIncSmall, eventHandlerSmallTune, LV_EVENT_CLICKED, NULL);

//     imgButtonDDecSmall = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonDDecSmall, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_imgbtn_set_src(imgButtonDDecSmall, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_obj_set_size(imgButtonDDecSmall, 25, 25);
//     lv_obj_align(imgButtonDDecSmall, LV_ALIGN_CENTER, -25, 100);
//     lv_obj_add_event_cb(imgButtonDDecSmall, eventHandlerSmallTune, LV_EVENT_CLICKED, NULL);

// }
// void rd::PIDView::makeBigTune(){
//         //creating the PID buttons
//     imgButtonPDecBig = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonPDecBig, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_imgbtn_set_src(imgButtonPDecBig, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_obj_set_size(imgButtonPDecBig, 50, 50);
//     lv_obj_align(imgButtonPDecBig, LV_ALIGN_CENTER, -50, 0);
//     lv_obj_add_event_cb(imgButtonPDecBig, eventHandlerBigTune, LV_EVENT_CLICKED, NULL);

//     imgButtonPIncBig = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonPIncBig, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_imgbtn_set_src(imgButtonPIncBig, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_obj_set_size(imgButtonPIncBig, 50, 50);
//     lv_obj_align(imgButtonPIncBig, LV_ALIGN_CENTER, 50, 0);
//     lv_obj_add_event_cb(imgButtonPIncBig, eventHandlerBigTune, LV_EVENT_CLICKED, NULL);

//     imgButtonIDecBig = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonIDecBig, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_imgbtn_set_src(imgButtonIDecBig, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_obj_set_size(imgButtonIDecBig, 50, 50);
//     lv_obj_align(imgButtonIDecBig, LV_ALIGN_CENTER, -50, 50);
//     lv_obj_add_event_cb(imgButtonIDecBig, eventHandlerBigTune, LV_EVENT_CLICKED, NULL);

//     imgButtonIIncBig = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonIIncBig, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_imgbtn_set_src(imgButtonIIncBig, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_obj_set_size(imgButtonIIncBig, 50, 50);
//     lv_obj_align(imgButtonIIncBig, LV_ALIGN_CENTER, 50, 50);
//     lv_obj_add_event_cb(imgButtonIIncBig, eventHandlerBigTune, LV_EVENT_CLICKED, NULL);

//     imgButtonDIncBig = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonDIncBig, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_imgbtn_set_src(imgButtonDIncBig, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/plusLeft.png","S:/uiLIB/plusMiddle.png","S:/uiLIB/plusRight.png");
//     lv_obj_set_size(imgButtonDIncBig, 50, 50);
//     lv_obj_align(imgButtonDIncBig, LV_ALIGN_CENTER, 50, 100);
//     lv_obj_add_event_cb(imgButtonDIncBig, eventHandlerBigTune, LV_EVENT_CLICKED, NULL);

//     imgButtonDDecBig = lv_imgbtn_create(view->obj);
//     lv_imgbtn_set_src(imgButtonDDecBig, LV_IMGBTN_STATE_RELEASED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_imgbtn_set_src(imgButtonDDecBig, LV_IMGBTN_STATE_PRESSED, "S:/uiLIB/minusLeft.png","S:/uiLIB/minusMiddle.png","S:/uiLIB/minusRight.png");
//     lv_obj_set_size(imgButtonDDecBig, 50, 50);
//     lv_obj_align(imgButtonDDecBig, LV_ALIGN_CENTER, -50, 100);
//     lv_obj_add_event_cb(imgButtonDDecBig, eventHandlerBigTune, LV_EVENT_CLICKED, NULL);
// }

// void rd::PIDView::focus(){
//     rd_view_focus(this->view);
// }

// void rd::PIDView::updateFromSD(float kP, float kI, float kD){
//     //add later
// }

// // File: src/uiLIB/views/odom.cpp
// // @Brief: Odometry view implementation
// // @Credit: Using Robodash's View, also partially using Lemlib's Pose functions
// // @Description: Image of field, robot that updates position, and stats of robot's pose on the right
// #include "views/odomView.hpp"
// #include "liblvgl/core/lv_obj_pos.h"
// #include "liblvgl/widgets/lv_img.h"
// #include "liblvgl/widgets/lv_label.h"
// #include "main.h"
// #include "pros/misc.hpp"
// #include "pros/rtos.hpp"
// #include "lemlib/asset.hpp"
// #include "assets/fieldResized.c"
// #include "assets/robot.c"
// #include "assets/disconnected.c"
// #include "assets/autonomous.c"
// #include "assets/op.c"
// #include "assets/disabled.c"
// #include "assets/enabled.c"
// #include "assets/robot.c"
// #include "liblvgl/font/lv_font.h"
// #include "lemlib/chassis/chassis.hpp"


// rd::OdomView::OdomView() {
//     lv_point_t line_points[]= {{0, 0}, {0, 0}};
    

//     //creating view named odomview
//     this->view = rd_view_create("odomview");
//     lv_style_init(&styleTitle);
//     lv_style_init(&styleLabel);
//     lv_style_init(&styleLine);
//     lv_style_init(&styleLabelVar);
//     //setting variables
//     this->x = 0;
//     this->y = 0;
//     this->theta = 0;
//     this->timer = 0;
    
//     //set style
//     lv_style_set_text_font(&styleTitle, &lv_font_montserrat_20);
//     lv_style_set_text_font(&styleLabel, &lv_font_montserrat_18);
//     lv_style_set_text_color(&styleLabelVar, lv_palette_main(LV_PALETTE_GREY));

//     //creating images
//     field = lv_img_create(view->obj);
//     lv_img_set_src(field, &fieldResized);
//     robot = lv_img_create(view->obj);
//     lv_img_set_src(robot, &robotL);
//     lv_obj_set_pos(robot,50,50);
//     lv_obj_move_foreground(robot);
//     //creating changing images

//     // MatchState = lv_img_create(view->obj);
//     // lv_img_set_src(MatchState, &disconnected);
//     // lv_obj_set_x(MatchState, 314);
//     // lv_obj_set_y(MatchState, 189);
//     // DisabledState = lv_img_create(view->obj);
//     // lv_img_set_src(DisabledState, &disconnected);
//     // lv_obj_set_x(DisabledState, 364);
//     // lv_obj_set_y(DisabledState, 189);
    
//     //creating labels
//     labelTitle = lv_label_create(view->obj);
//     lv_label_set_text(labelTitle, "Odometry");
//     lv_obj_add_style(labelTitle,&styleTitle,LV_PART_MAIN);
//     lv_obj_set_x(labelTitle, 310);
//     lv_obj_set_y(labelTitle, 16);
//     labelX = lv_label_create(view->obj);
//     lv_obj_add_style(labelX,&styleLabel,LV_PART_MAIN);
//     lv_label_set_text(labelX, "X");
//     lv_obj_set_x(labelX, 302);
//     lv_obj_set_y(labelX, 84);
//     labelY = lv_label_create(view->obj);
//     lv_obj_add_style(labelY,&styleLabel,LV_PART_MAIN);
//     lv_label_set_text(labelY, "Y");
//     lv_obj_set_x(labelY, 302);
//     lv_obj_set_y(labelY, 109);
//     labelTheta = lv_label_create(view->obj);
//     lv_obj_add_style(labelTheta,&styleLabel,LV_PART_MAIN);
//     lv_label_set_text(labelTheta, "θ");
//     lv_obj_set_x(labelTheta, 302);
//     lv_obj_set_y(labelTheta, 134);

//     //creating variable labels
//     labelXVar= lv_label_create(view->obj);
//     lv_obj_add_style(labelXVar,&styleLabelVar,LV_PART_MAIN);
//     lv_label_set_text(labelXVar, "0");
//     lv_obj_set_x(labelXVar, 322);
//     lv_obj_set_y(labelXVar, 84);
//     lv_label_set_recolor(labelXVar,true) ;
//     labelYVar = lv_label_create(view->obj);
//     lv_obj_add_style(labelYVar,&styleLabelVar,LV_PART_MAIN);
//     lv_label_set_text(labelYVar, "0");
//     lv_obj_set_x(labelYVar, 322);
//     lv_obj_set_y(labelYVar, 109);
//     lv_label_set_recolor(labelYVar,true) ;
//     labelThetaVar = lv_label_create(view->obj);
//     lv_label_set_recolor(labelThetaVar,true) ;
//     lv_obj_add_style(labelThetaVar,&styleLabelVar,LV_PART_MAIN);
//     lv_label_set_text(labelThetaVar, "0");
//     lv_obj_set_x(labelThetaVar, 322);
//     lv_obj_set_y(labelThetaVar, 134);
    

//     // labelXVar = lv_label_create(view->obj);
//     // lv_label_set_text(labelXVar, "0");
//     // labelYVar = lv_label_create(view->obj);
//     // lv_label_set_text(labelYVar, "0");
//     // labelThetaVar = lv_label_create(view->obj);
//     // lv_label_set_text(labelThetaVar, "0");

//     // //creating timer label
//     // timerVarMin = lv_label_create(view->obj);
//     // timerVarSec = lv_label_create(view->obj);
//     // lv_label_set_text(timerVarMin, "0:");
//     // lv_label_set_text(timerVarSec, "00");
    
//     //creating lines styles

//     lv_style_set_line_width(&styleLine, 30);
//     lv_style_set_line_rounded(&styleLine, true);
//     lv_style_set_line_color(&styleLine, lv_palette_main(LV_PALETTE_BLUE));
//     //creating lines
//     // line1 = lv_line_create(view->obj);
//     // lv_line_set_points(line1, line_points, 2);
//     // lv_obj_add_style(line1,&styleLine,0);
    

// }
// void rd::OdomView::update(lemlib::Chassis* chassis){
//     this->x = chassis->getPose().x;
//     this->y = chassis->getPose().y;
//     this->theta = chassis->getPose().theta;
//     timer = pros::millis();
//     timerMin = (timer/1000)/60;
//     timerSec = (timer/1000)%60;
// }

// void rd::OdomView::updateLabels(){
//     lv_label_set_text(labelXVar, (std::to_string(x)).c_str());
//     lv_label_set_text(labelYVar, (std::to_string(y)).c_str());
//     lv_label_set_text(labelThetaVar, (std::to_string(theta)).c_str());
//     lv_label_set_text(timerVarMin, (std::to_string(timerMin)).c_str());
//     lv_label_set_text(timerVarSec, (std::to_string(timerSec)).c_str());
    
// }
// void rd::OdomView::updateImage(){
//     lv_obj_set_pos(robot, (x+180)*0.69, (y*-1+180)*0.69);
//     lv_img_set_angle(robot, theta*100);

// }

// void rd::OdomView::focus(){
//     rd_view_focus(this->view);
// }
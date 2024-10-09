#include "gui.h"
#include "setup.h"

// Robodash selector initialization...
rd::Selector selector({{"Red Left", &pidtune},
				   {"Red Right", &auton_red_non_rush},
				   {"Blue Left", &auton_red_non_rush},
				   {"Skills", &skills}});

rd::Image img("logo");

// Robodash console initialization...
rd::Console console;

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
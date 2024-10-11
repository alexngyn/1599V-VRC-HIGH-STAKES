#pragma once
#include "main.h" // IWYU pragma: keep

// Robodash selector initialization...
// extern rd::Selector selector;

// Robodash console initialization...
// extern rd::Console console;

//extern rd::Image img;

//extern void imageTest();
// Creates the custom pages
//extern rd_view_t *view;
//extern rd_view_t *auto_override;

//int update_ui();

// #include "liblvgl/core/lv_obj.h"
// #include "robodash/api.h"
// #include "robodash/views/image.hpp"
// #include <string>
// #include <sys/types.h>
// #pragma once

// typedef  FILE * pc_file_t;


// namespace rd{
//     class PIDView{
//         private:
//             rd_view_t *view;

//             //PID vars
//             static float kP;
//             static float kI;
//             static float kD;
            
//             //PID type
//             static std::string type;
            
//             //chart
//             lv_obj_t * chart;

//             //chart series
//             lv_chart_series_t * seriesP;
//             lv_chart_series_t * seriesD;

//             lv_chart_series_t * seriesTarg;
//             lv_chart_series_t * seriesCur;

//             //still labels
//             lv_obj_t * labelTitle;
//             lv_obj_t * labelType;
//             lv_obj_t * labelP;
//             lv_obj_t * labelI;
//             lv_obj_t * labelD;

//             //still buttons
//             lv_obj_t * buttonType;//added to label later

//             //big PID tuning buttons
//             static lv_obj_t * imgButtonPDecBig;
//             static lv_obj_t * imgButtonPIncBig;
//             static lv_obj_t * imgButtonIDecBig;
//             static lv_obj_t * imgButtonIIncBig;
//             static lv_obj_t * imgButtonDDecBig;
//             static lv_obj_t * imgButtonDIncBig;

//             //small PID tuning buttons
//             static lv_obj_t * imgButtonPDecSmall;
//             static lv_obj_t * imgButtonPIncSmall;
//             static lv_obj_t * imgButtonIDecSmall;
//             static lv_obj_t * imgButtonIIncSmall;
//             static lv_obj_t * imgButtonDDecSmall;
//             static lv_obj_t * imgButtonDIncSmall;
            
        

//             //changing labels
//             lv_obj_t * labelPVar;
//             lv_obj_t * labelIVar;
//             lv_obj_t * labelDVar;

//         public:
//             //rd functions
//             PIDView();
//             void focus();
            
//             //event handler
//             static void eventHandlerBigTune(lv_event_t * event);
//             static void eventHandlerSmallTune(lv_event_t * event);
//             static void eventHandlerType(lv_event_t * event);
//             //updates
//             void makeBigTune();//make big PID button
//             void makeSmallTune();//make small PID button
//             void initValues(float * P, float *  I, float * D); 
//             void updateFromSD(float kP, float kI, float kD);
//             void updateLabels();
//             void updateChart(float P, float D, float target, float current);

//     };
// }

// #include "robodash/api.h"
// #include "robodash/views/image.hpp"
// #include <string>
// #include <sys/types.h>
// #include "lemlib/chassis/chassis.hpp"
// #pragma once

// namespace rd{
//     class OdomView{
//         private: 
//             //View
//             rd_view_t *view;

//             //Pose Vars
//             float x;
//             float y;
//             float theta;

//             //Timer Vars
//             u_int32_t timer;
//             u_int32_t timerMin;
//             u_int32_t timerSec;

//             //State Vars
//             bool isConnected;
//             bool isAuton;
//             bool isDisabled;
//             //Styles

//             lv_style_t styleTitle;
//             lv_style_t styleLabel;
//             lv_style_t styleLine;

//             lv_style_t styleLabelVar;
//             //Images
//             lv_obj_t * field;
//             lv_obj_t * robot;

//             //Changing Images
//             lv_obj_t * MatchState;
//             lv_obj_t * DisabledState;


//             //Still Labels
//             lv_obj_t * labelTitle;
//             lv_obj_t * labelX;
//             lv_obj_t * labelY;
//             lv_obj_t * labelTheta;


//             //Changing Labels
//             lv_obj_t * labelXVar;
//             lv_obj_t * labelYVar;
//             lv_obj_t * labelThetaVar;
//             lv_obj_t * timerVarMin;
//             lv_obj_t * timerVarSec;

//             //lines
//             lv_obj_t * line1;

            
            
//         public:
//             OdomView();
//             void focus();//focuses page to view
//             void update(lemlib::Chassis* chassis);//updates pose
//             void updateLabels();//updates labels
//             void updateImage();//updates image of robot
//             void updateState();//updates stateimages of robot
//     };
// }
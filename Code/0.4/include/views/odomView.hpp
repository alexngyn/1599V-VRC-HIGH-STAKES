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
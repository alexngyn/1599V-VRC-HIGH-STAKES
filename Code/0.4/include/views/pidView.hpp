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
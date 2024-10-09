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


#include "main.h"
#include "views/imageTest.hpp"
#include "assets/imageTest.c"
#include "assets/fieldResized.c"
rd::imageTest::imageTest(){
    this->view = rd_view_create("imageTest");
    image = lv_img_create(view->obj);
    lv_img_set_src(image, &fieldResized);
}

void rd::imageTest::focus(){
    rd_view_focus(this->view);
}
#include "robodash/api.h"
#include "robodash/views/image.hpp"
#include <string>
#include <sys/types.h>
#pragma once
namespace rd{
    class imageTest{
        private:
            rd_view_t *view;
            lv_obj_t * image;
        public:
            imageTest();
            void focus();//focuses page to view
    };  
};
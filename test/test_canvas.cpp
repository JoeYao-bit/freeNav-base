//
// Created by yaozhuo on 2021/9/20.
//

#include "../visualization/canvas/canvas.h"
#include "../basic_elements/point.h"
#include "gtest/gtest.h"

using namespace freeNav;

Pointi<2> catched_point1;
Pointi<2> catched_point2;

TEST(CANVASTEST, test) {
    Canvas canvas("test canvas", 1000, 800, 50);

    auto mouse_call_back = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "** start (" << x << ", " << y <<  ") has been set -*-" << std::endl;
            catched_point1[0] = x;
            catched_point1[1] = y;
        }
        else if (event == cv::EVENT_LBUTTONUP) {
            std::cout << "** target (" << x << ", " << y <<  ") has been set, ready for path planning -*-" << std::endl;
            catched_point2[0] = x;
            catched_point2[1] = y;
        }
    };

    canvas.setMouseCallBack(mouse_call_back);

    while (1) {
        canvas.resetCanvas();
        canvas.show();
    }


}
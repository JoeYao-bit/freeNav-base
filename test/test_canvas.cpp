//
// Created by yaozhuo on 2021/9/20.
//
#include "gtest/gtest.h"

#include "../dependencies/2d_grid/text_map_loader.h"
#include "../visualization/canvas/canvas.h"
#include "../basic_elements/point.h"
#include "../test/test_data.h"

using namespace freeNav;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

auto map_test_config = MapTestConfig_Milan_1_512;
TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto dimension = loader.getDimensionInfo();
auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };
auto set_occupied = [](const Pointi<2> & pt) { loader.setOccupied(pt); };

IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

Pointi<2> pt1;
Pointi<2> pt2;
bool center_offset = false;

TEST(CANVASTEST, test) {
    Canvas canvas("test canvas", dimension[0], dimension[1], 5);

    auto mouse_call_back = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "** start (" << x << ", " << y <<  ") has been set -*-" << std::endl;
            pt1[0] = x;
            pt1[1] = y;
        }
        else if (event == cv::EVENT_RBUTTONUP) {
            std::cout << "** target (" << x << ", " << y <<  ") has been set, ready for path planning -*-" << std::endl;
            pt2[0] = x;
            pt2[1] = y;
        }
    };

    canvas.setMouseCallBack(mouse_call_back);

    while (1) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dimension, is_occupied_func);
        canvas.drawCircleInt(pt1[0], pt1[1], 5, center_offset, -1, COLOR_TABLE[0]);
        canvas.drawCircleInt(pt2[0], pt2[1], 5, center_offset, -1, COLOR_TABLE[1]);
        canvas.drawArrowInt(pt1[0], pt1[1], pt2[0], pt2[1], 2, center_offset, COLOR_TABLE[1]);
        canvas.show();
    }
}
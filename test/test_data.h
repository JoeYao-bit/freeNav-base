//
// Created by yaozhuo on 2022/9/6.
//

#ifndef FREENAV_BASE_TEST_DATA_H
#define FREENAV_BASE_TEST_DATA_H
#include <iostream>
#include <map>
#include "../basic_elements/point.h"
#include "../dependencies/massive_test_interfaces.h"

namespace freeNav {



    SingleMapTestConfig<2> MapTestConfig_Milan_1_512 =
            {
                    {"map_name",    "Milan_1_512"},
                    {"map_path",    "../test/test_data/Milan_1_512.map"},
                    {"vis_path",    "../test/test_data/Milan_1_512_ENLSVG.vis"},
                    {"config_path", "../test/test_data/Milan_1_512.map.scen"},
                    {"output_path", "../test/test_data/Milan_1_512.txt"}
            };


    SingleMapTestConfig<3> MapTestConfig_Complex =

            {
                    {"map_name",    "Complex"},
                    {"map_path",    "../test/test_data/Complex.3dmap"},
                    {"vis_path",    "../test/test_data/Complex.vis"},
                    {"block_path",  "../test/test_data/Complex.block"},
                    {"block_path_oc",  "../test/test_data/Complex_oc.block"},
                    {"minimum_block_width", "10"}, // optimal 26
                    {"los_output_path", "../test/test_data/Complex-los.txt"},
                    {"config_path", "../test/test_data/Complex.3dmap.3dscen"},
                    {"output_path", "../test/test_data/Complex.txt"},
                    {"shrink_level", "1"}
            };

    SingleMapTestConfig<2> MAPFTestConfig_Berlin_1_256 =

            {
                    {"map_name",     "Berlin_1_256"},
                    {"map_path",     "../test/test_data/Berlin_1_256.map"},
                    {"scene_path",   "../test/test_data/Berlin_1_256-random-1.scen"},
                    {"ct_path",   "../test/test_data/Berlin_1_256.ct"},
                    {"output_path", "../test/test_data/Berlin_1_256-random-1.txt"},
                    {"decomposition_output_path", "../test/test_data/Berlin_1_256-random-1_de.txt"},
                    {"agent_num",    "400"}, // 600
                    {"cut_off_time", "200"},
                    {"max_run_time", "60"} // in second
            };

}
#endif //FREENAV_TEST_DATA_H

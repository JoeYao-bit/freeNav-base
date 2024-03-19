//
// Created by yaozhuo on 2022/7/5.
//

#include "path_planning_interface.h"
#include <fstream>
namespace freeNav {

    void appendToFile(const std::string &file_name, const std::string &content, bool prune) {
        // 3, save result to file
        std::ofstream os(file_name, prune ? std::ios_base::out : std::ios_base::app);
        //os << "TYPE START TARGET TIME_COST MAKE_SPAN TOTAL_LENGTH " << std::endl;
        os << content << std::endl;
        os.close();
    }
}
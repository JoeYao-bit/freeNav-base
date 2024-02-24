//
// Created by yaozhuo on 2023/1/19.
//

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
#include "../dependencies/directory_loader.h"

void getFiles(const char* path, std::vector<std::string>& files){

    const std::string path0 = path;
    DIR* pDir;
    struct dirent* ptr;

    struct stat s;
    lstat(path, &s);

    if(!S_ISDIR(s.st_mode)) {
        std::cout << "not a valid directory: " << path << std::endl;
        return;
    }

    if(!(pDir = opendir(path))){
        std::cout << "opendir error: " << path << std::endl;
        return;
    }
    int i = 0;
    std::string subFile;
    while((ptr = readdir(pDir)) != 0){
        subFile = ptr -> d_name;
        if(subFile == "." || subFile == "..")
            continue;
        subFile = path0 + subFile;
        std::cout << ++i << ": " << subFile << std::endl;
        files.push_back(subFile);
    }
    closedir(pDir);

}

int main() {

    std::string folder = "/home/yaozhuo/code/Grid-based Path Planning Competition Dataset/all-map";
    std::string format_name = std::string("map");
    auto files = getFilesWithFormat(folder.c_str(), format_name);
    int i;
    for(const auto file : files) {
        std::cout << ++i << ": " << file << std::endl;
        auto config = convertFromMapToTestConfig(file);
        for(const auto& config_pair : config) {
            std::cout << config_pair.first << " : " << config_pair.second << std::endl;
        }
        break;
    }
}
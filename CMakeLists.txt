cmake_minimum_required(VERSION 2.8)

project(freeNav-base)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
enable_testing()
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()


find_package(Boost REQUIRED COMPONENTS program_options system filesystem thread graph)

include(GoogleTest)

include_directories(
                    ${CMAKE_SOURCE_DIR}
                    ${Boost_INCLUDE_DIRS}

)

aux_source_directory(basic_elements basic_src)
aux_source_directory(dependencies depen_src)
aux_source_directory(dependencies/2d_grid     2d_grid_src)
aux_source_directory(dependencies/3d_textmap  3d_textmap_src)
aux_source_directory(dependencies/massive_scene_loader  mass_src)


#add_library(harrt_src_lib ${harrt_src})
#target_link_libraries(harrt_src_lib ${OpenCV_LIBS} ${CGAL_LIBRARIES})

add_library(freeNav-base ${3d_vis_src} ${canvas_src} ${qt_src}
                    ${basic_src}
        ${depen_src} ${2d_grid_src} ${3d_octomap_src} ${3d_textmap_src} ${mass_src}
         )

set(EXTERNAL_LIBS
        gtest
        glog
        gtest_main
        pthread
        ${Boost_LIBRARIES}
        )

target_link_libraries(freeNav-base ${EXTERNAL_LIBS})






cmake_minimum_required(VERSION 3.5)

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

#set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")

find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options system filesystem thread graph)

message ( STATUS "Python3_FOUND = ${Python3_FOUND} " )
message ( STATUS "Python3_INCLUDE_DIRS = ${Python3_INCLUDE_DIRS} " )
message ( STATUS "Python3_LIBRARIES = ${Python3_LIBRARIES} " )


include_directories(
        #SYSTEM
        ${Boost_INCLUDE_DIRS}
)


include_directories(${EIGEN3_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}
        )

find_package(Boost REQUIRED COMPONENTS program_options system filesystem thread graph)

include(GoogleTest)

include_directories(
                    ${CMAKE_SOURCE_DIR}
                    ${Boost_INCLUDE_DIRS}
)

INSTALL(DIRECTORY basic_elements DESTINATION include/freeNav-base)
INSTALL(DIRECTORY dependencies DESTINATION include/freeNav-base)


set(G2O_LIBS
        g2o_cli
        g2o_ext_freeglut_minimal
        g2o_simulator
        g2o_solver_slam2d_linear
        g2o_types_icp
        g2o_types_slam2d
        g2o_core g2o_interface
        g2o_solver_csparse
        g2o_solver_structure_only
        g2o_types_sba
        g2o_types_slam3d
        g2o_csparse_extension
        g2o_opengl_helper
        g2o_solver_dense
        g2o_stuff
        g2o_types_sclam2d
        g2o_parser
        g2o_solver_pcg
        g2o_types_data
        g2o_types_sim3
        cxsparse)


aux_source_directory(basic_elements basic_src)
aux_source_directory(dependencies depen_src)
aux_source_directory(dependencies/2d_grid     2d_grid_src)
aux_source_directory(dependencies/3d_textmap  3d_textmap_src)
aux_source_directory(dependencies/massive_scene_loader  mass_src)

INSTALL(DIRECTORY basic_elements DESTINATION include/freeNav-base)
INSTALL(DIRECTORY dependencies DESTINATION include/freeNav-base)


add_library(freeNav-base
                    ${basic_src}
        ${depen_src} ${2d_grid_src} ${3d_textmap_src} ${mass_src}
        basic_elements/pose_se2.cpp
         )



set(EXTERNAL_LIBS
        ${EIGEN3_LIBS}
        ${G2O_LIBS}
        ${Boost_LIBRARIES}
        )

target_link_libraries(freeNav-base ${EXTERNAL_LIBS})


INSTALL (TARGETS freeNav-base freeNav-base LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)



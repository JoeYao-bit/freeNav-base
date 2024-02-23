//
// Created by yaozhuo on 2022/2/26.
//
#include <sys/time.h>

#include "visualization/3d_viewer/3d_viewer.h"

#include "test_data.h"
#include "basic_elements/point.h"

#include "octomap/octomap.h"
#include "dependencies/random_map_generator.h"
#include "dependencies/thread_pool.h"
#include "dependencies/3d_textmap/voxel_loader.h"

Viewer3D* viewer_3d;
ThreadPool viewer_thread(1);

using namespace freeNav;

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;


#define OCTO_MAP 1
#if OCTO_MAP
OctoMapLoader* oml_ = nullptr;
#else
TextMapLoader_3D* oml_ = nullptr;
#endif


//bool (*f1)(const freeNav::RimJump::Pointi<3>&) = is_occupied;
//void (*f2)(const freeNav::RimJump::Pointi<3>&) = set_occupied;


bool plan_finish = false;

Pointi<3> pt1, pt2;

std::vector<Path<3> > paths;
Path<3> path_rrt, path_jps, path_astar;


// MapTestConfig_Full4 // run out of space, shrink space 4
// MapTestConfig_Simple // success
// MapTestConfig_DA2
// MapTestConfig_DC1 // one bywave, need 8~14s
// MapTestConfig_Complex
// MapTestConfig_A1
// MapTestConfig_FA2
auto config = MapTestConfig_Complex;

std::string file_path = "/home/yaozhuo/code/free-nav/resource/binary/fr_campus.vis";

int main() {
#if OCTO_MAP
    oml_ = new OctoMapLoader("/home/yaozhuo/code/free-nav/resource/map/fr_campus.bt", 6 );
#else
    std::string map_path = config.at("map_path");
    oml_ = new TextMapLoader_3D(map_path, atoi(config.at("shrink_level").c_str())); // shrink to enable load
    file_path = config.at("vis_path");
#endif
    // load octomap
    auto is_occupied = [](const Pointi<3> & pt) -> bool { return oml_->isOccupied(pt); };
    auto set_occupied = [](const Pointi<3> & pt) { oml_->setOccupied(pt); };

    IS_OCCUPIED_FUNC<3> is_occupied_func = is_occupied;

    SET_OCCUPIED_FUNC<3> set_occupied_func = set_occupied;

    gettimeofday(&tv_pre, &tz);

    ThreadPool tp(1);

    // set viewer
    viewer_3d = new Viewer3D();
    viewer_3d->start_[0] = 94;
    viewer_3d->start_[1] = 89;
    viewer_3d->start_[2] = 126;
    viewer_3d->target_[0] = 160;
    viewer_3d->target_[1] = 59;
    viewer_3d->target_[2] = 94;

    //std::cout << "MAX<uint_least32_t> " << MAX<uint_least32_t> << std::endl;

    viewer_thread.Schedule([&]{
        viewer_3d->init();

        pangolin::Var<bool> menu_octomap = pangolin::Var<bool>("menu.OctoMap",false, true);
        pangolin::Var<bool> menu_occupied_surface = pangolin::Var<bool>("menu.DrawOctomap",true, true);
        pangolin::Var<bool> menu_tangent_graph("menu.TangentGraph",false, true);
        pangolin::Var<bool> menu_path = pangolin::Var<bool>("menu.DrawPath",true, true);
        //pangolin::Var<bool> menu_grid("menu.GridOn",false, true);
        pangolin::Var<bool> menu_vlo = pangolin::Var<bool>("menu.DrawVoxelLine",false, true);
        pangolin::Var<bool> menu_gv = pangolin::Var<bool>("menu.DrawGreyVoxel",false, true);
        pangolin::Var<bool> menu_state = pangolin::Var<bool>("menu.DrawStart&Target",true, true);
        pangolin::Var<bool> menu_bound = pangolin::Var<bool>("menu.DrawBoundary",true, true);
        pangolin::Var<bool> menu_tangent_candidate_grid = pangolin::Var<bool>("menu.DrawTangentCandidate",false, true);
        pangolin::Var<bool> menu_set_state = pangolin::Var<bool>("menu.SetStartNow",true, true);
        pangolin::Var<bool> menu_start_plan = pangolin::Var<bool>("menu.StartPlan",false, false);
        pangolin::Var<bool> menu_rimjump = pangolin::Var<bool>("menu.RimJump*",true, true);
        pangolin::Var<bool> menu_dijk = pangolin::Var<bool>("menu.Dijkstra",true, true);
        pangolin::Var<bool> menu_astar = pangolin::Var<bool>("menu.Astar",true, true);
        pangolin::Var<bool> menu_theta = pangolin::Var<bool>("menu.Theta*",true, true);
        pangolin::Var<bool> menu_rrt = pangolin::Var<bool>("menu.RRT",true, true);
        pangolin::Var<bool> menu_jps = pangolin::Var<bool>("menu.JPS",true, true);
        pangolin::Var<bool> menu_visible_start = pangolin::Var<bool>("menu.visible_of_start",false, true);
        pangolin::Var<bool> menu_visible_target = pangolin::Var<bool>("menu.visible_of_target",false, true);
        pangolin::Var<bool> menu_reset_view = pangolin::Var<bool>("menu.ResetView",false, false);


        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // Define Camera Render Object (for view / scene browsing)
        //viewer_3d->mViewpointX = oml_->getDimensionInfo()[0]/2; // doesn't matter
        //viewer_3d->mViewpointY = oml_->getDimensionInfo()[1]/2; // doesn't matter
        double max_depth = 1.5*std::max(oml_->getDimensionInfo()[0], oml_->getDimensionInfo()[1]) + oml_->getDimensionInfo()[2];
        auto modeview_matrix = pangolin::ModelViewLookAt(oml_->getDimensionInfo()[0]/2,
                                                         oml_->getDimensionInfo()[1]/2,
                                                         max_depth,//2*planner_3d->space_3d->max_z,
                                                         oml_->getDimensionInfo()[0]/2,
                                                         oml_->getDimensionInfo()[1]/2,
                                                         0, 0.0,-1.0, 0.0);
        // set parameters for the window
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,viewer_3d->mViewpointF,viewer_3d->mViewpointF,512,389,0.1,max_depth + 1000),
                modeview_matrix
        );
        MyHandler3D* handle_3d = new MyHandler3D(s_cam);
        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(handle_3d);
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        while( !pangolin::ShouldQuit() )
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if(menu_reset_view) {
                s_cam.SetModelViewMatrix(modeview_matrix);
                menu_reset_view = false;
            }
            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            if(menu_vlo) viewer_3d->show_line_for_voxel = true;
            else viewer_3d->show_line_for_voxel = false;
            if(menu_gv) viewer_3d->show_grey_voxel = true;
            else viewer_3d->show_grey_voxel = false;
            //if(menu_grid) DrawGrid();

            if(menu_octomap && oml_ != nullptr) {
#if OCTO_MAP
                viewer_3d->DrawOctoMapGrid(*oml_);
#else
                viewer_3d->DrawVoxels(surface_processor->getOccVoxels(), oml_->getDimensionInfo());
#endif
            }

            // if(menu_surface) DrawSurface(planner_3d->space_3d);
            if(menu_state) {
                viewer_3d->DrawPoint(viewer_3d->start_[0], viewer_3d->start_[1], viewer_3d->start_[2]);
                viewer_3d->DrawPoint(viewer_3d->target_[0], viewer_3d->target_[1], viewer_3d->target_[2]);
            }

            if(menu_set_state) viewer_3d->viewer_set_start = true;
            else viewer_3d->viewer_set_start = false;

            if(menu_bound) {
                viewer_3d->DrawBound(0, oml_->getDimensionInfo()[0], 0, oml_->getDimensionInfo()[1], 0, oml_->getDimensionInfo()[2]);
            }
            if(menu_rimjump) {
                //viewer_3d->drawRoadMapEdges(pps);
                for(const auto& path : paths) {
                    viewer_3d->DrawPath(path);
                }
            }
            if(menu_rrt) {
                viewer_3d->DrawPath(path_rrt);
            }
            if(menu_jps) {
                viewer_3d->DrawPath(path_jps);
            }
            if(menu_astar) {
                viewer_3d->DrawPath(path_astar);
            }
            pangolin::FinishFrame();
        }

    });
    return 0;
}

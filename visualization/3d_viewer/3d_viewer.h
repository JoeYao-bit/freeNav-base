//
// Created by yaozhuo on 2022/2/26.
//

#ifndef FREENAV_3D_VIEWER_H
#define FREENAV_3D_VIEWER_H

#include <mutex>
#include <thread>
#include <deque>
#include <functional>
#include <condition_variable>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/LU"
#include "eigen3/Eigen/Core"
#include <pangolin/pangolin.h>

#include "rim_jump/basic_elements/point.h"
#include "rim_jump/online_search/path_tree_with_edge.h"
#include "3d_octomap/octomap_loader.h"

#include "dependencies/thread_pool.h"
#include "3d_octomap/octomap_loader.h"
#include "3d_textmap/voxel_loader.h"
#include "EECBS/inc/common.h"

struct MyHandler3D : public pangolin::Handler3D
{
    MyHandler3D(pangolin::OpenGlRenderState& render_) : pangolin::Handler3D(render_) {}
    void Keyboard(pangolin::View& view_, unsigned char key, int x, int y, bool pressed);
};

namespace fr = freeNav;
struct Viewer3D
{

    Viewer3D();
    ~Viewer3D();

    void init();

    void DrawOctoMap(OctoMapLoader& octomap);
    void DrawOctoMapGrid(OctoMapLoader& octomap);

    void DrawSurface();
    void DrawVoxel(double x, double y, double z, float halfsize, double min_z, double max_z);
    void DrawVoxel(double x, double y, double z, float halfsize, double r, double g, double b, bool filled = true);

    void heightMapColor(double h, double& r, double &g, double& b);
    void DrawVoxels(const fr::Pointis<3>& voxels, fr::DimensionLength* dim);
    void DrawGrid();
    void DrawLine(const fr::Pointi<3>& p1, const fr::Pointi<3>& p2, float r=1, float g=0, float b=0);
    void DrawLine(int x1, int y1, int z1, int x2, int y2, int z2, float r=1, float g=0, float b=0);
    void DrawTangentGraph(const fr::RimJump::RoadMapGraphPtr<3>& tg);

    void DrawTangentNode(const fr::RimJump::RoadMapGraphPtr<3>& tg, const fr::NodeId& node_id, const cv::Vec3b& color);
    void DrawVoxel(const fr::Pointi<3>& pt, const cv::Vec3b& color);
    void DrawVoxels(const fr::Pointis<3>& pts, const cv::Vec3b& color);
    void DrawBlock(const fr::Pointi<3>& p1, const fr::Pointi<3>& p2, float r=1, float g=0, float b=0);


    void DrawLine(double x1, double y1, double z1, double x2, double y2, double z2, double line_width = 2);
    void DrawPath(const fr::Path<3> &path, float r=1, float g=0, float b=0);
    void DrawBound(double min_x, double max_x,double min_y, double max_y,double min_z, double max_z);
    void DrawPoint(double px, double py, double pz);
    void drawRoadMapEdgesAsPath(fr::RimJump::RoadMapGraphPtr<3>& tg, fr::RimJump::RoadMapEdgeTraitPtrs<3>& edges);
    void drawRoadMapEdges(fr::RimJump::RoadMapGraphPtr<3>& tg, fr::RimJump::RoadMapEdgeTraitPtrss<3>& edgess);
    void drawRoadMapEdgeAsPath(fr::RimJump::RoadMapGraphPtr<3>& tg, fr::RimJump::RoadMapEdgeTraitPtr<3> edge, const cv::Vec3b& color);
    void drawEdge(fr::RimJump::RoadMapGraphPtr<3>& tg, const fr::RimJump::RoadMapEdgeTraitPtr<3>& edge, bool only_loop, bool draw_branch, const cv::Vec3b& color1);

    // draw an arrow in horizontal level
    void DrawArrow(double x1, double y1, double x2, double y2, double z, double head_width = 0.3, double line_width = 2);

    // 2D MAPF scene visualization
    void DrawGridMapAtTimeIndex(fr::DimensionLength* dimension_info, const fr::IS_OCCUPIED_FUNC<2>& is_occupied, int time_index);
    void DrawMAPFPathAtTimeIndex(int time_index, bool draw_specific, int path_id, bool fix_height = true);
    void DrawOneMAPFPathAtTimeIndex(int time_index, int path_id, bool fix_height = true);

    void SetMapfPath(const fr::Paths<2>& multiple_paths) {
        mapf_paths_ = multiple_paths;
        max_time_index_ = 0;
        for(const auto& path : mapf_paths_) {
            max_time_index_ = std::max((int)path.size(), max_time_index_);
        }
    }

    // key board interaction event listen function, for setting start and target coordinate
    static void KeyBoardCallBack_W(); // y + 1
    static void KeyBoardCallBack_A(); // y - 1
    static void KeyBoardCallBack_S(); // x - 1
    static void KeyBoardCallBack_D(); // x + 1
    static void KeyBoardCallBack_Q(); // z + 1
    static void KeyBoardCallBack_E(); // z - 1

    static bool StateLegalityCheck(); // check start or target is legal or not

    void Run();

    double mImageWidth, mImageHeight;

    double mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    static bool viewer_set_start;
    bool show_line_for_voxel = false;
    bool show_grey_voxel = false;
    bool under_planning;

    static freeNav::Pointi<3> start_;

    static freeNav::Pointi<3> target_;

    static int current_top_level; // for MAPF visualization

    static fr::Paths<2> mapf_paths_; // for MAPF visualization

    static int max_time_index_; // max time index in MAPF path visualization

    static int current_path_id; // for MAPF visualization

};

#endif //FREENAV_3D_VIEWER_H

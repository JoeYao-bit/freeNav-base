//
// Created by yaozhuo on 2021/9/20.
//

#ifndef FREENAV_CANVAS_H
#define FREENAV_CANVAS_H

#include "dependencies/color_table.h"
#include <functional>
#include "path_smooth/pose_se2.h"
#include "path_smooth/robot_footprint_model.h"
#include "2d_grid/picture_loader.h"
#include "rim_jump/online_search/search_path_with_edge.h"
#include "rim_jump/surface_processor/surface_processor_ENLSVG_LineScanner.h"
#include "rim_jump/surface_processor/surface_processor_LineScanner.h"
#include "rim_jump/online_search/dynamic_data_with_node.h"
#include "rim_jump/online_search/dynamic_data_with_edge.h"
#include "rim_jump/los_check_for_sparse/map_down_sampler.h"
#include "rim_jump/los_check_for_sparse/block_detect.h"
#include "rim_jump/basic_elements/voronoi_graph.h"
#include "my_cbs/single_agent_solver.h"
#include "ENLSVG.h"

namespace freeNav {

    class Canvas {
    public:

        // m ratio must be integer that greater than 1
        explicit Canvas(std::string name, int size_x, int size_y, double resolution, int zoom_ratio = 1);

        void setColorTable();

        /* e default (0, 0) is the center of the map */
        //explicit Canvas(std::string name, int resolution);

        void drawLineInt(int x1, int y1, int x2, int y2, bool center_offset, int line_width = 1, const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawLineInt(const Fraction& x1, const Fraction& y1, const Fraction& x2, const Fraction& y2, bool center_offset, int line_width = 1, const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawLine(double x1, double y1, double x2, double y2, int line_width = 1,
                      const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawPointInt(int x, int y, const cv::Vec3b &color = cv::Vec3b(0, 0, 0));

        void drawPoint(double x1, double y1, const cv::Vec3b &color = cv::Vec3b(0, 0, 0));

        void drawCircleInt(int x, int y, int radius, bool center_offset = true, int line_width = 1, const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawCircleInt(const Fraction& x, const Fraction& y, int radius, bool center_offset = true, int line_width = 1, const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawCircle(double x1, double y1, double radius, int line_width = 1,
                        const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawArrow(double x, double y, double theta, double arrow_length, int line_width = 1,
                       const cv::Scalar &color = cv::Scalar::all(0));

        void drawArrowInt(int x, int y, double theta, double arrow_length, int line_width = 1,
                          const cv::Scalar &color = cv::Scalar::all(0));

        void drawArrowInt(int x1, int y1, int x2, int y2, int line_width, bool center_offset,
                          const cv::Scalar &color = cv::Scalar::all(0));

        void drawPathf(const Pointds<2> &pathd, int line_width = 1, const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawPathi(const std::vector<Pathfinding::GridVertex> &path, const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawPointfs(const Pointds<2> &pathd, double radius, int line_width = 1, const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawPointfs(const std::vector<PoseSE2> &path,  const std::vector<double>& time_diffs, double current_time, const Point2dContainer &polygon, int line_width = 1);

        void drawPointfs(const std::vector<PoseSE2> &path, double radius, int line_width = 1,
                         const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawAxis(double x_range = 5., double y_range = 5., double wing_length = .05);

        void drawPolygon(double x, double y, double theta, const Point2dContainer &polygon, int line_width = 1,
                         const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawGrid(int x, int y, const cv::Vec3b &color = cv::Vec3b(0, 0, 0));

        void drawGridLine(int x1, int y1, int x2, int y2, int line_width = 1, bool center_offset = true,
                          const cv::Scalar &color = cv::Scalar(0, 0, 0));

        void drawGridMap(freeNav::DimensionLength *dimension,
                         IS_OCCUPIED_FUNC<2> is_occupied);

        void drawGridMap(const freeNav::RimJump::MapDownSampler<2>& down_sampler, int down_sample_level);

        void drawENLVisibilityGraph(const Pathfinding::ENLSVG::VisibilityGraph &graph);

        void drawENLVisibilityGraph(const Pathfinding::ENLSVG::VisibilityGraph &graph, const Pathfinding::ENLSVG::Memory& memory);

        void drawGrids(const freeNav::Pointis<2> &pts, const cv::Vec3b &color = cv::Vec3b(0, 0, 0));

        void drawGrids(std::map<Id, freeNav::RimJump::GridPtr<2>> pts, const cv::Vec3b &color = cv::Vec3b(0, 0, 0));

        void drawPointiCircles(const freeNav::Pointis<2> &pts, const cv::Vec3b &color, int radius, int line_width);

        void drawSurfaceGrids(const std::vector<freeNav::RimJump::GridPtr<2>> &sfg,
                              const cv::Vec3b &color = cv::Vec3b(0, 0, 0), bool with_nearby = true);

        template<typename T>
        void drawEdges(const T &edges, bool center_offset, bool only_loop = false, bool draw_branch = false) {
            if (edges.empty()) return;
            int color_count = 0;
            for (const auto &edge : edges) {
                drawEdge(edge, center_offset, only_loop, draw_branch, COLOR_TABLE[color_count]);
                //drawEdge(edge, only_loop, draw_branch, cv::Scalar());
                color_count++;
            }
        }

        void
        drawEdge(freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                 const freeNav::RimJump::RoadMapEdgeTraitPtr<2> &edge,
                 bool center_offset,
                 bool only_loop = false, bool draw_branch = false,
                 const cv::Scalar &color1 = cv::Scalar(0, 0, 0));

        void drawEdges(freeNav::RimJump::RoadMapGraphPtr<2> &tg, const freeNav::RimJump::RoadMapEdgeTraitPtrs<2> &edges, bool center_offset, const cv::Scalar &color1 = cv::Scalar(0, 0, 0));

        void drawNodes(freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                       RimJump::DynamicDataOfSearchWithNodePtr<2> data,
                       bool center_offset, const cv::Scalar &color1 = cv::Scalar(0, 0, 0));

        void drawNode(freeNav::RimJump::RoadMapGraphPtr<2> &tg, freeNav::NodeId & id, bool center_offset, const cv::Scalar &color1 = cv::Scalar(0, 0, 0));

        void drawNodes(freeNav::RimJump::RoadMapGraphPtr<2> &tg, bool center_offset, const cv::Scalar &color1);

        void drawEdgess(freeNav::RimJump::RoadMapGraphPtr<2> &tg, const freeNav::RimJump::RoadMapEdgeTraitPtrss<2> &edgess, bool center_offset);

        void drawTangentGraphAllNodes(freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                                      const cv::Vec3b &color1 = cv::Vec3b(
                                              0, 0, 0), const cv::Vec3b &color2 = cv::Vec3b(0, 0, 0));

        void drawTangentGraphEdge(freeNav::DimensionLength dimen[2],
                                  freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                                  const freeNav::Pointi<2> &pt1,
                                  const freeNav::Pointi<2> &pt2,
                                  bool center_offset);

        void drawTangentGraphLegalEdges(freeNav::RimJump::RoadMapGraphPtr<2> &tg, bool center_offset,
                                        const cv::Vec3b &color1 = cv::Vec3b(0, 0, 0),
                                        const cv::Vec3b &color2 = cv::Vec3b(0, 0, 0));

        void drawMarkedInitialEdges(freeNav::RimJump::RoadMapGraphPtr<2> &tg, bool center_offset);

        void drawEmptyGrid();

        void drawTangentPoints(const freeNav::Pointi<2> &start, const freeNav::RimJump::RoadMapNodePtrs<2> &tpts);

        void drawTangentPathPoints(freeNav::RimJump::PathPointWithEdgePtrs<2> pps, const freeNav::Pointi<2> &start,
                                   const freeNav::Pointi<2> &target);

        void drawTangentPathPoint(freeNav::RimJump::PathPointWithEdgePtr<2> pps, const freeNav::Pointi<2> &start,
                                  const freeNav::Pointi<2> &target, const cv::Scalar &color);

        void drawRoadMapEdgeAsPath(freeNav::RimJump::GeneralGraphPathPlannerWithEdge<2>& g2p2, freeNav::RimJump::RoadMapEdgeTraitPtr<2> edge,
                                   bool center_offset, bool is_edge, bool is_start,
                                   const cv::Scalar &color = cv::Scalar(1, 1, 1, 1));


        void drawTextInt(int x, int y, const char *string, const cv::Scalar &color, double scale = .7, bool center_offset = true);

        //void drawWaveTree(const freeNav::WaveTree<2> &wave_tree);

        //void drawWaveTreeNode(const freeNav::WaveTreeNodePtr<2> &wave_tree_node_ptr);

        void drawRoadMapEdges(freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                              const freeNav::RimJump::RoadMapEdgeTraitPtrs<2> &edges,
                              bool center_offset,
                              const cv::Scalar &color1);

        void drawPaths(const freeNav::Paths<2> &paths);

        void drawPath(const freeNav::Path<2> &path, bool center_offset = false,
                      const cv::Scalar &color = cv::Scalar::all(0));

        void draw_RimJump_Extent(const freeNav::RimJump::Extent &extent, freeNav::DimensionLength dimen[2], double scale = .7);

        void draw_RimJump_IntervalTree(const FractionPair& start, const freeNav::RimJump::IntervalTree &tree, freeNav::DimensionLength dimen[2],  const cv::Scalar &color = cv::Scalar(0,0,255), double scale = .7);

        void draw_RimJump_Intervals(const FractionPair& start, const RimJump::IntervalPtrs& prev_halfs, const cv::Scalar &color = cv::Scalar::all(0));

        void draw_ENLSVG_Extent(const std::vector<int> &extents, freeNav::DimensionLength dimen[2], double scale = .7);

        void draw_ENLSVG_ScannerInterval(const Pathfinding::ScanInterval &scanner_interval,
                                         const cv::Scalar &color = cv::Scalar::all(0));

        void draw_ENLSVG_ScannerStacks(const Pathfinding::ScannerStacks &scanner_stacks,
                                       const cv::Scalar &color = cv::Scalar::all(0));

        void draw_DistMap(freeNav::DimensionLength *dimension,
                          const std::vector<PathLen>& dist_map,
                          const Pointi<2>& offset = Pointi<2>(), // offset of grid space
                          double max_dist = 100,
                          double min_dist = 100);

        void draw_Block(const freeNav::Pointi<2>& min_pt, const freeNav::Pointi<2>& max_pt);

        void drawVoronoiGraph(const freeNav::RimJump::BlockDetectorPtr<2>& block_detector,
                              freeNav::DimensionLength *dimension, int hyper_node_id);

        void drawHeuristicTable(const CBS::HeuristicTable& ht, DimensionLength* dim);

        void resetCanvas(const cv::Scalar &color = cv::Scalar(255, 255, 255));

        Pointd<2> transformToWorld(const Pointi<2> &pt);


        template <typename T>
        Pointi<2> transformToPixel(const Point<T, 2> &pt)  {
            return transformToPixel(pt[0], pt[1]);
        }

        template <typename T>
        Pointi<2> transformToPixel(T x, T y) {
            Pointi<2> v;
            v[0] = center_[0] + x * resolution_;
            v[1] = center_[1] - y * resolution_;
            return v;
        }
        void setMouseCallBack(void (*func)(int, int, int, int, void *));

        /* default FPS set to 30 */
        int show(int ms = 33);

        double zoom_ratio_ = 1;

        void (*mouse_call_back_func_)(int, int, int, int, void *) = nullptr;

        cv::Mat getCanvas() { return canvas_; }

        void setCanvas(const cv::Mat& canvas) {
            canvas_ = canvas;
        }

    private:

        std::vector<cv::Scalar> gradation_color_table_;

        cv::Mat canvas_;

        /* how many pixel corresponding to 1 meter */
        double resolution_;

        /* center of the real world coordinate */
        Pointi<2> center_;

        std::string name_;


    };

}
#endif //FREENAV_CANVAS_H

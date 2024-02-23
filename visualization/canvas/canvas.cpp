//
// Created by yaozhuo on 2021/9/20.
//

#include <Eigen/Dense>
#include "canvas.h"
#include "rim_jump/graph_construction/tangent_graph_build.h"
#include "iomanip"
#include "path_smooth/optimal_planner.h"
namespace freeNav {

    Canvas::Canvas(std::string name, int size_x, int size_y, double resolution, int zoom_ratio) :
            canvas_(size_y * zoom_ratio, size_x * zoom_ratio, CV_8UC3, cv::Scalar::all(255)), resolution_(resolution) {
        center_[0] = size_x / 2;
        center_[1] = size_y / 2;
        name_ = name;
        zoom_ratio_ = zoom_ratio;
        setColorTable();
        cv::namedWindow(name_, CV_WINDOW_NORMAL);
    }

    void Canvas::setColorTable() {
        /* set gradation color table, from blue -> green -> red */
        for (double i = 0; i < 1.; i += .1) {
            gradation_color_table_.push_back(cv::Scalar(0, 255 * (1. - i), 255 * i));
        }
        for (double i = 0; i < 1.; i += .1) {
            gradation_color_table_.push_back(cv::Scalar(255 * (i), 0, 255 * (1. - i)));
        }
        std::vector<cv::Scalar> reverse_color_table(gradation_color_table_.rbegin(), gradation_color_table_.rend());
        gradation_color_table_.insert(gradation_color_table_.end(), reverse_color_table.begin(),
                                      reverse_color_table.end());
    }

    void Canvas::drawLineInt(int x1, int y1, int x2, int y2, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::line(canvas_,
                 cv::Point2i(x1 * zoom_ratio_, y1 * zoom_ratio_) + cv::Point(offset, offset),
                 cv::Point2i(x2 * zoom_ratio_, y2 * zoom_ratio_) + cv::Point(offset, offset),
                 color, 1, cv::LINE_AA);
    }

    void Canvas::drawLineInt(const Fraction& x1, const Fraction& y1, const Fraction& x2, const Fraction& y2, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;

        int round_x1 = round(x1.toFloat()*zoom_ratio_), round_y1 = round(y1.toFloat()*zoom_ratio_);
        int round_x2 = round(x2.toFloat()*zoom_ratio_), round_y2 = round(y2.toFloat()*zoom_ratio_);

        cv::line(canvas_,
                 cv::Point2i(round_x1, round_y1) + cv::Point(offset, offset),
                 cv::Point2i(round_x2, round_y2) + cv::Point(offset, offset),
                 color, 1, cv::LINE_AA);
    }

    void Canvas::drawLine(double x1, double y1, double x2, double y2, int line_width, const cv::Scalar &color) {
        Pointi<2> pti1 = transformToPixel(x1, y1);
        Pointi<2> pti2 = transformToPixel(x2, y2);
        drawLineInt(pti1[0], pti1[1], pti2[0], pti2[1], false, line_width, color);
    }

    void Canvas::drawPointInt(int x, int y, const cv::Vec3b &color) {
        canvas_.at<cv::Vec3b>(x * zoom_ratio_, y * zoom_ratio_) = color;
    }

    void Canvas::drawPoint(double x, double y, const cv::Vec3b &color) {
        Pointi<2> pti = transformToPixel(x, y);
        drawPointInt(pti[0], pti[1], color);
    }

    void Canvas::drawCircleInt(int x, int y, int radius, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::circle(canvas_, cv::Point(x * zoom_ratio_, y * zoom_ratio_) + cv::Point(offset, offset), radius, color, line_width, cv::LINE_AA);
    }

    void Canvas::drawCircleInt(const Fraction& x, const Fraction& y, int radius, bool center_offset, int line_width, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        int round_x = round(x.toFloat()*zoom_ratio_), round_y = round(y.toFloat()*zoom_ratio_);
        cv::circle(canvas_, cv::Point(round_x, round_y) + cv::Point(offset, offset), radius, color, line_width, cv::LINE_AA);
    }

    void Canvas::drawCircle(double x, double y, double radius, int line_width, const cv::Scalar &color) {
        Pointi<2> pti = transformToPixel(x, y);
        int radius_i = radius * resolution_;
        drawCircleInt(pti[0], pti[1], radius_i, false, line_width, color);
    }

    void Canvas::resetCanvas(const cv::Scalar &color) {
        canvas_ = cv::Mat(canvas_.rows, canvas_.cols, CV_8UC3, cv::Scalar::all(255));
    }

    Pointd<2> Canvas::transformToWorld(const Pointi<2> &pt) {
        Pointd<2> v;
        v[0] = (pt[0] - center_[0]) / resolution_;
        v[1] = -(pt[1] - center_[1]) / resolution_;
        return v;
    }

    void canvasMouseCallBack(int event, int x, int y, int flags, void *canvas) {
        Canvas *canvas_ptr = reinterpret_cast<Canvas *>(canvas);
        int x_zoomed = x / canvas_ptr->zoom_ratio_;
        int y_zoomed = y / canvas_ptr->zoom_ratio_;
        if (canvas_ptr->mouse_call_back_func_ != nullptr) {
            (*(canvas_ptr->mouse_call_back_func_))(event, x_zoomed, y_zoomed, flags, nullptr);
        } else {
            std::cout << " canvas_ptr->mouse_call_back_func_ = nullptr !" << std::endl;
            exit(0);
        }
    }


    void Canvas::setMouseCallBack(void (*func)(int, int, int, int, void *)) {
        mouse_call_back_func_ = func;
        cv::setMouseCallback(name_, canvasMouseCallBack, this);
    }

    int Canvas::show(int ms) {
        cv::imshow(name_, canvas_);
        return cv::waitKey(ms);
    }

    void Canvas::drawAxis(double x_range, double y_range, double wing_length) {

        drawLine(-x_range, 0., x_range, 0., 1, cv::Scalar::all(0));
        drawLine(0., -y_range, 0., y_range, 1, cv::Scalar::all(0));

        drawArrow(x_range, 0., 0., .5, 1, cv::Scalar::all(0));
        drawArrow(0., y_range, M_PI_2, .5, 1, cv::Scalar::all(0));

        for (double i = ceil(-x_range + 1); i <= floor(x_range); i++) {
            if (i == 0) continue;
            drawLine(i, -wing_length, i, wing_length, 1, cv::Scalar::all(0));
        }

        for (double i = ceil(-y_range + 1); i <= floor(y_range); i++) {
            if (i == 0) continue;
            drawLine(-wing_length, i, wing_length, i, 1, cv::Scalar::all(0));
        }

    }

    void
    Canvas::drawArrow(double x, double y, double theta, double arrow_length, int line_width, const cv::Scalar &color) {
        Pointi<2> pti = transformToPixel(x, y);
        drawArrowInt(pti[0], pti[1], theta, arrow_length, line_width, color);
    }

    void
    Canvas::drawArrowInt(int x, int y, double theta, double arrow_length, int line_width, const cv::Scalar &color) {
        cv::Point p1(x, y);
        int arrow_length_i = arrow_length * resolution_;
        cv::Point2i p2 = p1 + cv::Point2i(arrow_length_i * cos(theta), -arrow_length_i * sin(theta));
        cv::arrowedLine(canvas_, p1 * zoom_ratio_, p2 * zoom_ratio_, color, line_width, cv::LINE_AA, 0, .2);
    }

    void
    Canvas::drawArrowInt(int x1, int y1, int x2, int y2, int line_width, bool center_offset, const cv::Scalar &color) {
        int offset = center_offset ? .5 * zoom_ratio_ : 0;
        cv::Point p1(x1, y1);
        cv::Point p2(x2, y2);
        cv::arrowedLine(canvas_, p1 * zoom_ratio_ + cv::Point(offset, offset),
                        p2 * zoom_ratio_ + cv::Point(offset, offset), color, line_width, cv::LINE_AA, 0, .1);
    }

    void Canvas::drawPathf(const Pointds<2> &pathd, int line_width, const cv::Scalar &color) {
        if (pathd.empty()) return;
        for (int i = 0; i < pathd.size() - 1; i++) {
            drawLine(pathd[i][0], pathd[i][1], pathd[i + 1][0], pathd[i + 1][1], line_width, color);
        }
    }

    void Canvas::drawPointfs(const Pointds<2> &pathd, double radius, int line_width, const cv::Scalar &color) {
        if (pathd.empty()) return;
        for (int i = 0; i < pathd.size() - 1; i++) {
            drawCircle(pathd[i][0], pathd[i][1], radius, line_width, color);
        }
        drawCircle(pathd.back()[0], pathd.back()[1], radius, line_width, color);
    }

    void Canvas::drawPointfs(const std::vector<PoseSE2> &path, double radius, int line_width, const cv::Scalar &color) {
        if (path.empty()) return;
        for (int i = 0; i < path.size() - 1; i++) {
            drawCircle(path[i].x(), path[i].y(), radius, line_width, color);
        }
        drawCircle(path.back().x(), path.back().y(), radius, line_width, color);
    }

    void Canvas::drawPointfs(const std::vector<PoseSE2> &path, const std::vector<double>& time_diffs, double current_time,
                             const Point2dContainer &polygon, int line_width) {
        if (path.empty()) return;
        int color_count = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            drawPolygon(path[i].x(), path[i].y(), path[i].theta(), polygon, line_width,
                        gradation_color_table_[color_count]);
            color_count++;
            color_count = color_count % gradation_color_table_.size();
        }
        drawPolygon(path.back().x(), path.back().y(), path.back().theta(), polygon, line_width,
                    gradation_color_table_[color_count]);
    }

    void Canvas::drawPolygon(double x, double y, double theta, const Point2dContainer &polygon, int line_width,
                             const cv::Scalar &color) {
        PoseSE2 pose(x, y, theta);
        const auto &rotate_matrix = Eigen::Rotation2Dd(pose.theta()).toRotationMatrix();
//        Point2dContainer global_polygon(polygon.size());
//        for (int i = 0; i < global_polygon.size(); i++) {
//            PoseSE2 offset(polygon[i], 0);
//            offset.rotateGlobal(theta);
//            global_polygon[i] = pose.position() + offset.position();
//        }
        Point2dContainer global_polygon = transformedFrom(polygon, pose);
        for (int i = 0; i < global_polygon.size() - 1; i++) {
            drawLine(global_polygon[i].x(), global_polygon[i].y(),
                     global_polygon[i + 1].x(), global_polygon[i + 1].y(),
                     line_width, color);
        }
        drawLine(global_polygon.front().x(), global_polygon.front().y(),
                 global_polygon.back().x(), global_polygon.back().y(),
                 line_width, color);
    }

    void Canvas::drawGrid(int x, int y, const cv::Vec3b &color) {
        if (x < 0 || x >= canvas_.cols / zoom_ratio_ || y < 0 || y >= canvas_.rows / zoom_ratio_) return;
        for (int i = x * zoom_ratio_; i < (x + 1) * zoom_ratio_; i++) {
            for (int j = y * zoom_ratio_; j < (y + 1) * zoom_ratio_; j++) {
                canvas_.at<cv::Vec3b>(j, i) = color;
            }
        }
    }

    void Canvas::drawHeuristicTable(const CBS::HeuristicTable &ht, DimensionLength *dim) {
        for(int i=0; i<ht.size(); i++) {
            if(ht[i] != MAX<CBS::HeuristicValue>) {
                Pointi<2> pt = IdToPointi<2>(i, dim);
                std::stringstream ss;
                ss << ht[i];
                drawTextInt(pt[0], pt[1], ss.str().c_str(), cv::Vec3b::all(0));
            }
        }
    }

    void
    Canvas::drawGridLine(int x1, int y1, int x2, int y2, int line_width, bool center_offset, const cv::Scalar &color) {
        double offset = center_offset ? .5 : 0;
        cv::line(canvas_,
                 cv::Point2f((x1 + offset) * zoom_ratio_, (y1 + offset) * zoom_ratio_),
                 cv::Point2f((x2 + offset) * zoom_ratio_, (y2 + offset) * zoom_ratio_),
                 color, line_width, cv::LINE_AA);
        cv::circle(canvas_, cv::Point2f((x1 + offset) * zoom_ratio_, (y1 + offset) * zoom_ratio_), 2, color, -1);
        cv::circle(canvas_, cv::Point2f((x2 + offset) * zoom_ratio_, (y2 + offset) * zoom_ratio_), 2, color, -1);

    }


    void Canvas::drawGridMap(freeNav::DimensionLength *dimension,
                             IS_OCCUPIED_FUNC<2> is_occupied) {
        if (dimension[0] > canvas_.cols * zoom_ratio_ || dimension[1] > canvas_.rows * zoom_ratio_) { return; }
        for (int i = 0; i < dimension[0]; i++) {
            for (int j = 0; j < dimension[1]; j++) {
                freeNav::Pointi<2> pt;
                pt[0] = i;
                pt[1] = j;
                if (is_occupied(pt)) {
                    //std::cout << pt << " is occ " << std::endl;
                    drawGrid(i, j);
                }
                //else { std::cout << pt << " is free " << std::endl; }
            }
        }
    }

    void Canvas::drawGridMap(const freeNav::RimJump::MapDownSampler<2>& down_sampler, int down_sample_level) {
        const auto& dimension = down_sampler.raw_dimension_info_;
        const auto& is_occupied = down_sampler.raw_is_occupied_;
        if(down_sample_level == 0) {
            drawGridMap(dimension, is_occupied);
        } else {
            for (int i = 0; i < dimension[0]; i++) {
                for (int j = 0; j < dimension[1]; j++) {
                    freeNav::Pointi<2> pt({2, 2});
                    pt[0] = i;
                    pt[1] = j;
                    if (down_sampler.isOccupied(pt, down_sample_level)) { drawGrid(i, j); }
                }
            }
        }
    }

    void Canvas::drawENLVisibilityGraph(const Pathfinding::ENLSVG::VisibilityGraph &graph) {
        for (const auto &edge : graph.edges) {
            //if (edge.level % 2 == 1) continue;
            auto color = cv::Scalar::all(20 * (edge.level % 10));
            if (edge.level != Pathfinding::ENLSVG::VisibilityGraph::LEVEL_W) {
                color = 200;
                continue;
            }
            int sx = graph.vertices[edge.sourceVertex].x,
                    sy = graph.vertices[edge.sourceVertex].y,
                    ex = graph.vertices[edge.destVertex].x,
                    ey = graph.vertices[edge.destVertex].y;
            drawGridLine(sx, sy, ex, ey, 1, false, cv::Vec3b(0,255,0));
        }
    }

    void Canvas::drawENLVisibilityGraph(const Pathfinding::ENLSVG::VisibilityGraph &graph, const Pathfinding::ENLSVG::Memory& memory) {
        for (int i=0;i<graph.edges.size();i++) {
            const auto& edge = graph.edges[i];
            //if (edge.level % 2 == 1) continue;
            auto color = cv::Scalar::all(20 * (edge.level % 10));
//            if (edge.level != freeNav::RimJump::VisibilityGraph<2>::LEVEL_W) {
//                continue;
//            }
            if (!memory.markedEdges.isMarked[i]) {
                continue;
            }
//            if(edge.level != Pathfinding::ENLSVG::VisibilityGraph::LEVEL_W) continue;
//            if (!graph.isHyperEdge_[i]) {
//                color = cv::Vec3b(0,255,0);
//                //continue;
//            } else {
//                color = cv::Vec3b(0,0,255);
//            }
            const auto& grid_start  = graph.vertices[edge.sourceVertex];
            const auto& grid_target = graph.vertices[edge.destVertex];

//            if(memory.visited(grid_start->id_))
//            {
//                drawCircleInt(grid_start->pt_[0], grid_start->pt_[1], 5, false, -1, cv::Vec3b(0,255,0));
//            }
//            if(memory.visited(grid_target->id_))
//            {
//                drawCircleInt(grid_target->pt_[0], grid_target->pt_[1], 5, false, -1, cv::Vec3b(0,255,0));
//            }

            int sx = grid_start.x,
                    sy = grid_start.y,
                    ex = grid_target.x,
                    ey = grid_target.y;
            // drawGridLine(sx, sy, ex, ey, 1, false, color);
            drawArrowInt(sx, sy, ex, ey, 1, false, color);

        }
    }

    void Canvas::drawGrids(std::map<Id, freeNav::RimJump::GridPtr<2>> pts, const cv::Vec3b &color) {
        for (const auto &pt : pts) {
            drawGrid(pt.second->pt_[0], pt.second->pt_[1], color);
            //drawTextInt(pt.second->pt_[0], pt.second->pt_[1], std::to_string(pt.second->surface_id_).c_str(), cv::Scalar::all(0));
        }
    }

    void Canvas::drawPointiCircles(const freeNav::Pointis<2> &pts, const cv::Vec3b &color, int radius, int line_width) {
        for (const auto &pt : pts) {
            //drawGrid(pt.second->pt_[0], pt.second->pt_[1], color);
            drawCircleInt(pt[0], pt[1], radius, false, line_width, cv::Scalar(color[0], color[1], color[2]));
            //drawTextInt(pt.second->pt_[0], pt.second->pt_[1], std::to_string(pt.second->surface_id_).c_str(), cv::Scalar::all(0));
        }
    }

    void Canvas::drawGrids(const freeNav::Pointis<2> &pts, const cv::Vec3b &color) {
        for (const auto &pt :pts) {
            drawGrid(pt[0], pt[1], color);
        }
    }

    void Canvas::drawSurfaceGrids(const std::vector<freeNav::RimJump::GridPtr<2>> &sfg, const cv::Vec3b &color,
                                  bool with_nearby) {
        if (with_nearby) {
            for (const auto &grid :sfg) {
                for (const auto &pt : grid->nearby_surface_pts_) {
                    drawGrid(pt[0], pt[1], cv::Vec3b::all(200));
                }
            }
        }
    }
//
//    void Canvas::drawWaveTreeNode(const freeNav::RimJump::WaveTreeNodePtr<2> &wave_tree_node_ptr) {
//        if (wave_tree_node_ptr != nullptr) {
//            drawGrid(wave_tree_node_ptr->pt_[0], wave_tree_node_ptr->pt_[1],
//                     COLOR_TABLE[wave_tree_node_ptr->wave_length_ % 30]);
//            for (const auto &nextNode_ptr : wave_tree_node_ptr->next_wave_nodes_) {
//                drawWaveTreeNode(nextNode_ptr);
//            }
//        }
//    }
//
//    void Canvas::drawWaveTree(const freeNav::RimJump::WaveTree<2> &wave_tree) {
//        if (!wave_tree.empty()) {
//            int color_count = 0;
//            for (const auto &circle : wave_tree) {
//                for (const auto &gird : circle) {
//                    drawGrid(gird->pt_[0], gird->pt_[1], COLOR_TABLE[color_count % 30]);
//                }
//                color_count++;
//            }
//        }
//    }

    void
    Canvas::drawTangentGraphAllNodes(freeNav::RimJump::RoadMapGraphPtr<2> &tg, const cv::Vec3b &color1,
                                     const cv::Vec3b &color2) {
        int color_count = 0;
        for (const auto &edge : tg->nodes_) {
            drawGrid(edge->sg_->pt_[0], edge->sg_->pt_[1], color1);
        }
    }

    void Canvas::drawMarkedInitialEdges(freeNav::RimJump::RoadMapGraphPtr<2> &tg, bool center_offset) {
//        for (int i=0; i<tg.edges_.size(); i++) {
//            if (tg.dist_to(i, true, true, MIN_TIME) != std::numeric_limits<double>::max())
//                drawEdge(tg, tg.edges_[i], false, false, cv::Scalar(0, 255, 0));
//            if (tg.dist_to(i, false, true, MIN_TIME) != std::numeric_limits<double>::max())
//                drawEdge(tg, tg.edges_[i], false, false, cv::Scalar(255, 0, 0));
//        }
    }

    void Canvas::drawTangentGraphEdge(freeNav::DimensionLength dimen[2],
                                      freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                                      const freeNav::Pointi<2> &pt1,
                                      const freeNav::Pointi<2> &pt2,
                                      bool center_offset) {
        Id start_id = freeNav::PointiToId<2>(pt1, dimen), end_id = freeNav::PointiToId<2>(pt2, dimen);
        int color_count = 0;
        //drawGrid(pt1[0], pt1[1], COLOR_TABLE[0]);
        //drawGrid(pt2[0], pt2[1], COLOR_TABLE[1]);
        auto start_grid_ptr = tg->surface_processor_->grid_map_[start_id];
        if (start_grid_ptr != nullptr && start_grid_ptr->node_id_ != MAX<NodeId>) {
            RimJump::RoadMapEdgeTraitPtr<2> edge_iter = nullptr;//start_iter->second->split_edges_.find(end_id);
            EdgeId edge_id;//start_iter->second->split_edges_.find(end_id);
            for(auto edge_temp_id : tg->nextEdges(tg->nodes_[start_grid_ptr->node_id_], true)) {
                auto edge_temp = tg->edges_[edge_temp_id];
                if(tg->nodes_[edge_temp->nextNode(true)]->sg_->id_ == end_id) {
                    edge_iter = edge_temp;
                    edge_id = edge_temp_id;
                }
            }
            if(edge_iter != nullptr) {
                drawEdge(tg, edge_iter, center_offset, false, false, cv::Scalar(0, 0, 255));

                // draw edge
                for (auto &next_edge_id : tg->nextEdges(tg->edges_[edge_id], true, true)) {
                    drawEdge(tg, tg->edges_[next_edge_id], center_offset, false, false, cv::Scalar(255, 0, 0));
                }
                for (auto &pre_edge_id : tg->nextEdges(tg->edges_[edge_id], false, true)) {
                    drawEdge(tg, tg->edges_[pre_edge_id], center_offset, false, false, cv::Scalar(0, 255, 0));
                }
            }

        }
    }

    void Canvas::drawRoadMapEdges(freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                                  const freeNav::RimJump::RoadMapEdgeTraitPtrs<2> &edges,
                                  bool center_offset, const cv::Scalar &color1) {
        for (const auto &edge : edges) {
            drawEdge(tg, edge, center_offset, false, false, color1);
        }
    }


    void Canvas::drawRoadMapEdgeAsPath(RimJump::GeneralGraphPathPlannerWithEdge<2>& g2p2, freeNav::RimJump::RoadMapEdgeTraitPtr<2> edge, bool center_offset, bool is_edge, bool is_start,
                                       const cv::Scalar &color) {
        if(edge == nullptr) return;
        //if (is_start) edge = edge->current_on_edge(is_edge, MIN_TIME);
        if(edge == nullptr) return;
        drawEdge(g2p2.tg_, edge, center_offset, false, false, color);
        auto& data = g2p2.data_;
        auto & ns = g2p2.tg_->nodes_;
        auto & es = g2p2.tg_->edges_;
        auto & hes = g2p2.tg_->hyper_edges_;

        if (data->dist_to(edge, true, is_edge, MIN_TIME) != freeNav::MAX<freeNav::PathLen>)
            drawTextInt((3 * ns[edge->nextNode(false)]->sg_->pt_[0] + ns[edge->nextNode(true)]->sg_->pt_[0]) / 4,
                        (3 * ns[edge->nextNode(false)]->sg_->pt_[1] + ns[edge->nextNode(true)]->sg_->pt_[1]) / 4,
                        std::to_string((int) (data->dist_to(edge, true, true, MIN_TIME))).c_str(), cv::Scalar(0, 255, 0));
        if (data->dist_to(edge, false, is_edge, MIN_TIME) != freeNav::MAX<freeNav::PathLen>)
            drawTextInt((ns[edge->nextNode(false)]->sg_->pt_[0] + 3 * ns[edge->nextNode(true)]->sg_->pt_[0]) / 4,
                        (ns[edge->nextNode(false)]->sg_->pt_[1] + 3 * ns[edge->nextNode(true)]->sg_->pt_[1]) / 4,
                        std::to_string((int) (data->dist_to(edge, false, true, MIN_TIME))).c_str(), cv::Scalar(255, 0, 0));

        auto to_start = data->close_edge_to(edge, true, is_edge, MIN_TIME);
        while (to_start != MAX<EdgeId>) {
            drawEdge(g2p2.tg_, es[to_start], center_offset, false, false, color);
            drawTextInt((3 * ns[es[to_start]->nextNode(false)]->sg_->pt_[0] + ns[es[to_start]->nextNode(true)]->sg_->pt_[0]) / 4,
                        (3 * ns[es[to_start]->nextNode(false)]->sg_->pt_[1] + ns[es[to_start]->nextNode(true)]->sg_->pt_[1]) / 4,
                        std::to_string((int) (data->dist_to(es[to_start], true, true, MIN_TIME))).c_str(), cv::Scalar(0, 255, 0));
            to_start = data->close_edge_to(es[to_start], true, is_edge, MIN_TIME);
        }
        auto to_target = data->close_edge_to(edge, false, is_edge, MIN_TIME);
        while (to_target != MAX<EdgeId>) {
            drawEdge(g2p2.tg_, es[to_target], center_offset, false, false, color);
            drawTextInt((ns[es[to_target]->nextNode(false)]->sg_->pt_[0] + 3 * ns[es[to_target]->nextNode(true)]->sg_->pt_[0]) / 4,
                        (ns[es[to_target]->nextNode(false)]->sg_->pt_[1] + 3 * ns[es[to_target]->nextNode(true)]->sg_->pt_[1]) / 4,
                        std::to_string((int) (data->dist_to(es[to_target], false, true, MIN_TIME))).c_str(), cv::Scalar(255, 0, 0));
            to_target = data->close_edge_to(es[to_target], false, is_edge, MIN_TIME);
        }
    }

    void Canvas::drawPaths(const freeNav::Paths<2> &paths) {
        int color_count = 0;
        for (const auto &path : paths) {
            drawPath(path, true, COLOR_TABLE[color_count % 30]);
            color_count++;
        }
    }

    void Canvas::drawPath(const freeNav::Path<2> &path, bool center_offset, const cv::Scalar &color) {
        if (path.size() <= 1) return;
        for (int i = 0; i < path.size() - 1; i++) {
            //drawGridLine(path[i][0], path[i][1], path[i + 1][0], path[i + 1][1], 1, center_offset, color);
            drawArrowInt(path[i+1][0], path[i+1][1], path[i][0], path[i][1], 2, center_offset, color);
        }
        auto arrow_tail = path[path.size() - 2];
        auto arrow_tip = path[path.size() - 1];
        //drawArrowInt(arrow_tail[0], arrow_tail[1], arrow_tip[0], arrow_tip[1], 2, center_offset, color);
    }

    void Canvas::drawEdges(freeNav::RimJump::RoadMapGraphPtr<2> &tg, const freeNav::RimJump::RoadMapEdgeTraitPtrs<2> &edges, bool center_offset,const cv::Scalar &color1) {
        for (int i=0;i<edges.size();i++) {
            const auto &edge = edges[i];
            drawEdge(tg, edge, center_offset, true, false, COLOR_TABLE[i%30]);
        }
    }

    void Canvas::drawNode(freeNav::RimJump::RoadMapGraphPtr<2> &tg, freeNav::NodeId & id, bool center_offset, const cv::Scalar &color1) {
        auto& ns = tg->nodes_;
        for(const auto& vid : tg->nextNodes(ns[id])) {
            const auto& pt1 = ns[id]->sg_->pt_;
            const auto& pt2 = ns[vid]->sg_->pt_;
            drawArrowInt(pt1[0], pt1[1], pt2[0], pt2[1],
                         1, center_offset, color1);
        }
    }

    void Canvas::drawNodes(freeNav::RimJump::RoadMapGraphPtr<2> &tg, bool center_offset, const cv::Scalar &color1) {
        auto& ns = tg->nodes_;
        for(NodeId i=0; i<ns.size(); i++) {
            drawNode(tg, i, center_offset, color1);
        }
    }

    void Canvas::draw_DistMap(freeNav::DimensionLength *dimension,
                              const std::vector<PathLen>& dist_map,
                              const Pointi<2>& offset,
                              double max_dist, double min_dist) {

        for(int id=0; id<dist_map.size(); id++) {
            Pointi<2> pt = IdToPointi<2>(id, dimension) + offset;
            if(dist_map[id] == MAX<PathLen>) continue;
            if(dist_map[id] > 0) {
                drawGrid(pt[0], pt[1], cv::Vec3b(255,200*dist_map[id]/max_dist + 50,255));
            } else {
                double color = 200*dist_map[id]/(-max_dist) + 50;
                drawGrid(pt[0], pt[1], cv::Vec3b(color, color, color));
            }
        }

    }

    void Canvas::draw_Block(const freeNav::Pointi<2>& min_pt, const freeNav::Pointi<2>& max_pt) {
        drawLineInt(min_pt[0], min_pt[1], min_pt[0], max_pt[1], false);
        drawLineInt(min_pt[0], min_pt[1], max_pt[0], min_pt[1], false);
        drawLineInt(min_pt[0], max_pt[1], max_pt[0], max_pt[1], false);
        drawLineInt(max_pt[0], min_pt[1], max_pt[0], max_pt[1], false);
    }

    void Canvas::drawVoronoiGraph(const freeNav::RimJump::BlockDetectorPtr<2>& block_detector,
                                  freeNav::DimensionLength *dimension, int hyper_node_id) {
        const auto& hyper_pts = block_detector->hyper_pts_;
        const auto& voronoi_pts = block_detector->voronoi_pts_;
        const auto& voronoi_id_map = block_detector->voronoi_id_map_;
        const auto& voronoi_graph = block_detector->voronoi_graph_;
        const auto& hyper_voronoi_nodes = block_detector->hyper_vnode_ptrs_;
        Id id;
        // draw node with hyper group id
//        for(const auto& pt : voronoi_pts)
//        {
//            id = PointiToId(pt, dimension);
//            if(voronoi_graph[voronoi_id_map[id]].visible_hypers_.empty()) { continue; }
//            const auto& hnode = voronoi_graph[voronoi_id_map[id]];
//            Id total_hnode = 0;
//            for(const auto& hyper_pair : hnode.visible_hypers_) {
//                total_hnode += hyper_pair.first;
//            }
//            drawGrid(pt[0], pt[1], COLOR_TABLE[total_hnode % 30]);
//        }

          // draw hyper node with color
//        for(const auto& node : hyper_voronoi_nodes) {
//            for(const auto& pt : node->pts_) {
//                drawGrid(pt[0], pt[1], COLOR_TABLE[node->hyper_group_id_ % 30]);
//            }
//        }
        const auto& hyper_voronoi_edges = block_detector->hyper_vedge_ptrs_;
        for(int i=0; i<hyper_voronoi_edges.size(); i++) {
            const auto& edge = hyper_voronoi_edges[i];
            for(const auto& pt : edge->pts_) {
                drawGrid(pt[0], pt[1], COLOR_TABLE[i % 30]);
            }
        }
    }

    void Canvas::drawNodes(RimJump::RoadMapGraphPtr<2> &tg,
                           RimJump::DynamicDataOfSearchWithNodePtr<2> data,
                           bool center_offset, const cv::Scalar &color1) {
        auto& ns = tg->nodes_;
        for(NodeId i=0; i<ns.size(); i++) {
            if(data->close_node_to(ns[i], false, MIN_TIME) != MAX<NodeId>) {
                const auto & pt1 = ns[i]->sg_->pt_;
                const auto & pt2 = ns[data->close_node_to(ns[i], false, MIN_TIME)]->sg_->pt_;

                drawCircleInt(pt1[0], pt1[1], 1, false, -1, cv::Vec3b(0,255,0));

                //drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], center_offset, 1, color1);

            }
            if(data->close_node_to(ns[i], true, MIN_TIME) != MAX<NodeId>) {
                const auto & pt1 = ns[i]->sg_->pt_;
                const auto & pt2 = ns[data->close_node_to(ns[i], true, MIN_TIME)]->sg_->pt_;
                drawArrowInt(pt2[0], pt2[1], pt1[0], pt1[1], 1, center_offset, cv::Vec3b(255,0,0));
            }
        }
    }

    void Canvas::drawEdgess(freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                            const freeNav::RimJump::RoadMapEdgeTraitPtrss<2> &edgess,
                            bool center_offset) {
        for (int i = 0; i < edgess.size(); i++) {
            drawEdges(tg, edgess[i], center_offset, COLOR_TABLE[i]);
        }
    }

    void Canvas::drawEdge(freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                          const freeNav::RimJump::RoadMapEdgeTraitPtr<2> &edge,
                          bool center_offset,
                          bool only_loop, bool draw_branch,
                          const cv::Scalar &color1) {
        if(edge == nullptr) return;
        if (only_loop) {
            if (!edge->isLoopEdge()) return;
            if(edge->nextEdges(false).empty() || edge->nextEdges(true).empty()) return;
            //if(!freeNav::RimJump::RoadMapGraphBuilder<2>::isHyperLoopEdge(edge)) return;
            //if (!edge->isHyperLoopEdge()) return;
        }
        //if(edge->isLoopEdge()) return;
        //if(!edge->is_hyper_edge_node_) return;
        // draw arrow cause error in big map
        auto node_pre_ptr  = tg->nodes_[edge->nextNode(false)];
        auto node_next_ptr = tg->nodes_[edge->nextNode(true)];

        drawArrowInt(node_pre_ptr->sg_->pt_[0], node_pre_ptr->sg_->pt_[1], node_next_ptr->sg_->pt_[0], node_next_ptr->sg_->pt_[1],
                     2, center_offset, color1);

        //drawGridLine(edge->nextNode(false)->sg_->pt_[0], edge->nextNode(false)->sg_->pt_[1], edge->nextNode(true)->sg_->pt_[0], edge->nextNode(true)->sg_->pt_[1], 1, false, color1);
        if (!edge->isLoopEdge()) {
            //drawTextInt((3*edge->nextNode(false)->sg_->pt_[0] + edge->nextNode(true)->sg_->pt_[0]) / 4, (3*edge->nextNode(false)->sg_->pt_[1] + edge->nextNode(true)->sg_->pt_[1]) / 4, std::to_string(edge->level_).c_str(), cv::Scalar(0,255, 0));
        } else {
            //drawTextInt((3*edge->nextNode(false)->sg_->pt_[0] + edge->nextNode(true)->sg_->pt_[0]) / 4, (3*edge->nextNode(false)->sg_->pt_[1] + edge->nextNode(true)->sg_->pt_[1]) / 4, "INF", cv::Scalar(0,0, 255));
        }
        //std::stringstream ss;
        //ss << edge->tarjan_time_ << "|" << edge->tarjan_earliest_time_;
        //drawTextInt((3*edge->from_->sg_->pt_[0] + edge->to_->sg_->pt_[0]) / 4, (3*edge->from_->sg_->pt_[1] + edge->to_->sg_->pt_[1]) / 4, ss.str().c_str(), color1);

        if (!edge->isLoopEdge()) {
            //drawTextInt((3*edge->from_->sg_->pt_[0] + edge->to_->sg_->pt_[0]) / 4, (3*edge->from_->sg_->pt_[1] + edge->to_->sg_->pt_[1]) / 4, std::to_string(edge->level_).c_str(), COLOR_TABLE[4]);
        }
        if (!edge->isLoopEdge()) {
            //drawTextInt((3*edge->from_->sg_->pt_[0] + edge->to_->sg_->pt_[0]) / 4, (3*edge->from_->sg_->pt_[1] + edge->to_->sg_->pt_[1]) / 4, std::to_string(edge->level_).c_str(), COLOR_TABLE[4]);
        }

        int color_count = 0;
        if (draw_branch) {
//            auto& grid_map = tg.surface_processor_;
//            auto& edges = tg.edges_;
//            auto& nodes = tg.nodes_;
//            for (const auto &next_edge_id : tg.edge_next_edges(edge->edge_id(), true, true)) {
//                drawArrowInt(nodes[edge->nextNode(true)]->sg_->pt_[0],
//                             nodes[edge->nextNode(true)]->sg_->pt_[1],
//                             nodes[edges[next_edge_id]->nextNode(true)]->sg_->pt_[0],
//                             nodes[edges[next_edge_id]->nextNode(true)]->sg_->pt_[1], 1, true, COLOR_TABLE[color_count % 30]);
//                color_count++;
//            }
//            for (const auto &pre_edge_id : tg.edge_next_edges(edge->edge_id(), false, true)) {
//                drawArrowInt(nodes[edges[pre_edge_id]->nextNode(false)]->sg_->pt_[0],
//                             nodes[edges[pre_edge_id]->nextNode(false)]->sg_->pt_[1],
//                             nodes[edge->nextNode(true)]->sg_->pt_[0],
//                             nodes[edge->nextNode(true)]->sg_->pt_[1], 1, true,
//                             COLOR_TABLE[color_count % 30]);
//                color_count++;
//            }
        }
    }

    void
    Canvas::drawTangentGraphLegalEdges(freeNav::RimJump::RoadMapGraphPtr<2> &tg,
                                       bool center_offset,
                                       const cv::Vec3b &color1,
                                       const cv::Vec3b &color2) {
        freeNav::Pointi<2> pt1, pt2, pt3, pt4;
        auto & edges = tg->edges_;
        auto & nodes = tg->nodes_;
        for (auto &node : tg->nodes_) {
            // for each edge in the graph
            for (auto &edge_id : tg->nextEdges(node, true)) {
                auto &next_edges = tg->nextEdges(tg->edges_[edge_id], true, true);
                //if(!next_edges.empty()) continue;
                pt1 = nodes[edges[edge_id]->nextNode(false)]->sg_->pt_;
                pt2 = nodes[edges[edge_id]->nextNode(true)]->sg_->pt_;
                // draw current edge
                drawGrid(pt2[0], pt2[1], color1);
                drawGridLine(pt1[0], pt1[1], pt2[0], pt2[1], 1, true, color2);
                // draw each next edges
                for (auto next_edge_id : next_edges) {
                    pt3 = nodes[edges[next_edge_id]->nextNode(false)]->sg_->pt_;
                    pt4 = nodes[edges[next_edge_id]->nextNode(true)]->sg_->pt_;
                    // draw next edge
                    drawGridLine(pt3[0], pt3[1], pt4[0], pt4[1], 1, true, color2);
                }
            }
        }
    }

//     void Canvas::drawPath(const std::vector<freeNav::RimJump::GridNode> &path, const cv::Scalar &color) {
//        if (path.empty()) return;
//        for (int i = 0; i < path.size() - 1; i++) {
//            drawGridLine(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y, 1, false, color);
//        }
//    }


    void Canvas::drawPathi(const std::vector<Pathfinding::GridVertex> &path, const cv::Scalar &color) {
        if (path.empty()) return;
        for (int i = 0; i < path.size() - 1; i++) {
            drawGridLine(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y, 1, false, color);
        }
    }


    void Canvas::drawEmptyGrid() {
        if (zoom_ratio_ > 5) {
            for (int i = 0; i < canvas_.cols; i += zoom_ratio_) {
                cv::line(canvas_, cv::Point2i(i, 0), cv::Point2i(i, canvas_.rows), COLOR_TABLE[10], 1);
            }
            for (int j = 0; j < canvas_.rows; j += zoom_ratio_) {
                cv::line(canvas_, cv::Point2i(0, j), cv::Point2i(canvas_.cols, j), COLOR_TABLE[10], 1);
            }
        }
    }

    void Canvas::drawTangentPoints(const freeNav::Pointi<2> &start,
                                   const freeNav::RimJump::RoadMapNodePtrs<2> &tpts) {
        int color_count = 0;
        for (const auto &tpt : tpts) {
            freeNav::Pointi<2> end = tpt->sg_->pt_;
            drawGridLine(start[0], start[1], end[0], end[1], 1, true, COLOR_TABLE[color_count % 30]);
            color_count++;
        }
    }

//    void Canvas::drawTangentPathPoints(freeNav::RimJump::PathPointWithEdgePtrs<2> pps, const freeNav::RimJump::Pointi<2> &start,
//                                       const freeNav::RimJump::Pointi<2> &target) {
//        for (int i = 0; i < pps.size(); i++) {
//            drawTangentPathPoint(pps[i], start, target, COLOR_TABLE[i]);
//        }
//    }

//    void Canvas::drawTangentPathPoint(freeNav::RimJump::PathPointWithEdgePtr<2> pps, const freeNav::RimJump::Pointi<2> &start,
//                                      const freeNav::RimJump::Pointi<2> &target, const cv::Scalar &color) {
//        freeNav::RimJump::PathPointWithEdgePtr<2> ppt = pps;
//        freeNav::RimJump::PathPointWithEdgePtr<2> pre_ppt = nullptr;
//
//        freeNav::RimJump::Pointi<2> current(0);
//        freeNav::RimJump::Pointi<2> pre(0);
//        while (ppt != nullptr) {
//            //pre_ppt = ppt->from();
//            pre = ppt->last_edge_->nextNode(false)->sg_->pt_;
//            current = ppt->last_edge_->nextNode(true)->sg_->pt_;
//            drawGridLine(current[0], current[1], pre[0], pre[1], 1, true, color);
//            ppt = ppt->from();
//        }
//    }

    void Canvas::drawTextInt(int x, int y, const char *string, const cv::Scalar &color, double scale, bool center_offset) {

        int offset = center_offset ? .5 * zoom_ratio_ : 0;


        cv::putText(canvas_,
                    cv::String(string),
                    cv::Point2i((x) * zoom_ratio_ + offset, (y) * zoom_ratio_ + offset),
                    cv::FONT_HERSHEY_COMPLEX,
                    scale, color, 2);
    }

    void Canvas::draw_RimJump_Extent(const freeNav::RimJump::Extent &extents, freeNav::DimensionLength dimen[2], double scale) {
        for (int i = 0; i < extents.size(); i++) {
            const freeNav::Pointi<2> pt = freeNav::IdToPointi<2>(i, dimen);
            if (isOutOfBoundary(pt, dimen)) continue;
            drawTextInt(pt[0], pt[1], std::to_string(extents[i]).c_str(), {0, 0, 255}, scale);
        }
    }

    void Canvas::draw_RimJump_Intervals(const FractionPair& start,const RimJump::IntervalPtrs& prev_halfs, const cv::Scalar &color) {
        for(const auto& interval : prev_halfs) {
            drawLineInt(interval->xL_, interval->y_, interval->xR_, interval->y_, true, 4, color);
            drawLineInt(interval->xL_, interval->y_, start.first, start.second, true, 2, color);
            drawLineInt(start.first, start.second, interval->xR_, interval->y_, true, 2, color);

            drawCircleInt(interval->xL_, interval->y_, 5, true, 1, color);
            drawCircleInt(interval->xR_, interval->y_, 5, true, 1, color);
        }
    }

    void Canvas::draw_RimJump_IntervalTree(const FractionPair& start,
                                           const freeNav::RimJump::IntervalTree &tree, freeNav::DimensionLength dimen[2],
                                           const cv::Scalar &color, double scale) {
        for(const auto& intervals : tree) {
            for(const auto& interval : intervals) {

                if(interval == nullptr) continue;

                drawLineInt(interval->xL_, interval->y_, interval->xR_, interval->y_, true, 4, cv::Scalar(0, 255, 0));
                drawLineInt(interval->xL_, interval->y_, start.first, start.second, true, 2, color);
                drawLineInt(start.first, start.second, interval->xR_, interval->y_, true, 2, color);


                int left_width = 1, right_width = 1;

                drawCircleInt(interval->xL_, interval->y_, 5, true, left_width, color);
                drawCircleInt(interval->xR_, interval->y_, 5, true, right_width, color);

            }
        }
    }


    void Canvas::draw_ENLSVG_Extent(const std::vector<int> &extents, freeNav::DimensionLength dimen[2],
                                    double scale) {
        freeNav::DimensionLength internal_dimen[2];
        internal_dimen[0] = dimen[0] + 1; // ENL_SVG internal setting
        internal_dimen[1] = dimen[1] + 2; // ENL_SVG internal setting
        for (int i = 0; i < extents.size(); i++) {
            const freeNav::Pointi<2> pt = freeNav::IdToPointi<2>(i, internal_dimen);
            if (isOutOfBoundary(pt, dimen)) continue;
            drawTextInt(pt[0], pt[1], std::to_string(extents[i]).c_str(), {0, 0, 255}, scale);
        }
    }

    void Canvas::draw_ENLSVG_ScannerStacks(const Pathfinding::ScannerStacks &scanner_stacks, const cv::Scalar &color) {
//        for(const auto& interval : scanner_stacks.intervalStack_backup) {
//            draw_ENLSVG_ScannerInterval(interval);
//        }
        for (const auto &neighbour : scanner_stacks.neighbours) {
            drawCircleInt(neighbour.x, neighbour.y, 5, false, 1, color);
        }
    }

    void Canvas::draw_ENLSVG_ScannerInterval(const Pathfinding::ScanInterval &scanner_interval, const cv::Scalar &color) {

        drawLineInt(scanner_interval.xL.toFloat(), scanner_interval.y, scanner_interval.xR.toFloat(), scanner_interval.y, false);

        drawCircleInt(scanner_interval.xL.toFloat(), scanner_interval.y, 5, false, 1, color);
        drawCircleInt(scanner_interval.xR.toFloat(), scanner_interval.y, 5, false, 1, color);

    }

}








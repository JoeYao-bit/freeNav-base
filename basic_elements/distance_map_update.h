//
// Created by yaozhuo on 2023/8/25.
//

#ifndef FREENAV_BASE_DISTANCE_MAP_UPDATE_H
#define FREENAV_BASE_DISTANCE_MAP_UPDATE_H

#include "../basic_elements/point.h"
#include <memory>
namespace freeNav {

    template <Dimension N>
    bool isOnSurface(const Pointi<N>& pt, const Pointis<N>& nearby_offset, const IS_OCCUPIED_FUNC<N>& is_occupied) {
        bool existing_free = false;
        bool existing_occupied = false;
        for(const auto& offset : nearby_offset) {
            if(is_occupied(pt + offset)) { existing_occupied = true; }
            else { existing_free = true; }
            if(existing_free & existing_occupied) {
                return true;
            }
        }
        return false;
    }

    template<Dimension N>
    class DistanceMapUpdater {
    public:

        DistanceMapUpdater(const IS_OCCUPIED_FUNC<N>& is_occupied, DimensionLength* dim) {
            dimension_info_ = dim;
            is_occupied_ = is_occupied;
            waveFront();
        }

        bool getClosestDistance(const Pointi<N>& pt, PathLen& dist, Pointi<N>& close_pt) const {
            if(isOutOfBoundary(pt, dimension_info_)) {
                std::cout << "DistanceMapUpdater::" << __FUNCTION__ << " pt " << pt << " out of boundary " << std::endl;
                return false;
            }
            Id id = PointiToId(pt, dimension_info_);
            dist = dist_map_[id];
            close_pt = closet_pt_map_[id];
            return true;
        }

        PathLen getClosestDistance(const Id& id) const {
            return dist_map_[id];
        }

    private:

        virtual bool isAllExpansionStop(const std::vector<Pointis<N> > & all_direction_grids) {
            for(const auto & each_direction : all_direction_grids) {
                if(!each_direction.empty()) {
                    return false;
                }
            }
            return true;
        }

        virtual void detectSurfaceGrids() {
            surface_nodes_.clear();
            Id total_index = getTotalIndexOfSpace<N>(dimension_info_);
            nearby_offsets_ = GetNeightborOffsetGrids<N>();
            for(Id i=0; i<total_index; i++) {
                Pointi<N> pt = IdToPointi<N>(i, this->dimension_info_);
                if(isOnSurface(pt, nearby_offsets_, is_occupied_)) {
                    surface_nodes_.push_back(pt);
                }
            }
        }

        virtual void waveFront() {
//            if(surface_nodes_.empty()) {
//                dist_map_ = std::vector<PathLen>(getTotalIndexOfSpace<N>(dimension_info_), 0);
//                return ;
//            }
            dist_map_ = std::vector<PathLen>(getTotalIndexOfSpace<N>(dimension_info_), MAX<PathLen>);
            closet_pt_map_ = std::vector<Pointi<N> >(getTotalIndexOfSpace<N>(dimension_info_));
            all_direction_local_moves_ = initAllDirectionLocalMoves<N>();
            detectSurfaceGrids();
            // set dist of tangent node to obstacle to 1
            for(const auto& pt : surface_nodes_)
            {
                //const auto& pt = tangent_nodes_.back();
                Id id = PointiToId(pt, dimension_info_);
                dist_map_[id] = 0;
                closet_pt_map_[id] = pt;
            }
            Pointis<N> next_expansion;
            max_dist_ = 0;
            //for(const auto& tangent_node : tangent_nodes_)
            {
                std::vector<Pointis<N> > all_direction_grids(2 * N, surface_nodes_);//{tangent_node});

                while (!isAllExpansionStop(all_direction_grids)) {
                    // each tangent node expand simultaneously
                    for (Dimension dim = 0; dim < N * 2; dim++) {
                        next_expansion.clear();
                        for (const auto &current_pt : all_direction_grids[dim]) {
                            const auto& current_pt_expansion = expandCurrentPoint(current_pt, dim);
                            next_expansion.insert(next_expansion.end(), current_pt_expansion.begin(),
                                                  current_pt_expansion.end());
                        }
                        std::swap(all_direction_grids[dim], next_expansion);
                    }
                }
            }
            //std::cout << " max_dist_ " << max_dist_ << " / min_dist_" << min_dist_ << std::endl;
        }

        virtual Pointis<N> expandCurrentPoint(const Pointi<N>& current_pt, int dim) {
            // expand current point
            Pointis<N> next_expansion;
            Id current_id = PointiToId(current_pt, dimension_info_);
            Pointi<N> new_pt;
            Id new_id;
            PathLen new_dist;
            auto& dist_map = dist_map_;
            auto& closet_pt_map = closet_pt_map_;
            bool is_occ = is_occupied_(current_pt);
            bool is_sfc = isOnSurface(current_pt, nearby_offsets_, is_occupied_);
            for(const auto& offset : all_direction_local_moves_[dim])
            {
                new_pt = current_pt + offset;
                if(isOutOfBoundary(new_pt, dimension_info_)) { continue; }
                new_id = PointiToId(new_pt, dimension_info_);
                // ensure expand in the same way
                if(!is_sfc && (is_occupied_(new_pt) ^ is_occ)) { continue; }
                //if(!is_occupied_(new_pt)) continue;

                new_dist = (new_pt-closet_pt_map[current_id]).Norm();
                // stop expansion when hit obstacle, smaller dist to obstacle
                if(dist_map[new_id] == MAX<PathLen>) {
                    //std::cout << " new_dist " << new_dist << std::endl;
                    dist_map[new_id] = new_dist * (is_occ ? -1 : 1);
                    if(!is_occ && max_dist_ < new_dist) { max_dist_ = new_dist; }
                    if(is_occ && min_dist_ < new_dist) { min_dist_ = new_dist; }
                    closet_pt_map[new_id] = closet_pt_map[current_id];
                    next_expansion.push_back(new_pt);

                } else {

                    // considering the order of expansion, allow equal to considering expansion of two nearby corner
                    // with out equal, may cause slit that never visit
                    if(new_dist < fabs(dist_map[new_id])) {
                        dist_map[new_id] = new_dist * (is_occ ? -1 : 1);
                        //std::cout << " new_dist " << new_dist << std::endl;
                        if(!is_occ && max_dist_ < new_dist) { max_dist_ = new_dist; }
                        if(is_occ && min_dist_ < new_dist) { min_dist_ = new_dist; }
                        closet_pt_map[new_id] = closet_pt_map[current_id];
                        next_expansion.push_back(new_pt);
                    }
                }
                //
            }
            return next_expansion;
        }

    public:

        DimensionLength* dimension_info_;

        IS_OCCUPIED_FUNC<N> is_occupied_;

        Pointis<N> surface_nodes_;

        std::vector<Pointis<N> > all_direction_local_moves_; // constant value, during iteration

        // negative for in obstacle zone, positive for in free zone
        std::vector<PathLen> dist_map_;

        std::vector<Pointi<N> > closet_pt_map_;

        // max_dist from free to obstacle / min_dist_ from obstacle to obstacle
        PathLen max_dist_, min_dist_;

        Pointis<N> nearby_offsets_;

    };

    template<Dimension N>
    using DistanceMapUpdaterPtr = std::shared_ptr<DistanceMapUpdater<N> >;

}
#endif //FREENAV_DISTANCE_MAP_UPDATE_H

#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include "../basic_elements/point.h"
namespace freeNav {

    enum GridState {
        OCCUPIED,
        FREE
    };

    typedef std::vector<GridState> GridMap;


    template <freeNav::Dimension N>
    class Grid_Loader {

    public:

        Grid_Loader() {}

        virtual bool isOccupied(const Pointi<N> & pt) const = 0;

        virtual void setOccupied(const Pointi<N> & pt) = 0;

        virtual DimensionLength* getDimensionInfo() {
            return dimen_;
        }

    protected:

        DimensionLength dimen_[N];

        /* the editable map */
        //GridMap grid_map_; // storage the whole map take too many space

    };
}
#endif
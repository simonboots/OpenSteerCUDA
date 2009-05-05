#ifndef _NEIGHBOR_DATA_PROVIDER_H_
#define _NEIGHBOR_DATA_PROVIDER_H_

#include "OpenSteer/NeighborData.h"

namespace OpenSteer {
    class NeighborDataProvider {
    public:
        virtual NeighborData *getNeighborData(void) {
            return d_neighborData;
        }
        
    protected:
        NeighborData* d_neighborData;
    };
} // namespace

#endif // _NEIGHBOR_DATA_PROVIDER_H_
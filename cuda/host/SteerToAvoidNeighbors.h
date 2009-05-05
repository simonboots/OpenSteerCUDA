#ifndef _STEER_TO_AVOID_NEIGHBORS_H_
#define _STEER_TO_AVOID_NEIGHBORS_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/NeighborDataProvider.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerToAvoidNeighbors : public AbstractCUDAKernel
        {
        public:
            SteerToAvoidNeighbors(NeighborDataProvider*, float, float, kernel_options);
            ~SteerToAvoidNeighbors(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            float weight;
            kernel_options options;
            NeighborDataProvider* neighborDataProvider;
            float minTimeToCollision;
        };
    
} // namespace

#endif // _STEER_TO_AVOID_NEIGHBORS_H_
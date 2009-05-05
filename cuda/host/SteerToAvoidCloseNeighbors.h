#ifndef _STEER_TO_AVOID_CLOSE_NEIGHBORS_H_
#define _STEER_TO_AVOID_CLOSE_NEIGHBORS_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/NeighborDataProvider.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerToAvoidCloseNeighbors : public AbstractCUDAKernel
        {
        public:
            SteerToAvoidCloseNeighbors(NeighborDataProvider*, float, float, kernel_options);
            ~SteerToAvoidCloseNeighbors(void);

            void init(void);
            void run(void);
            void close(void);
            
        private:
            float weight;
            kernel_options options;
            NeighborDataProvider* neighborDataProvider;
            float minSeparationDistance;
        };
    
} // namespace

#endif // _STEER_TO_AVOID_CLOSE_NEIGHBORS_H_
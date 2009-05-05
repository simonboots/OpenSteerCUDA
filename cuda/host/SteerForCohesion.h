#ifndef _STEER_FOR_COHESION_H_
#define _STEER_FOR_COHESION_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/NeighborDataProvider.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerForCohesion : public AbstractCUDAKernel
        {
        public:
            SteerForCohesion(NeighborDataProvider*, float, float, float, kernel_options);
            ~SteerForCohesion(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            float weight;
            kernel_options options;
            NeighborDataProvider* neighborDataProvider;
            float maxDistance;
            float cosMaxAngle;
        };
    
} // namespace

#endif // _STEER_FOR_COHESION_H_
#ifndef _STEER_FOR_ALIGNMENT_H_
#define _STEER_FOR_ALIGNMENT_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/NeighborDataProvider.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerForAlignment : public AbstractCUDAKernel
        {
        public:
            SteerForAlignment(NeighborDataProvider*, float, float, float, kernel_options);
            ~SteerForAlignment(void);
            
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

#endif // _STEER_FOR_ALIGNMENT_H_
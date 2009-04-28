#ifndef _STEER_FOR_WANDER_KERNEL_
#define _STEER_FOR_WANDER_KERNEL_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/RandomizedVector.h"

namespace OpenSteer {
    
    class SteerForWanderKernel : public AbstractCUDAKernel
        {
            SteerForWanderKernel(void);
            ~SteerForWanderKernel(void);
            
            void init(void);
            void run(void);
            void close(void);
            void reset(void);
            
        private:
            RandomizedVector *randomizedVector;
            float* d_randomNumbers;
            unsigned int mem_size_random;
            float2* d_wanderData;
            unsigned int mem_size_wander;
        };
    
} // namespace

#endif // _STEER_FOR_WANDER_KERNEL_
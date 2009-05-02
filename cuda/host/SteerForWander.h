#ifndef _STEER_FOR_WANDER_KERNEL_H_
#define _STEER_FOR_WANDER_KERNEL_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/RandomizedVector.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerForWander : public AbstractCUDAKernel
        {
            SteerForWander(float, float, kernel_options);
            ~SteerForWander(void);
            
            void init(void);
            void run(void);
            void close(void);
            void reset(void);
            
        private:
            float dt;
            float weight;
            kernel_options options;
            RandomizedVector *randomizedVector;
            float* d_randomNumbers;
            unsigned int mem_size_random;
            float2* d_wanderData;
            unsigned int mem_size_wander;
        };
    
} // namespace

#endif // _STEER_FOR_WANDER_KERNEL_H_
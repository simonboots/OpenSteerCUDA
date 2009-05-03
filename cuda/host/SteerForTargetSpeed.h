#ifndef _STEER_FOR_TARGET_SPEED_H_
#define _STEER_FOR_TARGET_SPEED_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/TargetSpeedProviderCUDAKernel.h"
#include "OpenSteer/RandomizedVector.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerForTargetSpeed : public AbstractCUDAKernel
        {
        public:
            SteerForTargetSpeed(TargetSpeedProviderCUDAKernel*, float, kernel_options);
            ~SteerForTargetSpeed(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            TargetSpeedProviderCUDAKernel* targetSpeedProvider;
            float weight;
            kernel_options options;
        };
    
} // namespace

#endif // _STEER_FOR_TARGET_SPEED_H_
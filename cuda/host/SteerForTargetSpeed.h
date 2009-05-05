#ifndef _STEER_FOR_TARGET_SPEED_H_
#define _STEER_FOR_TARGET_SPEED_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/TargetSpeedProvider.h"
#include "OpenSteer/RandomizedVector.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerForTargetSpeed : public AbstractCUDAKernel
        {
        public:
            SteerForTargetSpeed(TargetSpeedProvider*, float, kernel_options);
            ~SteerForTargetSpeed(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            TargetSpeedProvider* targetSpeedProvider;
            float weight;
            kernel_options options;
        };
    
} // namespace

#endif // _STEER_FOR_TARGET_SPEED_H_
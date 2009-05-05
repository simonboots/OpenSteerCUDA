#ifndef _STEER_FOR_EVASION_H_
#define _STEER_FOR_EVASION_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/PursuitDataProvider.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerForEvasion : public AbstractCUDAKernel
        {
        public:
            SteerForEvasion(PursuitDataProvider*, float, float, kernel_options);
            ~SteerForEvasion(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            float weight;
            kernel_options options;
            PursuitDataProvider* pursuitDataProvider;
            float maxPredictionTime;
        };
    
} // namespace

#endif // _STEER_FOR_EVASION_H_
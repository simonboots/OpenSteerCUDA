#ifndef _STEER_TO_STAY_ON_PATH_H_
#define _STEER_TO_STAY_ON_PATH_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/PathwayData.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerToStayOnPath : public AbstractCUDAKernel
        {
        public:
            SteerToStayOnPath(float, float, kernel_options);
            ~SteerToStayOnPath(void);
            
            void init(void);
            void run(void);
            void close(void);
            
            void setPathwayData(PathwayData*);
            
        private:
            float weight;
            kernel_options options;
            float predictionTime;
        };
    
} // namespace

#endif // _STEER_TO_STAY_ON_PATH_H_
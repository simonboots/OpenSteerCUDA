#ifndef _STEER_TO_FOLLOW_PATH_H_
#define _STEER_TO_FOLLOW_PATH_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/PathwayData.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerToFollowPath : public AbstractCUDAKernel
        {
        public:
            SteerToFollowPath(float, float, kernel_options);
            ~SteerToFollowPath(void);
            
            void init(void);
            void run(void);
            void close(void);
            
            void setPathwayData(PathwayData*);
            void setDirections(int*);
            
        private:
            float weight;
            kernel_options options;
            float predictionTime;
            
            // device memory pointer
            int* d_directions;
            unsigned int mem_size_directions;
        };
    
} // namespace

#endif // _STEER_TO_FOLLOW_PATH_H_
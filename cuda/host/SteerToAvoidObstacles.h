#ifndef _STEER_TO_AVOID_OBSTACLES_H_
#define _STEER_TO_AVOID_OBSTACLES_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/ObstacleData.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerToAvoidObstacles : public AbstractCUDAKernel
        {
        public:
            SteerToAvoidObstacles(float, kernel_options);
            ~SteerToAvoidObstacles(void);
            
            void init(void);
            void run(void);
            void close(void);
            
            void setObstacles(ObstacleData *obstacleData, int numOfObstacles);
            
        private:
            float weight;
            kernel_options options;
        };
    
} // namespace

#endif // _STEER_TO_AVOID_OBSTACLES_H_
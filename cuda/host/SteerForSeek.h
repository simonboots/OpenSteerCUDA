#ifndef _STEER_FOR_SEEK_KERNEL_H_
#define _STEER_FOR_SEEK_KERNEL_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/SeekVectorProvider.h"
#include "OpenSteer/RandomizedVector.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class SteerForSeek : public AbstractCUDAKernel
        {
        public:
            SteerForSeek(SeekVectorProvider*, float, kernel_options);
            ~SteerForSeek(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            SeekVectorProvider* seekVectorProvider;
            float weight;
            kernel_options options;
        };
    
} // namespace

#endif // _STEER_FOR_SEEK_KERNEL_H_
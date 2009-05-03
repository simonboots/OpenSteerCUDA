#ifndef _UPDATE_H_
#define _UPDATE_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class Update : public AbstractCUDAKernel
        {
        public:
            Update(kernel_options);
            ~Update(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            kernel_options options;
        };
    
} // namespace

#endif // _UPDATE_H_
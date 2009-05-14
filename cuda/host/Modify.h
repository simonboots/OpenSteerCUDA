#ifndef _MODIFY_H_
#define _MODIFY_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class Modify : public AbstractCUDAKernel
        {
        public:
            Modify(kernel_options);
            ~Modify(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            kernel_options options;
        };
    
} // namespace

#endif // _MODIFY_H_
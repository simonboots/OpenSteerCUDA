#ifndef _FIND_HAUNTED_H_
#define _FIND_HAUNTED_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/SeekVectorProvider.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class FindHaunted : public AbstractCUDAKernel, public SeekVectorProvider
        {
        public:
            FindHaunted(unsigned int);
            ~FindHaunted(void);
            
            void init(void);
            void run(void);
            void close(void);
            
        private:
            unsigned int mem_size_seek_vectors;
            unsigned int stride;
        };
    
} // namespace

#endif // _FIND_HAUNTED_H_
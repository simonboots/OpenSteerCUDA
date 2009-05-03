#ifndef _FIND_FOLLOWER_H_
#define _FIND_FOLLOWER_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/SeekVectorProviderCUDAKernel.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class FindFollower : public AbstractCUDAKernel, public SeekVectorProviderCUDAKernel
        {
        public:
            FindFollower();
            ~FindFollower(void);
            
            void init(void);
            void run(void);
            void close(void);
            float3* getSeekVectors(void);
            
        private:
            float3 *d_seekVectors;
            unsigned int mem_size_seek_vectors;
        };
    
} // namespace

#endif // _FIND_FOLLOWER_H_
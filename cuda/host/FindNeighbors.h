#ifndef _FIND_NEIGHBORS_H_
#define _FIND_NEIGHBORS_H_

#include "OpenSteer/AbstractCUDAKernel.h"
#include "OpenSteer/NeighborDataProvider.h"
#include "OpenSteer/Grid.h"
#include "CUDAKernelOptions.cu"

namespace OpenSteer {
    
    class FindNeighbors : public AbstractCUDAKernel, public NeighborDataProvider
    {
    public:
        FindNeighbors(float);
        ~FindNeighbors(void);
        
        void init(void);
        void run(void);
        void close(void);
        
    private:
        unsigned int mem_size_neighbor_data;
        Grid *grid;
        float radius;
        
        // device memory pointers
        int* d_indices;
        unsigned int mem_size_neighbor_indices;
        int* d_agents;
        unsigned int mem_size_neighbor_agents;
    };
    
} // namespace

#endif // _FIND_NEIGHBORS_H_
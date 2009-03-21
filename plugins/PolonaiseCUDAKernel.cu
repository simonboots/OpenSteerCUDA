#ifndef _POLONAISE_CUDA_KERNEL_H_
#define _POLONAISE_CUDA_KERNEL_H_

#include "vehicle_t.h"

__global__ void
PolonaiseKernel(vehicle_t *data)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x);
    int numOfAgents = gridDim.x * blockDim.x;
    
    // find follower
    int follower = ((id - 1) % numOfAgents);
    
//    const Vec3 desiredVelocity = target - position();
//    return desiredVelocity - velocity();
    
    data[id].velocity.x = (data[follower].position.x - data[id].position.x) - data[id].velocity.x;
    data[id].velocity.y = (data[follower].position.y - data[id].position.y) - data[id].velocity.y;
    data[id].velocity.z = (data[follower].position.z - data[id].position.z) - data[id].velocity.z;
    
//    data[id + 3] = (data[follower + 0] - data[id + 0]) - data[id + 3];
//    data[id + 4] = (data[follower + 1] - data[id + 1]) - data[id + 4];
//    data[id + 5] = (data[follower + 2] - data[id + 2]) - data[id + 5];
}

#endif // _POLONAISE_CUDA_KERNEL_H_
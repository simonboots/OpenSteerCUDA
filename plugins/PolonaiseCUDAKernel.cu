#ifndef _POLONAISE_CUDA_KERNEL_H_
#define _POLONAISE_CUDA_KERNEL_H_

__global__ void
PolonaiseKernel(float *data)
{
    int id = (blockIdx.x * blockDim.x + threadIdx.x) * 6;
    int numOfAgents = gridDim.x * blockDim.x;
    
    // find follower
    int follower = ((id/6 - 1) % numOfAgents) * 6;
    
//    const Vec3 desiredVelocity = target - position();
//    return desiredVelocity - velocity();
    
    data[id + 3] = (data[follower + 0] - data[id + 0]) - data[id + 3];
    data[id + 4] = (data[follower + 1] - data[id + 1]) - data[id + 4];
    data[id + 5] = (data[follower + 2] - data[id + 2]) - data[id + 5];
}

#endif // _POLONAISE_CUDA_KERNEL_H_
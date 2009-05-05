#ifndef _SIMPLE_PURSUIT_MULTIPLIER_KERNEL_CU_
#define _SIMPLE_PURSUIT_MULTIPLIER_KERNEL_CU_

#include "CUDAKernelOptions.cu"

#define CHECK_BANK_CONFLICTS 0
#if CHECK_BANK_CONFLICTS
#define V_F(i) (CUT_BANK_CHECKER(((float*)velocity), i))
#define P_F(i) (CUT_BANK_CHECKER(((float*)position), i))
#define V(i) (CUT_BANK_CHECKER(velocity, i))
#define P(i) (CUT_BANK_CHECKER(position, i))
#else
#define V_F(i) ((float*)velocity)[i]
#define P_F(i) ((float*)position)[i]
#define V(i) velocity[i]
#define P(i) position[i]
#endif

__global__ void
simplePursuitMultiplierKernel(float3 new_position, float3 new_velocity, float3 *positions, float3 *velocities)
{
    int blockOffset = (blockDim.x * blockIdx.x * 3);
    
    // shared memory for position vector
    __shared__ float3 position[TPB];
    
    // shared memory for velocity vector
    __shared__ float3 velocity[TPB];
    
    P(threadIdx.x) = new_position;
    V(threadIdx.x) = new_velocity;
    
    __syncthreads();
    
    // copy position data back to global memory (coalesced)
    ((float*)positions)[blockOffset + threadIdx.x] = P_F(threadIdx.x);
    ((float*)positions)[blockOffset + threadIdx.x + blockDim.x] = P_F(threadIdx.x + blockDim.x);
    ((float*)positions)[blockOffset + threadIdx.x + 2*blockDim.x] = P_F(threadIdx.x + 2*blockDim.x);

    // copy velocity data back to global memory (coalesced)
    ((float*)velocities)[blockOffset + threadIdx.x] = V_F(threadIdx.x);
    ((float*)velocities)[blockOffset + threadIdx.x + blockDim.x] = V_F(threadIdx.x + blockDim.x);
    ((float*)velocities)[blockOffset + threadIdx.x + 2*blockDim.x] = V_F(threadIdx.x + 2*blockDim.x);
}

#endif // _SIMPLE_PURSUIT_MULTIPLIER_KERNEL_CU_
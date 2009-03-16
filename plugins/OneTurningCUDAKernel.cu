#ifndef _ONE_TURNING_CUDA_KERNEL_H_
#define _ONE_TURNING_CUDA_KERNEL_H_

__global__ void
OneTurningCUDAKernel(int* values)
{
    values[0] *= 2;
    values[1] *= 3;
}

#endif _ONE_TURNING_CUDA_KERNEL_H_

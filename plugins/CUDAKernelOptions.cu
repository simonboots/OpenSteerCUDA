#ifndef _CUDA_KERNEL_OPTIONS_H_
#define _CUDA_KERNEL_OPTIONS_H_

typedef enum kernel_opt
{
    NONE = 0,
    IGNORE_UNLESS_ZERO = 1
} kernel_options;

#endif // _CUDA_KERNEL_OPTIONS_H_
#include "AbstractCUDAKernel.h"
#include "CUDAPlugIn.h"

int OpenSteer::AbstractCUDAKernel::getNumberOfAgents(void) {
    if (cudaplugin != NULL) {
        return cudaplugin->getNumberOfAgents();
    }
    return 0;
}
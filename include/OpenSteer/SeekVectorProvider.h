#ifndef _SEEK_VECTOR_PROVIDER_H_
#define _SEEK_VECTOR_PROVIDER_H_

namespace OpenSteer {
    class SeekVectorProvider {
    public:
        virtual float3* getSeekVectors(void) {
            return d_seekVectors;
        }
    protected:
        float3* d_seekVectors;
    };
} // namespace


#endif // _SEEK_VECTOR_PROVIDER_H_
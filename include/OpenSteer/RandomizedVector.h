#ifndef _RANDOMIZED_VECTOR_H_
#define _RANDOMIZED_VECTOR_H_

namespace OpenSteer {
    
    class RandomizedVector {
    public:
        RandomizedVector(int size = 1024);
        ~RandomizedVector();
        float* getVector(void);
        void renew(void);
        int size(void);
    private:
        float* _randomizedVector;
        int _size;
    };
    
} // namespace OpenSteer

#endif // _RANDOMIZED_VECTOR_H_
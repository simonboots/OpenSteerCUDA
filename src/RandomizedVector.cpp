#include "RandomizedVector.h"
#include <stdlib.h>
#include <time.h>

OpenSteer::RandomizedVector::RandomizedVector(int size) {
    srand(time(NULL));
    _size = size;
    _randomizedVector = new float[size];
    renew();
}

OpenSteer::RandomizedVector::~RandomizedVector() {
    delete[] _randomizedVector;
}

float* OpenSteer::RandomizedVector::getVector(void) {
    return _randomizedVector;
}

int OpenSteer::RandomizedVector::size(void) {
    return _size;
}

void OpenSteer::RandomizedVector::renew(void) {
    for (int i = 0; i < _size; i++) {
        _randomizedVector[i] = ((float) rand ()) / ((float) RAND_MAX);
    }
}

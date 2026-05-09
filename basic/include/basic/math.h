#ifndef BASIC_MATH_H
#define BASIC_MATH_H

#include <cmath>

/// sign function, +1 for positive, -1 for negative, 0 for zero
inline int SIGN(float x) {return ((x > 0) - (x < 0));}  

inline float logit(float x) {
  return std::log(x / (1.0f - x));
}


#endif
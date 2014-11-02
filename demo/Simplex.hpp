/*
===============================================================================
A C++ port of a speed-improved simplex noise algorithm for 2D in Java.

Based on example code by Stefan Gustavson (stegu@itn.liu.se).
Optimisations by Peter Eastman (peastman@drizzle.stanford.edu).
Better rank ordering method by Stefan Gustavson in 2012.
C++ port and minor type and algorithm changes by Josh Koch (jdk1337@gmail.com).

This could be speeded up even further, but it's useful as it is.

Version 2012-04-12

The original Java code was placed in the public domain by its original author,
Stefan Gustavson. You may use it as you see fit,
but attribution is appreciated.
===============================================================================
*/

#ifndef SIMPLEX_H
#define SIMPLEX_H

#include <stdint.h>

float SimplexNoise2D(float xin, float yin);

//  noise with octaves
float Noise(float x, float zoom, int octaves, float persistence);
float Noise(float x, float y, float zoom, int octaves, float persistence);


//     xa  xb
//1    .___.
//0__./     \.___
//   xa-s    xb+s    // val, min, max, smooth range
inline static float linRange(const float& x, const float& xa, const float& xb, const float& s)
{
    if (x <= xa-s || x >= xb+s)  return 0.f;
    if (x >= xa && x <= xb)  return 1.f;
    if (x < xa)  return (x-xa)/s+1;
    if (x > xb)  return (xb-x)/s+1;
    return 0.f;
}

#endif

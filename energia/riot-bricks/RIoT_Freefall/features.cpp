// Port of the Max objects to extract motion features
#include <energia.h>
#include "features.h"

float magnitude3D(float x, float y, float z){
  return sqrt(x*x+y*y+z*z);
}

float delta(float previous, float next, float dt){
  return (next-previous)/(2*dt); // first-order derivative, centred finite difference of order 2 (3-point stencil)
}

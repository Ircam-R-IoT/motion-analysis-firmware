// Port of the Max objects to extract motion features
#include <energia.h>
#include "features.h"

float magnitude3D(float x, float y, float z){
  return sqrt(x*x+y*y+z*z);
}

float delta(float previous, float next, float dt){
  return (next-previous)/(2*dt); // first-order derivative, centred finite difference of order 2: 3-point stencil // change to backward?
}

float slide(float previous_y, float current_x, float slide_factor){
  return previous_y + (current_x-previous_y)/slide_factor;
}

int lcm(int a, int b){
  int a1 = a, b1 = b;
  while (a1 != b1){
    if (a1 < b1){a1+=a;} else {b1+=b;}
  }
  return a1;
}

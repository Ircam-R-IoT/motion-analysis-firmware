// Port of the Max objects to extract motion features
#include <energia.h>
#include "features.h"


float delta(float previous, float next, float dt){
  return (next-previous)/(2*dt); // first-order derivative, centred finite difference of order 2: 3-point stencil // change to backward?
}

float intensity1D(float xnext, float xprev, float intensityprev, float param1, float param2, float dt){
 float dx =  delta(xprev, xnext, dt);
 return param2*dx*dx+param1*intensityprev;
}

int lcm(int a, int b){
  int a1 = a, b1 = b;
  while (a1 != b1){
    if (a1 < b1){a1+=a;} else {b1+=b;}
  }
  return a1;
}


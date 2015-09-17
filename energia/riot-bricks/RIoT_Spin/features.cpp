// Port of the Max objects to extract motion features
#include <energia.h>
#include "features.h"

float magnitude3D(float x, float y, float z){
  return sqrt(x*x+y*y+z*z);
}


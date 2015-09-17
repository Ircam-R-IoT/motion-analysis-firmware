// Port of the Max objects to extract motion features
#include <energia.h>
#include "features.h"

float slide(float previous_y, float current_x, float slide_factor){
  return previous_y + (current_x-previous_y)/slide_factor;
}

float still_cross_product(float x, float y, float z){
  return (y-z)*(y-z)+(x-y)*(x-y)+(z-x)*(z-x);
}

#define FREEFALL_ACC_THRESHOLD 0.15f
#define FREEFALL_GYR_THRESHOLD 750.f
#define FREEFALL_GYR_DELTA_THRESHOLD 40.f

float magnitude3D(float x, float y, float z); 
float delta(float previous, float next, float dt);

#define ACC_INTENSITY_PARAM1 0.8f
#define ACC_INTENSITY_PARAM2 0.1f

#define FREEFALL_ACC_THRESHOLD 0.15f
#define FREEFALL_GYR_THRESHOLD 750.f
#define FREEFALL_GYR_DELTA_THRESHOLD 40.f

#define GYR_INTENSITY_PARAM1 0.9f
#define GYR_INTENSITY_PARAM2 1.0f

#define KICK_THRESHOLD 0.01f
#define KICK_SPEEDGATE 200   // in ms
#define KICK_MEDIAN_FILTERSIZE 9 //should be odd

#define SHAKE_THRESHOLD 0.1f
#define SHAKE_WINDOWSIZE 200
#define SHAKE_SLIDE_FACTOR 10

#define SPIN_THRESHOLD 200.0f

#define STILL_THRESHOLD 5000.0f
#define STILL_SLIDE_FACTOR 5.0f

float delta(float previous, float next, float dt);
float intensity1D(float xnext, float xprev, float intensityprev, float param1, float param2, float dt);
float magnitude3D(float x, float y, float z); 
int lcm(int a, int b); // least common multiple
float slide(float previous_slide, float current_value, float slide_factor);
float still_cross_product(float x, float y, float z);

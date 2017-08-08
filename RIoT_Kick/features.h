#define ACC_INTENSITY_PARAM1 0.8f
#define ACC_INTENSITY_PARAM2 0.1f
#define KICK_THRESHOLD 0.01f
#define KICK_SPEEDGATE 200   // in ms
#define KICK_MEDIAN_FILTERSIZE 9 //should be odd

float delta(float previous, float next, float dt);
float intensity1D(float xnext, float xprev, float intensityprev, float param1, float param2, float dt);
int lcm(int a, int b); // least common multiple

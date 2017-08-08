#define ACC_INTENSITY_PARAM1 0.8f
#define ACC_INTENSITY_PARAM2 0.1f

float delta(float previous, float next, float dt);
float intensity1D(float xnext, float xprev, float intensityprev, float param1, float param2, float dt);
int lcm(int a, int b); // least common multiple

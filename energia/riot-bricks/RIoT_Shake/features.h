#define SHAKE_THRESHOLD 0.1f
#define SHAKE_WINDOWSIZE 200
#define SHAKE_SLIDE_FACTOR 10

float magnitude3D(float x, float y, float z);
float delta(float previous, float next, float dt);
float slide(float previous_slide, float current_value, float slide_factor);
int lcm(int a, int b); // least common multiple

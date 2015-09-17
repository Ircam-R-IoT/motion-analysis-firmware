#define STILL_THRESHOLD 5000.0f
#define STILL_SLIDE_FACTOR 5.0f

float slide(float previous_slide, float current_value, float slide_factor);
float still_cross_product(float x, float y, float z);

#include "main.h"


#define PI 3.14159265358979323846f
#define PI_D2 1.57079632679489661923f
#define INVPI 0.31830988618379067154f
#define INVPISQ 0.10132118364233777155f

typedef struct 
{
  float x;
  float y;
  float z;
} Point;

typedef Point Vector;
float sine_m(int32_t angle);
float cosine_m(int32_t angle);
float atan_m(const float x);
float atan2_m(const float ys, const float xs) ;

float sin_fast(const float x);
float cos_fast(const float x);

float atan2_nvidia(const float y, const float x);
float asin_nvidia(const float x_) ;
float acos_nvidia(const float x_) ;
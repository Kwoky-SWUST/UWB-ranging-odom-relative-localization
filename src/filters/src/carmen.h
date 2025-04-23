#ifndef CARMEN_GLOBAL_H
#define CARMEN_GLOBAL_H

#include <vector>
#include <ros/ros.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846  /* pi */
#endif


/* Useful macros */

typedef struct {
  double x;
  double y;
  union {
    double theta;
    double theta_z;
  };
} carmen_point_t, *carmen_point_p;

double carmen_uniform_random(double min, double max);

double carmen_get_time(void);

double carmen_square(double val);


double carmen_normalize_theta(double theta);

double carmen_gaussian_random(double mean, double std);

#endif

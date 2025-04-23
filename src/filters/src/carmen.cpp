#include "carmen.h"

double carmen_uniform_random(double min, double max)
{
  return min + (rand() / (double)RAND_MAX) * (max - min);
}

double carmen_get_time(void)
{
  struct timeval tv;
  double t;

 // if (gettimeofday(&tv, NULL) < 0)
   // carmen_warn("carmen_get_time encountered error in gettimeofday : %s\n",
	//      strerror(errno));
  t = tv.tv_sec + tv.tv_usec/1000000.0;
  return t;
}



double carmen_square(double val)
{
	return (val*val);
}


double carmen_normalize_theta(double theta)
{
  int multiplier;

  if (theta >= -M_PI && theta < M_PI)
    return theta;

  multiplier = (int)(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}


double
carmen_gaussian_random(double mean, double std)
{
  const double norm = 1.0 / (RAND_MAX + 1.0);
  double u = 1.0 - rand() * norm;                  /* can't let u == 0 */
  double v = rand() * norm;
  double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
  return mean + std * z;
}

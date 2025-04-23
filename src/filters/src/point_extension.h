#ifndef POINT_EXTENSION
#define POINT_EXTENSION

typedef struct {
  double x;
  double y;
  double z;
  union {
    double theta;
    double theta_z;
  };
} carmen_3d_point_t, *carmen_3d_point_p;

// Added by PV/AK: carmen_6d_point_t, *carmen_6d_point_p
typedef struct {
  double x;
  double y;
  double z;
  double theta_x;
  double theta_y;
  double qx;
  double qy;
  double qz;
  double qw;
  union {
    double theta;
    double theta_z;
  };
} carmen_6d_point_t, *carmen_6d_point_p;

#endif // POINT_EXTENSION

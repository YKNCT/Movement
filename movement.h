#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
#ifndef M_PI_4
#define M_PI_4 (M_PI / 4)
#endif

#include <math.h>
#include "mbed.h"

#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

class Move {
 public:
  Move(int *p_duty, double const_yaw);
  Move(int *p_duty, double *p_yaw);
  Move(int *p_duty, int *p_angle, double *p_yaw);
  void CrossOmni_Move(int lx, int ly, int rx);
  void XmarkOmni_Move(int lx, int ly, int rx);
  void TriOmni_Move(int lx, int ly, int rx);
  void Mecanum_Move(int lx, int ly, int rx);
  void PosSwerve_Move(int lx, int ly, int rx);
  void SteerSwerve_Move(int lx, int ly, int rx);

 private:
  int *duty, *angle;
  double *yaw;
  double const_yaw;
  int duty_max;
};

#endif
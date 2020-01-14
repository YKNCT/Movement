#include "movement.h"

Move::Move(int *p_duty, double const_yaw) {
  duty = p_duty;
  *yaw = const_yaw;
}

Move::Move(int *p_duty, double *p_yaw) {
  duty = p_duty;
  yaw = p_yaw;
}

Move::Move(int *p_duty, int *p_angle, double *p_yaw) {
  duty = p_duty;
  angle = p_angle;
  yaw = p_yaw;
}

void Move::TetraOmni_Move(int lx, int ly, int rx) {
  duty[0] = lx * cos(DEG_TO_RAD(*yaw) + M_PI / 4) +
            ly * sin(DEG_TO_RAD(*yaw) + M_PI / 4);
  duty[1] = -lx * sin(DEG_TO_RAD(*yaw) + M_PI / 4) +
            ly * cos(DEG_TO_RAD(*yaw) + M_PI / 4);
  duty[2] = -lx * cos(DEG_TO_RAD(*yaw) + M_PI / 4) -
            ly * sin(DEG_TO_RAD(*yaw) + M_PI / 4);
  duty[3] = lx * sin(DEG_TO_RAD(*yaw) + M_PI / 4) -
            ly * cos(DEG_TO_RAD(*yaw) + M_PI / 4);

  for (int i = 0; i < 4; i++) {
    duty[i] += rx;

    if (i == 0)
      duty_max = abs(duty[0]);
    else if (duty_max < abs(duty[i]))
      duty_max = abs(duty[i]);
  }
  for (int i = 0; i < 4; i++) {
    if (duty_max > 99) duty[i] = duty[i] * 99 / duty_max;
  }
}

void Move::TriOmni_Move(int lx, int ly, int rx) {
  duty[0] = lx * cos(DEG_TO_RAD(*yaw)) + ly * sin(DEG_TO_RAD(*yaw));
  duty[1] = -(lx + cos(DEG_TO_RAD(*yaw) + ly * sin(DEG_TO_RAD(*yaw)))) / 2.0 +
            (-lx * sin(DEG_TO_RAD(*yaw) + ly * cos(DEG_TO_RAD(*yaw)))) *
                sqrt(3.0) / 2.0;
  duty[2] = -(lx + cos(DEG_TO_RAD(*yaw) + ly * sin(DEG_TO_RAD(*yaw)))) / 2.0 -
            (-lx * sin(DEG_TO_RAD(*yaw) + ly * cos(DEG_TO_RAD(*yaw)))) *
                sqrt(3.0) / 2.0;

  for (int i = 0; i < 3; i++) {
    duty[i] += rx;

    if (i == 0)
      duty_max = abs(duty[0]);
    else if (duty_max < abs(duty[i]))
      duty_max = abs(duty[i]);
  }
  for (int i = 0; i < 3; i++) {
    if (duty_max > 99) duty[i] = duty[i] * 99 / duty_max;
  }
}

void Move::Mecanum_Move(int lx, int ly, int rx) {
  duty[0] = lx * cos(DEG_TO_RAD(*yaw)) + ly * sin(DEG_TO_RAD(*yaw)) -
            lx * sin(DEG_TO_RAD(*yaw)) + ly * cos(DEG_TO_RAD(*yaw));
  duty[1] = -lx * cos(DEG_TO_RAD(*yaw)) - ly * sin(DEG_TO_RAD(*yaw)) -
            lx * sin(DEG_TO_RAD(*yaw)) + ly * cos(DEG_TO_RAD(*yaw));
  duty[2] = -lx * cos(DEG_TO_RAD(*yaw)) - ly * sin(DEG_TO_RAD(*yaw)) +
            lx * sin(DEG_TO_RAD(*yaw)) - ly * cos(DEG_TO_RAD(*yaw));
  duty[3] = lx * cos(DEG_TO_RAD(*yaw)) + ly * sin(DEG_TO_RAD(*yaw)) +
            lx * sin(DEG_TO_RAD(*yaw)) - ly * cos(DEG_TO_RAD(*yaw));

  for (int i = 0; i < 4; i++) {
    duty[i] += rx;

    if (i == 0)
      duty_max = abs(duty[0]);
    else if (duty_max < abs(duty[i]))
      duty_max = abs(duty[i]);
  }
  for (int i = 0; i < 4; i++) {
    if (duty_max > 99) duty[i] = duty[i] * 99 / duty_max;
  }
}

void Move::PosSwerve_Move(int lx, int ly, int rx) {
  duty[0] = sqrt(pow(lx - rx * cos(DEG_TO_RAD(*yaw) + M_PI_4), 2) +
                 pow(ly - rx * sin(DEG_TO_RAD(*yaw) + M_PI_4), 2));
  duty[1] = sqrt(pow(lx + rx * sin(DEG_TO_RAD(*yaw) + M_PI_4), 2) +
                 pow(ly - rx * cos(DEG_TO_RAD(*yaw) + M_PI_4), 2));
  duty[2] = sqrt(pow(lx + rx * cos(DEG_TO_RAD(*yaw) + M_PI_4), 2) +
                 pow(ly + rx * sin(DEG_TO_RAD(*yaw) + M_PI_4), 2));
  duty[3] = sqrt(pow(lx - rx * sin(DEG_TO_RAD(*yaw) + M_PI_4), 2) +
                 pow(ly + rx * cos(DEG_TO_RAD(*yaw) + M_PI_4), 2));

  for (int i = 0; i < 4; i++) {
    if (i == 0)
      duty_max = abs(duty[0]);
    else if (duty_max < abs(duty[i]))
      duty_max = abs(duty[i]);
  }
  for (int i = 0; i < 4; i++) {
    if (duty_max > 99) duty[i] = duty[i] * 99 / duty_max;
  }

  if (lx == 0 && ly == 0) {
    angle[0] = -135;
    angle[1] = -45;
    angle[2] = 45;
    angle[3] = 135;
  } else {
    angle[0] = RAD_TO_DEG(atan2(ly - rx * cos(DEG_TO_RAD(*yaw) + M_PI_4),
                                lx - rx * sin(DEG_TO_RAD(*yaw) + M_PI_4)) -
                          *yaw);
    angle[1] = RAD_TO_DEG(atan2(ly - rx * sin(DEG_TO_RAD(*yaw) + M_PI_4),
                                lx + rx * cos(DEG_TO_RAD(*yaw) + M_PI_4)) -
                          *yaw);
    angle[2] = RAD_TO_DEG(atan2(ly + rx * cos(DEG_TO_RAD(*yaw) + M_PI_4),
                                lx + rx * sin(DEG_TO_RAD(*yaw) + M_PI_4)) -
                          *yaw);
    angle[3] = RAD_TO_DEG(atan2(ly + rx * sin(DEG_TO_RAD(*yaw) + M_PI_4),
                                lx - rx * cos(DEG_TO_RAD(*yaw) + M_PI_4)) -
                          *yaw);
  }
}
void Move::SteerSwerve_Move(int lx, int ly, int rx) {
  //  プルして―
}
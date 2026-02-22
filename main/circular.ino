double wallIntegral = 0;
long wallPrevMicros = 0;
double wallPrevError = 0;

double circle_r1;
double circle_r2;

double circle_ratio;

double circle_ticksps1;
double circle_ticksps2;

long circle_lprev_enc;
long circle_rprev_enc;

double circle_lprev_error;
double circle_rprev_error;

double circle_l_integral;
double circle_r_integral;

double circle_kp;
double circle_ki;
double circle_kd;

double circle_integral_max;
double circle_integral_min;

long circle_prev_micros = micros();



void moveAlongCircle(double ticks_ps1, double rad, double kp, double ki, double kd,
                     double integral_min, double integral_max) {

  task3 = true;

  resetEncoders();

  circle_ticksps1 = ticks_ps1;
  double r = rad;

  circle_r1 = r - (physical.wheelDist / 2.0);
  circle_r2 = r + (physical.wheelDist / 2.0);

  circle_ratio = (circle_r1 == 0) ? 1 : (circle_r2 / circle_r1);

  circle_ticksps2 = circle_ticksps1 * circle_ratio;

  circle_lprev_enc = 0;
  circle_rprev_enc = 0;

  circle_lprev_error = 0;
  circle_rprev_error = 0;

  circle_l_integral = 0;
  circle_r_integral = 0;

  circle_kp = kp;
  circle_ki = ki;
  circle_kd = kd;

  circle_integral_min = integral_min;
  circle_integral_max = integral_max;

  circle_prev_micros = micros();

  wallIntegral = 0;
}

void moveAlongCircleIter() {
  long now = micros();
  double dt = (double)(now - circle_prev_micros) / 1e6;  // seconds
  if (dt <= 0) dt = 0.001;

  noInterrupts();
  int l_enc = leftMotor.count;
  int r_enc = rightMotor.count;
  interrupts();

  int dl = l_enc - circle_lprev_enc;
  int dr = r_enc - circle_rprev_enc;

  double left_speed = dl / dt;
  double right_speed = dr / dt;

  double errorL = circle_ticksps1 - left_speed;
  double errorR = circle_ticksps1 - right_speed;

  circle_l_integral += errorL * dt * circle_ki;
  circle_r_integral += errorR * dt * circle_ki;

  circle_l_integral = constrain(circle_l_integral, circle_integral_min, circle_integral_max);
  circle_r_integral = constrain(circle_r_integral, circle_integral_min, circle_integral_max);

  double lu = circle_kp * errorL + circle_l_integral + circle_kd * (errorL - circle_lprev_error) / dt;
  double ru = circle_kp * errorR + circle_r_integral + circle_kd * (errorR - circle_rprev_error) / dt;

  setPWM(leftMotor, lu);
  setPWM(rightMotor, ru);

  circle_lprev_enc = l_enc;
  circle_rprev_enc = r_enc;
  circle_lprev_error = errorL;
  circle_rprev_error = errorR;
  circle_prev_micros = now;

  delay(10);
}

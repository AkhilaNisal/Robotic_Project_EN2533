
double wheelSyncIter(
  double target_diff,
  double kp, double ki, double kd,
  double integral_min, double integral_max) {
  unsigned long now = micros();
  double dt = (double)((now - sync_prev_micros) / 1e6);

  if (sync_prev_micros == 0) {
    sync_prev_micros = now;
    sync_prev_err = 0;
    sync_integral = 0;
    return 0;
  }

  double err = (leftMotor.count - rightMotor.count) - target_diff;

  sync_integral += 0.5 * (err + sync_prev_err) * dt * ki;
  sync_integral = constrain(sync_integral, integral_min, integral_max);

  double derivative = (err - sync_prev_err) / dt;

  double output = kp * err + sync_integral + kd * derivative;

  sync_prev_err = err;
  sync_prev_micros = now;

  return output;
}

double basePWM_prev_err = 0;
unsigned long basePWM_prev_micros = 0;
double basePWM_integral = 0;

double basePWMIter(
  long target_count,
  double kp, double ki, double kd,
  double integral_min, double integral_max) {

  unsigned long now = micros();
  double dt = (double)((now - basePWM_prev_micros) / 1e6);

  if (basePWM_prev_micros == 0) {
    basePWM_prev_micros = now;
    basePWM_prev_err = 0;
    basePWM_integral = 0;
    return 255;
  }

  long current = pos.count;
  double err = (double)(target_count - current);

  basePWM_integral += 0.5 * (err + basePWM_prev_err) * dt * ki;
  basePWM_integral = constrain(basePWM_integral, integral_min, integral_max);

  double derivative = (err - basePWM_prev_err) / dt;
  double output = kp * err + basePWM_integral + kd * derivative;

  basePWM_prev_err = err;
  basePWM_prev_micros = now;

  return output;
}

void moveForwardDist(double dist) {
  noInterrupts();
  pos.count = 0;
  leftMotor.count = 0;
  rightMotor.count = 0;
  interrupts();

  int zero_crossings = 0;
  int ticks = distToCounts(dist);

  Serial.println(dist);
  Serial.println(ticks);
  basePWM = 40;
  if (ticks < 0) {
    basePWM = -50;
    //basePWM = -40
  }

  while (true) {

    if ((ticks > 0 && ticks <= pos.count) || (ticks < 0 && ticks >= pos.count)) {
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);

      delay(100);
      break;
    }

    double sync_correction = wheelSyncIter(
      0,             // target diff: 0 → wheels match
      1.5, 0, 0.05,  // kp, ki, kd
      -100, 100      // integral clamp
    );

    double left_pwm = basePWM - sync_correction;
    double right_pwm = basePWM + sync_correction;

    // setPWM(leftMotor, 0);
    // setPWM(rightMotor, 0);
    setPWM(leftMotor, left_pwm);
    setPWM(rightMotor, right_pwm);

    delay(10);
  }
}

void resetEncoders() {
  noInterrupts();
  pos.target_count = 0;
  pos.count = 0;
  leftMotor.count = 0;
  rightMotor.count = 0;
  interrupts();
}
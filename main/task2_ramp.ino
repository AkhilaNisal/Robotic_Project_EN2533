// void ramp(double ticks_ps, double kp, double ki, double kd,
//           double integral_min, double integral_max) {

//   resetEncoders();

//   double basePWM = 60;

//   int lprev_enc = 0;
//   int rprev_enc = 0;

//   double lprev_error = 0;
//   double rprev_error = 0;

//   double l_integral = 0;
//   double r_integral = 0;

//   long prev_micros = micros();

//   while (true) {

//     long now = micros();
//     double dt = (double)(now - prev_micros) / 1e6;  // seconds
//     if (dt <= 0) dt = 0.001;

//     noInterrupts();
//     int l_enc = leftMotor.count;
//     int r_enc = rightMotor.count;
//     interrupts();

//     int dl = l_enc - lprev_enc;
//     int dr = r_enc - rprev_enc;

//     double left_speed = dl / dt;
//     double right_speed = dr / dt;

//     double errorL = ticks_ps - left_speed;
//     double errorR = ticks_ps - right_speed;

//     l_integral += errorL * dt * ki;
//     r_integral += errorR * dt * ki;

//     l_integral = constrain(l_integral, integral_min, integral_max);
//     r_integral = constrain(r_integral, integral_min, integral_max);

//     double lu = kp * errorL + l_integral + kd * (errorL - lprev_error) / dt;
//     double ru = kp * errorR + r_integral + kd * (errorR - rprev_error) / dt;

//     double sync = wheelSyncIter(
//       0,        // target diff
//       0, 0, 0,  // KP, KI, KD
//       -100, 100);

//     double left_pwm = lu + sync;
//     double right_pwm = ru - sync;

//     if (anySensorWhite()) {
//       moveForwardDist(50);
//       break;
//     }

//     setPWM(leftMotor, left_pwm);
//     setPWM(rightMotor, right_pwm);


//     // Serial.print(lu);
//     // Serial.print(" ");
//     // Serial.print(ru);
//     // Serial.print(" ");
//     // Serial.print(sync);
//     // Serial.print(" ");
//     // Serial.print(left_pwm);
//     // Serial.print(" ");
//     // Serial.println(right_pwm);

//     lprev_enc = l_enc;
//     rprev_enc = r_enc;
//     lprev_error = errorL;
//     rprev_error = errorR;
//     prev_micros = now;

//     delay(10);
//   }
// }


void ramp(double ticks_ps, double kp, double ki, double kd,
          double integral_min, double integral_max) {

  resetEncoders();

  double basePWM = 60;

  int lprev_enc = 0;
  int rprev_enc = 0;

  double lprev_error = 0;
  double rprev_error = 0;

  double l_integral = 0;
  double r_integral = 0;

  long prev_micros = micros();

  while (true) {

    long now = micros();
    double dt = (double)(now - prev_micros) / 1e6;  // seconds
    if (dt <= 0) dt = 0.001;

    noInterrupts();
    int l_enc = leftMotor.count;
    int r_enc = rightMotor.count;
    interrupts();

    int dl = l_enc;
    int dr = r_enc;

    double left_speed = dl / dt;
    double right_speed = dr / dt;

    double sync = wheelSyncIter(
      0,           // target diff
      22, 0, 0.0,  // KP, KI, KD
      -100, 100);

    double errorL = ticks_ps - sync - left_speed;
    double errorR = ticks_ps + sync - right_speed;

    Serial.println(sync);

    l_integral += errorL * dt * ki;
    r_integral += errorR * dt * ki;

    l_integral = constrain(l_integral, integral_min, integral_max);
    r_integral = constrain(r_integral, integral_min, integral_max);

    double lu = kp * errorL + l_integral + kd * (errorL - lprev_error) / dt;
    double ru = kp * errorR + r_integral + kd * (errorR - rprev_error) / dt;


    double left_pwm = lu;
    double right_pwm = ru;

    if (anySensorWhite()) {
      moveForwardDist(50);
      break;
    }

    setPWM(leftMotor, left_pwm);
    setPWM(rightMotor, right_pwm);



    // Serial.print(lu);
    // Serial.print(" ");
    // Serial.print(ru);
    // Serial.print(" ");
    // Serial.print(sync);
    // Serial.print(" ");
    // Serial.print(left_pwm);
    // Serial.print(" ");
    // Serial.println(right_pwm);

    resetEncoders();
    lprev_error = errorL;
    rprev_error = errorR;
    prev_micros = now;

    sync_prev_micros = now;
    sync_prev_err = 0;
    sync_integral = 0;

    delay(10);
  }
}

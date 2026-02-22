void wallFollow() {

  double target = 100;

  // double kp_dist = 0.3, ki_dist = 0.2, kd_dist = 0.05;
  // double kp_angle = 0.6, ki_angle = 0.2, kd_angle = 0.05;

  double kp_dist = 0.3, ki_dist = 0.2, kd_dist = 0.05;
  double kp_angle = 0.6, ki_angle = 0.2, kd_angle = 0.05;

  basePWM = 60;

  double dist_integral = 0;
  double dist_prev_error = 0;

  double angle_integral = 0;
  double angle_prev_error = 0;

  long prevMicros = 0;

  while (true) {

    long now = micros();
    double dt = (now - prevMicros) / 1e6;
    if (dt <= 0) dt = 0.001;
    prevMicros = now;

    int LFd = FL.readRange();
    int LBd = BL.readRange();

    if (LFd > 1000 || LBd > 1000) return;  // invalid readings → skip
    double distance_error = ((double)(LFd + LBd) / 2) - target;
    double angle_error = LFd - LBd;

    Serial.print(distance_error);
    Serial.print(" ");
    Serial.println(angle_error);

    dist_integral += distance_error * dt;
    double dist_derivative = (distance_error - dist_prev_error) / dt;
    dist_prev_error = distance_error;

    double dist_output =
      kp_dist * distance_error + ki_dist * dist_integral + kd_dist * dist_derivative;

    angle_integral += angle_error * dt;
    double angle_derivative = (angle_error - angle_prev_error) / dt;
    angle_prev_error = angle_error;

    double angle_output =
      kp_angle * angle_error + ki_angle * angle_integral + kd_angle * angle_derivative;

    double correction = dist_output + angle_output;

    int Lmotor = basePWM - correction;
    int Rmotor = basePWM + correction;

    // int Lmotor = 50 - correction;
    // int Rmotor = 90 + correction;


    setPWM(leftMotor, Lmotor);
    setPWM(rightMotor, Rmotor);

    // Serial.print("FL:");
    // Serial.print(FLd);
    // Serial.print(" BL:");
    // Serial.print(BLd);
    // Serial.print(" Corr:");
    // Serial.println(correction);
  }
}
double Invsync_prev_err = 0;
unsigned long Invsync_prev_micros = 0;
double Invsync_integral = 0;

double wheelInvertedSyncIter(
  double target_diff,
  double kp, double ki, double kd,
  double integral_min, double integral_max) {
  unsigned long now = micros();
  double dt = (now - Invsync_prev_micros) / 1e6;


  if (Invsync_prev_micros == 0 || dt <= 0) {
    Invsync_prev_micros = now;
    Invsync_prev_err = 0;
    Invsync_integral = 0;
    return 0;
  }

  double err =
    ((leftMotor.count) + rightMotor.count) - target_diff;

  Invsync_integral += 0.5 * (err + Invsync_prev_err) * dt * ki;
  Invsync_integral = constrain(Invsync_integral, integral_min, integral_max);

  double derivative = (err - Invsync_prev_err) / dt;

  double output = kp * err + Invsync_integral + kd * derivative;

  Invsync_prev_err = err;
  Invsync_prev_micros = now;

  return output;
}


double L_prev_err = 0, R_prev_err = 0;
double L_integral = 0, R_integral = 0;
unsigned long prev_micros = 0;

double wheelturnIter(double target, double current,
                     double kp, double ki, double kd,
                     double &prev_err, double &integral) {
  unsigned long now = micros();
  double dt = (now - prev_micros) / 1e6;

  if (prev_micros == 0 || dt <= 0) {
    prev_micros = now;
    prev_err = 0;
    integral = 0;
    return 0;
  }

  double err = target - current;

  integral += err * dt;

  double derivative = (err - prev_err) / dt;

  double output = kp * err + ki * integral + kd * derivative;

  prev_err = err;
  prev_micros = now;

  return output;
}

void turnLeft() {

  double arc_mm = physical.wheelDist * PI / 4;
  double target_counts = distToCounts(arc_mm);

  Serial.print("Left-turn target counts: ");
  Serial.println(target_counts);

  noInterrupts();
  leftMotor.count = 0;
  rightMotor.count = 0;
  interrupts();

  int zero_crossings = 0;

  int pwmL = 0;
  int pwmR = 0;

  prev_micros = 0;
  L_prev_err = R_prev_err = 0;
  L_integral = R_integral = 0;

  while (true) {

    double left = abs(leftMotor.count);
    double right = abs(rightMotor.count);

    if (left >= target_counts || right >= target_counts) {
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      delay(150);
      break;
    }

    double Inverted_sync_correction = wheelInvertedSyncIter(
      0,              // target diff: 0 → wheels match
      1.5, 0.2, 0.8,  // kp, ki, kd
      -100, 100       // integral clamp
    );


    // double L_out = wheelturnIter(target_counts, left,
    //                              1.5, 0.01, 0.7,  // KP, KI, KD (example)
    //                              L_prev_err, L_integral);

    // double R_out = wheelturnIter(target_counts, right,
    //                              1.5, 0.01, 0.7,
    //                              R_prev_err, R_integral);

    pwmL = -60 - Inverted_sync_correction;
    pwmR = 60 - Inverted_sync_correction;

    if (pwmL < 0) {
      pwmL = constrain(pwmL, -80, -40);  // negative = backward
    } else {
      pwmL = constrain(pwmL, 40, 80);  // negative = backward
    }
    if (pwmR < 0) {
      pwmR = constrain(pwmR, -80, -40);  // positive = forward
    } else {
      pwmR = constrain(pwmR, 40, 80);  // positive = forward
    }
    //  setPWM(leftMotor, 0);
    //     setPWM(rightMotor, 0);
    setPWM(leftMotor, pwmL);
    setPWM(rightMotor, pwmR);
    // Serial.print(left);
    // Serial.print(" ");
    // Serial.print(pwmL);
    // Serial.print(" ");
    // Serial.print(pwmR);

    // Serial.print(" ");
    // Serial.println(Inverted_sync_correction);
  }
}


void turnRight() {

  double arc_mm = physical.wheelDist * PI / 4;
  double target_counts = distToCounts(arc_mm);

  Serial.print("Right-turn target counts: ");
  Serial.println(target_counts);

  noInterrupts();
  leftMotor.count = 0;
  rightMotor.count = 0;
  interrupts();

  int zero_crossings = 0;

  int pwmL = 0;
  int pwmR = 0;

  prev_micros = 0;
  L_prev_err = R_prev_err = 0;
  L_integral = R_integral = 0;

  while (true) {

    double left = abs(leftMotor.count);
    double right = abs(rightMotor.count);

    if (left >= target_counts || right >= target_counts) {


      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      break;
    }


    double Inverted_sync_correction = wheelInvertedSyncIter(
      0,              // target diff: 0 → wheels match
      1.5, 0.2, 0.8,  // kp, ki, kd
      -100, 100       // integral clamp
    );


    // double L_out = wheelturnIter(target_counts, left,
    //                              1.5, 0.01, 0.7,  // KP, KI, KD (example)
    //                              L_prev_err, L_integral);

    // double R_out = wheelturnIter(target_counts, right,
    //                              1.5, 0.01, 0.7,
    //                              R_prev_err, R_integral);

    pwmL = 60 - Inverted_sync_correction;
    pwmR = -60 - Inverted_sync_correction;

    // if (pwmL == 0 || pwmR == 0) {
    //   zero_crossings++;
    //   if (zero_crossings > 10) {
    //     break;
    //   }
    // }

    if (pwmL < 0) {
      pwmL = constrain(pwmL, -80, -40);  // negative = backward
    } else {
      pwmL = constrain(pwmL, 40, 80);  // negative = backward
    }
    if (pwmR < 0) {
      pwmR = constrain(pwmR, -80, -40);  // positive = forward
    } else {
      pwmR = constrain(pwmR, 40, 80);  // positive = forward
    }
    // setPWM(leftMotor, 0);
    // setPWM(rightMotor, 0);
    setPWM(leftMotor, pwmL);
    setPWM(rightMotor, pwmR);
    // Serial.print(left);
    // Serial.print(" ");
    // Serial.print(pwmL);
    // Serial.print(" ");
    // Serial.print(pwmR);

    // Serial.print(" ");
    // Serial.println(Inverted_sync_correction);
  }
}


void turnBack() {

  int turn_offset = 37;

  double arc_mm = physical.wheelDist * PI / 2;
  double target_counts = distToCounts(arc_mm);

  Serial.print("Right-turn target counts: ");
  Serial.println(target_counts);

  noInterrupts();
  leftMotor.count = 0;
  rightMotor.count = 0;
  interrupts();

  int zero_crossings = 0;

  int pwmL = 0;
  int pwmR = 0;

  prev_micros = 0;
  L_prev_err = R_prev_err = 0;
  L_integral = R_integral = 0;

  while (true) {

    double left = abs(leftMotor.count);
    double right = abs(rightMotor.count);

    if (left >= target_counts + turn_offset || right >= target_counts + turn_offset) {


      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      break;
    }


    double Inverted_sync_correction = wheelInvertedSyncIter(
      0,              // target diff: 0 → wheels match
      1.5, 0.2, 0.8,  // kp, ki, kd
      -100, 100       // integral clamp
    );


    // double L_out = wheelturnIter(target_counts, left,
    //                              1.5, 0.01, 0.7,  // KP, KI, KD (example)
    //                              L_prev_err, L_integral);

    // double R_out = wheelturnIter(target_counts, right,
    //                              1.5, 0.01, 0.7,
    //                              R_prev_err, R_integral);

    pwmL = 60 - Inverted_sync_correction;
    pwmR = -60 - Inverted_sync_correction;

    // if (pwmL == 0 || pwmR == 0) {
    //   zero_crossings++;
    //   if (zero_crossings > 10) {
    //     break;
    //   }
    // }

    if (pwmL < 0) {
      pwmL = constrain(pwmL, -80, -40);  // negative = backward
    } else {
      pwmL = constrain(pwmL, 40, 80);  // negative = backward
    }
    if (pwmR < 0) {
      pwmR = constrain(pwmR, -80, -40);  // positive = forward
    } else {
      pwmR = constrain(pwmR, 40, 80);  // positive = forward
    }
    // setPWM(leftMotor, 0);
    // setPWM(rightMotor, 0);
    setPWM(leftMotor, pwmL);
    setPWM(rightMotor, pwmR);
    // Serial.print(left);
    // Serial.print(" ");
    // Serial.print(pwmL);
    // Serial.print(" ");
    // Serial.print(pwmR);

    // Serial.print(" ");
    // Serial.println(Inverted_sync_correction);
  }
}

void arrow_find(bool dir) {  // 0: left, 1: right
  double arc_mm = physical.wheelDist * PI;
  double target_counts = distToCounts(arc_mm);

  Serial.print("Rotational target counts: ");
  Serial.println(target_counts);

  noInterrupts();
  leftMotor.count = 0;
  rightMotor.count = 0;
  interrupts();

  int zero_crossings = 0;

  int pwmL = 0;
  int pwmR = 0;

  prev_micros = 0;
  L_prev_err = R_prev_err = 0;
  L_integral = R_integral = 0;
  uint16_t threshold = 330;

  Serial.println("arrow finding...");

  while (true) {

    double left = abs(leftMotor.count);
    double right = abs(rightMotor.count);

    int sensor_values[8];
    qtr.read(sensor_values);

    if (sensor_values[4] < threshold) {
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      Serial.println("arrow found");
      L_prev_err = R_prev_err = 0;
      L_integral = R_integral = 0;
      break;
    }

    // if (left == target_counts || right == target_counts) {
    //   zero_crossings++;
    //   if (zero_crossings > 5) {
    //     brakeLOW(leftMotor);
    //     brakeLOW(rightMotor);
    //     Serial.println("arrow not found");
    //     L_prev_err = R_prev_err = 0;
    //     L_integral = R_integral = 0;
    //     break;
    //   }
    // }


    if (left >= target_counts || right >= target_counts) {
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      Serial.println("arrow not found");
      L_prev_err = R_prev_err = 0;
      L_integral = R_integral = 0;
      break;
    }

    double Inverted_sync_correction = wheelInvertedSyncIter(
      0,         // target diff: 0 → wheels match
      1, 0, 0,   // kp, ki, kd
      -100, 100  // integral clamp
    );


    // double L_out = wheelturnIter(target_counts, left,
    //                              1.5, 0.01, 0.7,  // KP, KI, KD (example)
    //                              L_prev_err, L_integral);

    // double R_out = wheelturnIter(target_counts, right,
    //                              1.5, 0.01, 0.7,
    //                              R_prev_err, R_integral);

    double L_out = 60;
    double R_out = 60;

    if (dir == false) {
      pwmL = -L_out - Inverted_sync_correction;
      pwmR = R_out - Inverted_sync_correction;
    } else {
      pwmL = L_out - Inverted_sync_correction;
      pwmR = -R_out - Inverted_sync_correction;
    }



    if (pwmL < 0) {
      pwmL = constrain(pwmL, -80, -40);  // negative = backward
    } else {
      pwmL = constrain(pwmL, 40, 80);  // negative = backward
    }
    if (pwmR < 0) {
      pwmR = constrain(pwmR, -80, -40);  // positive = forward
    } else {
      pwmR = constrain(pwmR, 40, 80);  // positive = forward
    }
    //  setPWM(leftMotor, 0);
    //     setPWM(rightMotor, 0);
    setPWM(leftMotor, pwmL);
    setPWM(rightMotor, pwmR);
  }
}
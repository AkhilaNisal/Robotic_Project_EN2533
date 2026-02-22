bool allSensorsBlack(uint16_t threshold = 500);
bool allSensorsWhite(uint16_t threshold = 500);
bool anySensorWhite(uint16_t threshold = 500);

bool arrow_found_dir;

void arrow_shoot(double dist, double threshold) {
  noInterrupts();
  pos.count = 0;
  leftMotor.count = 0;
  rightMotor.count = 0;
  interrupts();

  int last_error = 0;
  int zero_crossings = 0;
  int ticks = distToCounts(dist);
  bool flag = false;
  double linePIDThreshold = 350;
  // Serial.println(dist);
  // Serial.println(ticks);

  Serial.println("arrow shooting");
  int count = 0;
  basePWM = 100;
  if (ticks < 0) {
    basePWM = -60;
  }
  while (true) {

    if (allSensorsWhite()) {
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      Serial.println("arrow task done");
      delay(500);
      break;
    }

    int sensor_values[8];
    qtr.read(sensor_values);
    long weighted_sum = 0;
    int active_sensors = 0;
    int sensor_weights[8] = { -16, -9, -4, -1, 1, 4, 9, 16 };
    int leftIR = (analogRead(A13) < linePIDThreshold) ? 1 : 0;
    int rightIR = (analogRead(A12) < linePIDThreshold) ? 1 : 0;
    for (uint16_t i = 0; i < 8; i++) {
      int value = (sensor_values[i] < threshold) ? 1 : 0;
      // Serial.println(value);
      weighted_sum += value * sensor_weights[i];
      active_sensors += value;
    }
    double error = (active_sensors != 0) ? (double)weighted_sum / active_sensors : 0;
    error = error + leftIR * -25 + rightIR * 25;
    Serial.println(error);
    if (abs(error) > 0) {
      flag = true;
    }

    if (flag) {
      if (!allSensorsBlack()) {
        last_error = error;
      }
    }

    if ((ticks > 0 && ticks <= pos.count) || (ticks < 0 && ticks >= pos.count) || (flag)) {
      if (flag && count <= 5) {
        count++;
        continue;
      }
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      Serial.println(last_error);
      if (last_error > 0) {
        arrow_found_dir = true;  //false
      } else {
        arrow_found_dir = false;
      }
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



void arrow_following() {
  basePWM = 60;
  resetEncoders();
  int cross_count = 0;
  bool arrow_align = false;
  Serial.println("arrow following...");

  while (true) {

    if (allSensorsWhite()) {
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      Serial.println("arrow task done");
      delay(500);
      break;
    }

    if (allSensorsBlack() && arrow_align == false) {
      Serial.println(
        "All black");
      arrow_align = true;
    }

    if (arrow_align) {
      Serial.println(cross_count);
      if (cross_count >= 1) {
        arrow_align = false;
        cross_count = 0;
        // moveForwardDist(200);

        brakeHIGH(leftMotor);
        brakeHIGH(rightMotor);
        Serial.println("exit arrow following");
        line_prev_error = 0;
        line_integral = 0;
        break;
      }
      cross_count++;
      moveForwardDist(-100);
      delay(100);
      arrow_align = false;
      basePWM = 90;
      // delay(20);
    }
    double line_correction = lineFollowIter(
      5, 0.8, 0.3,  //4, 0.1, 0.1,
      -100, 100);
    // Serial.println(line_correction);
    // double left_pwm = basePWM - (basePWM > 0 ? line_correction : -line_correction);
    // double right_pwm = basePWM + (basePWM > 0 ? line_correction : -line_correction);
    double left_pwm = basePWM + line_correction;
    double right_pwm = basePWM - line_correction;

    setPWM(leftMotor, left_pwm);
    setPWM(rightMotor, right_pwm);
    delay(10);
  }
}

void arrow_task() {
  while (true) {

    arrow_following();
    // moveForwardDist(200);
    arrow_shoot(250, 500);
    if (allSensorsWhite()) {
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      Serial.println("arrow task done");
      delay(500);
      break;
    }
    // moveForwardDist(120);
    Serial.print("dir: ");
    Serial.println(arrow_found_dir);
    arrow_find(arrow_found_dir);
  }
}
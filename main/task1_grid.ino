double alignment_output = 0;
int cross_count = 1;

bool isLineCrossedFully() {
  int sensor_values[8];
  qtr.read(sensor_values);
  noInterrupts();
  now_loc = pos.count;
  interrupts();

  bool left_now = (sensor_values[0] < 500);
  bool right_now = (sensor_values[6] < 500);

  if (left_now != left_prev && lt == -1) {
    lt = now_loc;
    Serial.print("lt: ");
    Serial.println(lt);
  }
  left_prev = left_now;

  if (right_now != right_prev && rt == -1) {
    rt = now_loc;
  }
  right_prev = right_now;

  if (lt != -1 && rt != -1) {

    Serial.println("true una httooooooooooooooo");
    return true;
  }
  return false;
}

void lineAlignmentIter(double Kp) {
  if (isLineCrossedFully()) {
    // Serial.println(now_loc - prev_loc);

    if (now_loc - prev_loc > distToCounts(100)) {

      is_aligning = true;
      disableLinePID();

      Serial.println("New Edge Crossed");
      basePWM = -40;
      cross_count = 1;
      digitalWrite(LEDpin, HIGH);

    } else if (is_aligning) {

      basePWM = -basePWM;
      cross_count++;

      // Serial.print("Cross count: ");
      // Serial.p+-rintln(cross_count);

      if (cross_count > 6) {
        basePWM = 40;
        onLineCross();
        Serial.print("cross iwarai");
        is_aligning = false;




        // brakeHIGH(leftMotor);
        // brakeHIGH(rightMotor);
        // delay(10);
        digitalWrite(LEDpin, LOW);
        resetEncoders();
      }
    }

    alignment_output = Kp * (double)(lt - rt - 7);
    Serial.print(Kp * (double)(lt - rt - 7));
    Serial.print(" ");
    Serial.print(lt);
    Serial.print(" ");
    Serial.println(rt);

    // Serial.print("prev_loc: ");
    // Serial.println(prev_loc);
    // Serial.print("now_loc: ");
    // Serial.println(now_loc);
    lt = -1;
    rt = -1;

    prev_loc = now_loc;
  }
}

void linePIDEnableCheck() {
  if (pos.count >= distToCounts(130) && is_aligning == false) {
    enableLinePID();
    Serial.println("linePID enabled");
  }
}

void solveGrid() {
  basePWM = 40;
  while (true) {
    double sync_correction = wheelSyncIter(
      0,
      0.25, 0.0, 0.01,
      -100, 100);

    lineAlignmentIter(0.4);
    // Serial.println(is_aligning);
    linePIDEnableCheck();

    double left_pwm;
    double right_pwm;

    if (is_aligning) {
      Serial.println("Aligning");
      left_pwm = basePWM + alignment_output;
      right_pwm = basePWM - alignment_output;
    } else if (isLinePIDEnabled()) {
      Serial.println("LinePID");
      basePWM = 60;
      double linePID_correction = linePIDIter(
        10.5, 0.2, 1,  // kp, ki, kd
        -100, 100      // integral clamp
      );
      Serial.println(linePID_correction);
      left_pwm = 45 + linePID_correction;
      right_pwm = 70 - linePID_correction;

    } else {
      Serial.println("Encoders");
      basePWM = 60;
      left_pwm = basePWM - sync_correction;
      right_pwm = basePWM + sync_correction;
    }

    // setPWM(leftMotor, 0);
    // setPWM(rightMotor, 0);
    setPWM(leftMotor, left_pwm);
    setPWM(rightMotor, right_pwm);

    delay(5);
  }
}
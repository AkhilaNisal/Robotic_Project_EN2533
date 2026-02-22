double dottedLineFollowIter(double kp, double ki, double kd, double integral_min, double integral_max) {

  unsigned long now = micros();
  double dt = (double)(now - line_prev_micros) / 1e6;

  int sensor_values[8];
  qtr.read(sensor_values);

  long weighted_sum = 0;
  int active_sensors = 0;

  int leftIR = (analogRead(A13) < tcrt_threshold) ? 1 : 0;
  int rightIR = (analogRead(A12) < tcrt_threshold) ? 1 : 0;

  int sensor_weights[8] = { -100, -50, -20, -5, 5, 20, 50, 100 };  //16

  weighted_sum += (leftIR * -150) + (rightIR * 150);

  for (uint16_t i = 0; i < 8; i++) {
    int value = (sensor_values[i] < threshold);

    weighted_sum += value * sensor_weights[i];
    active_sensors += value;
  }



  active_sensors += leftIR + rightIR;

  double error = (active_sensors != 0) ? (double)weighted_sum / active_sensors : 0;
  Serial.print(weighted_sum);
  Serial.print(" ");
  Serial.print(active_sensors);
  Serial.print(" ");
  Serial.println(error);

  line_integral += 0.5 * (line_prev_error + error) * dt * ki;
  line_integral = constrain(line_integral, integral_min, integral_max);

  double derivative = (error - line_prev_error) / dt;

  double output = kp * error + line_integral + kd * derivative;

  line_prev_error = error;
  line_prev_micros = now;

  return output;
}


void moveForwardDottedLine() {
  double left_pwm = basePWM;
  double right_pwm = basePWM; 
  basePWM = 50;
  bool line_detected = false;

  while (true) {

    if (allSensorsWhite()) {
      brakeHIGH(leftMotor);
      brakeHIGH(rightMotor);
      delay(1000);
      break;
    }
    Serial.print("any");
    Serial.print(" ");
    Serial.print(anySensorWhite());
    Serial.print(" ");

    if (!line_detected && anySensorWhiteEx()) {
      line_detected = true;
      line_prev_micros = micros();
      line_integral = 0;
      line_prev_error = 0;
    }
    else if (line_detected && allSensorsBlackEx()) {
      line_detected = false;
      sync_integral = 0;
      sync_prev_err = 0;
      sync_prev_micros = micros();
      resetEncoders();
    }

    double line_correction = dottedLineFollowIter(
      3, 1, 0.15,
      -100, 100);

    double sync_correction = wheelSyncIter(
      0,             // target diff: 0 → wheels match
      1.5, 0, 0.05,  // kp, ki, kd
      -100, 100);    // integral clamp



    if (line_detected) {
      Serial.println("Line detected");
      left_pwm = basePWM + line_correction;
      right_pwm = basePWM - line_correction;
    } else {
      // Serial.println("encoders");
      left_pwm = basePWM - sync_correction;
      right_pwm = basePWM + sync_correction;
    }

    setPWM(leftMotor, left_pwm);
    setPWM(rightMotor, right_pwm);
    Serial.print(left_pwm);
    Serial.print(" ");
    Serial.println(right_pwm);

    
    delay(10);
  }
}


// void microSearch(int target_turn_count, double kp, double ki, double kd,
//                  double integral_min, double integral_max) {

//   resetEncoders();
//   double high_speed = 2000;
//   double low_speed = 500;  //ticks ps

//   double ticks_ps_L;
//   double ticks_ps_R;

//   int lprev_enc = 0;
//   int rprev_enc = 0;

//   double lprev_error = 0;
//   double rprev_error = 0;

//   double l_integral = 0;
//   double r_integral = 0;

//   long prev_micros = micros();

//   bool dir = false;
//   int turn_count = 0;

//   bool line_detected = false;
//   while (true) {
//     // dir = !dir;
//     // turn_count++;
//     double line_correction = lineFollowIter(
//       200, 0, 0.1,
//       -100, 100);

//     if (anySensorWhite() && !line_detected) {
//       line_detected = true;
//     }

//     if (allSensorsBlack() && line_detected) {
//       line_detected = false;
//       resetEncoders();
//       turn_count = 0;

//       lprev_enc = 0;
//       rprev_enc = 0;

//       lprev_error = 0;
//       rprev_error = 0;

//       l_integral = 0;
//       r_integral = 0;
//     }

//     if ((dir ? leftMotor.count : rightMotor.count) >= target_turn_count && !line_detected) {
//       dir = !dir;
//       turn_count = 0;
//       resetEncoders();
//       brakeHIGH(leftMotor);
//       brakeHIGH(rightMotor);
//       lprev_enc = 0;
//       rprev_enc = 0;

//       lprev_error = 0;
//       rprev_error = 0;

//       l_integral = 0;
//       r_integral = 0;
//     }

//     if (line_detected) {

//       ticks_ps_L = (high_speed + low_speed) / 2 - line_correction;
//       ticks_ps_R = (high_speed + low_speed) / 2 + line_correction;
//     }
//     else {
//       if (dir) {
//         ticks_ps_L = high_speed;
//         ticks_ps_R = low_speed;
//       } else {
//         ticks_ps_L = low_speed;
//         ticks_ps_R = high_speed;
//       }
//     }


//     long now = micros();
//     double dt = (double)(now - prev_micros) / 1e6;
//     if (dt <= 0) dt = 0.001;

//     noInterrupts();
//     int l_enc = leftMotor.count;
//     int r_enc = rightMotor.count;
//     interrupts();

//     int dl = l_enc - lprev_enc;
//     int dr = r_enc - rprev_enc;

//     double left_speed = dl / dt;
//     double right_speed = dr / dt;

//     double errorL = ticks_ps_L - left_speed;
//     double errorR = ticks_ps_R - right_speed;

//     l_integral += errorL * dt * ki;
//     r_integral += errorR * dt * ki;

//     l_integral = constrain(l_integral, integral_min, integral_max);
//     r_integral = constrain(r_integral, integral_min, integral_max);

//     double lu = kp * errorL + l_integral + kd * (errorL - lprev_error) / dt;
//     double ru = kp * errorR + r_integral + kd * (errorR - rprev_error) / dt;

//     double left_pwm = lu;
//     double right_pwm = ru;

//     setPWM(leftMotor, left_pwm);
//     setPWM(rightMotor, right_pwm);


//     // Serial.print(lu);
//     // Serial.print(" ");
//     // Serial.print(ru);
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

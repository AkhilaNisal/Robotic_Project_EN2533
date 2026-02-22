unsigned long line_prev_micros = 0;
int threshold = 500;


double lineFollowIter(double kp, double ki, double kd, double integral_min, double integral_max) {

  unsigned long now = micros();
  double dt = (double)(now - line_prev_micros) / 1e6;

  int sensor_values[8];
  qtr.read(sensor_values);

  long weighted_sum = 0;
  int active_sensors = 0;

  int leftIR = (analogRead(A13) < tcrt_threshold) ? 1 : 0;
  int rightIR = (analogRead(A12) < tcrt_threshold) ? 1 : 0;

  int sensor_weights[8] = { -16, -9, -4, -1, 1, 4, 9, 16 };  //16

  weighted_sum += (leftIR * -100) + (rightIR * 100);

  for (uint16_t i = 0; i < 8; i++) {
    int value = (sensor_values[i] < threshold);

    weighted_sum += value * sensor_weights[i];
    active_sensors += value;
  }

  active_sensors += leftIR + rightIR;

  double error = (active_sensors != 0) ? (double)weighted_sum / active_sensors : 0;

  line_integral += 0.5 * (line_prev_error + error) * dt * ki;
  line_integral = constrain(line_integral, integral_min, integral_max);

  double derivative = (error - line_prev_error) / dt;

  double output = kp * error + line_integral + kd * derivative;

  line_prev_error = error;
  line_prev_micros = now;

  return output;
}

void moveForwardLine() {

  basePWM = 60;

  while (true) {
    double line_correction = lineFollowIter(
      7, 0.5, 0.5,
      -100, 100);

    Serial.println(line_correction);

    double left_pwm = basePWM + line_correction;
    double right_pwm = basePWM - line_correction;

    setPWM(leftMotor, left_pwm);
    setPWM(rightMotor, right_pwm);
    delay(10);
  }
}

void moveForwardLineRamp() {

  basePWM = 40;

  while (true) {
    double line_correction = lineFollowIter(
      7, 0.5, 0.5,
      -100, 100);

    Serial.println(line_correction);

    double left_pwm = basePWM + line_correction;
    double right_pwm = basePWM - line_correction;

    int sensor_values[8];
    qtr.read(sensor_values);
    if (sensor_values[4] && sensor_values[5] && sensor_values[6] && sensor_values[7]) {
      break;
    }

    setPWM(leftMotor, left_pwm);
    setPWM(rightMotor, right_pwm);
    delay(10);
  }
}


// void slotedLineFollow() {
//   basePWM = 40;
//   double max_dist = 150;
//   bool encoder_drive = true;
//   double sync_correction = 0;
//   double prev_center = 0;
//   double last_center = 0;
//   double predicted_center;
//   double current_center;
//   resetEncoders();

//   while (true) {

//     if (anySensorWhite() && encoder_drive == true) {
//       prev_center = current_center;
//       encoder_drive = false;
//       Serial.println("white begin");
//     }

//     double direction_bias = 0;  // <—— added

//     if (allSensorsBlack()) {

//       // if (predicted_center > 0) {
//       //   direction_bias = +20;  // white was on right → turn right
//       // } else if (predicted_center < 0) {
//       //   direction_bias = -20;  // white was on left → turn left
//       // } else {
//       //   direction_bias = 0;
//       // }
//       // direction_bias = 20;



//       if (encoder_drive == false) {
//         Serial.println("black start");
//         Serial.println(last_center - prev_center);
//         // Serial.println(predicted_center);
//         resetEncoders();
//         encoder_drive = true;
//       }
//       if (distToCounts(max_dist) <= (leftMotor.count + rightMotor.count) / 2) {
//         Serial.println("stop");
//         brakeHIGH(leftMotor);
//         brakeHIGH(rightMotor);

//         delay(100);
//         break;
//       }
//       sync_correction = wheelSyncIter(
//         0,             // target diff: 0 → wheels match
//         1.5, 0, 0.05,  // kp, ki, kd
//         -100, 100      // integral clamp
//       );
//     } else {
//       int sensor_values[8];
//       qtr.read(sensor_values);
//       long weighted_sum = 0;
//       int active_sensors = 0;
//       int sensor_weights[8] = { -16, -9, -4, -1, 1, 4, 9, 16 };
//       for (uint16_t i = 0; i < 8; i++) {
//         int value = (sensor_values[i] > threshold) ? 1 : 0;
//         weighted_sum += value * sensor_weights[i];
//         active_sensors += value;
//       }
//       // double error = (active_sensors != 0)
//       //                  ? (double)weighted_sum / active_sensors
//       //                  : 0;
//       current_center = (active_sensors != 0)
//                          ? (double)weighted_sum / active_sensors
//                          : 0;
//       // Serial.println(current_center);

//       if (active_sensors > 0) {

//         last_center = current_center;
//       }
//       predicted_center =
//         last_center + (last_center - prev_center);
//       // (y1) + (y1 - y0)  → weighted extrapolation

//       sync_correction = 0;
//       encoder_drive == false;
//     }

//     double line_correction = lineFollowIter(
//       4, 0.1, 0.2,
//       -100, 100);
//     // Serial.println(last_ir_error);
//     // Serial.print(direction_bias);
//     // Serial.print(" ");
//     // Serial.println(line_correction);


//     // double left_pwm = basePWM - line_correction - sync_correction;
//     // double right_pwm = basePWM + line_correction + sync_correction;
//     double left_pwm = basePWM - line_correction - direction_bias;
//     double right_pwm = basePWM + line_correction + direction_bias;

//     setPWM(leftMotor, left_pwm);
//     setPWM(rightMotor, right_pwm);
//     // setPWM(leftMotor, 0);
//     // setPWM(rightMotor, 0);
//     delay(10);
//   }
// }

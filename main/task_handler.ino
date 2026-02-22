void align() {
  basePWM = 60;
  long now_enc = 0;

  bool aligning_started = false;

  long left_enc = -1;
  long right_enc = -1;

  int sensor_values[8];
  qtr.read(sensor_values);
  bool left_ir_prev = (sensor_values[0] < 500);
  bool right_ir_prev = (sensor_values[6] < 500);
  int count = 0;

  double left_pwm = basePWM;
  double right_pwm = basePWM;
  resetEncoders();
  while (true) {
    qtr.read(sensor_values);
    noInterrupts();
    now_enc = pos.count;
    interrupts();

    bool left_ir_now = (sensor_values[0] < 500);
    bool right_ir_now = (sensor_values[6] < 500);

    if (left_ir_now != left_ir_prev && lt == -1) {
      left_enc = now_enc;
    }
    left_ir_prev = left_ir_now;

    if (right_ir_now != right_ir_prev && rt == -1) {
      right_enc = now_enc;
    }
    right_ir_prev = right_ir_now;

    if (left_enc != -1 && right_enc != -1) {
      aligning_started = true;
      basePWM = -basePWM;
      count++;
      double align_corr = 0.4 * (double)(left_enc - right_enc - 7);
      Serial.print(0.4 * (double)(lt - rt - 7));
      left_pwm = basePWM + align_corr;
      right_pwm = basePWM - align_corr;
      if (count > 7) {
        brakeHIGH(leftMotor);
        brakeHIGH(rightMotor);
        break;
      }

      left_enc = -1;
      right_enc = -1;
    }

    if (!aligning_started) {
      double sync_correction = wheelSyncIter(
        0,             // target diff: 0 → wheels match
        1.5, 0, 0.05,  // kp, ki, kd
        -100, 100      // integral clamp
      );

      left_pwm = basePWM - sync_correction;
      right_pwm = basePWM + sync_correction;
    }

    setPWM(leftMotor, left_pwm);
    setPWM(rightMotor, right_pwm);

    delay(10);
  }
}


void taskHandler() {
  //task 1
  // solveGrid();
  // align();
  // moveForwardDist(200);
  // //task 2 - dotted line & ramp push
  // moveForwardDottedLine();
  // align();


  // turnBack();
  // moveForwardDist(-350);
  // // moveForwardDist(100);
  // align();
  // moveForwardDist(130);

  // moveForwardDottedLine();
  // align();
  // moveForwardDist(-230);
  // turnRight();
  // //task2 - ramp
  // ramp(350, 0.1, 4, 0,
  //      -100, 100);

  align();
  moveForwardDist(100);


  moveForwardLineRamp();
  moveForwardDist(100);
  turnRight();

  // barcode()

  moveForwardDottedLine();

  // moveForwardLineRamp();
  moveForwardDist(100);
  // turnRight();
  //ball picking


  //arrow task
  arrow_task();
  align();
}
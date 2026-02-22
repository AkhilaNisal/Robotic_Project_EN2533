// void wallFollowLoop(int baseSpeed, int target) {
//   // --- PID gains ---
//   const double Kp_d = 0.2, Ki_d = 0.1, Kd_d = 0.0;
//   const double Kp_a = 0.2, Ki_a = 0.1, Kd_a = 0.0;

//   // --- Internal PID state ---
//   double dist_integral = 0;
//   double last_dist_error = 0;
//   double angle_integral = 0;
//   double last_angle_error = 0;
//   unsigned long last_t = millis();

//   bool movingback = false;

//   while (true) {
//     // --- Time delta ---
//     unsigned long now = millis();
//     double dt = (now - last_t) / 1000.0;
//     if (dt <= 0) dt = 0.001;
//     last_t = now;

//     // --- Read sensors ---
//     int FLd = FL.readRange();
//     int BLd = BL.readRange();

//     Serial.println(FLd);

//     if (FLd > 400 && movingback == false) {
//       baseSpeed = -baseSpeed;
//       setPWM(leftMotor, baseSpeed);
//       setPWM(rightMotor, baseSpeed);
//       movingback = true;
//       resetEncoders();
//     }

//     if (FLd > 400 || BLd > 400) continue;

//     if (movingback && pos.count <= distToCounts(-50)) {

//       brakeHIGH(leftMotor);
//       brakeHIGH(rightMotor);
//       delay(100);
//       baseSpeed = -baseSpeed;
//       movingback = false;
//     }

//     if (movingback) {
//       Serial.println("Moving back");
//     }



//     // --- Distance PID ---
//     double dist_error = FLd - target;
//     dist_integral += dist_error * dt;
//     dist_integral = constrain(dist_integral, -200, 200);
//     double dist_derivative = (dist_error - last_dist_error) / dt;
//     last_dist_error = dist_error;
//     double dist_pid = Kp_d * dist_error + Ki_d * dist_integral + Kd_d * dist_derivative;



//     // --- Angle PID ---
//     double angle_error = FLd - BLd;
//     angle_integral += angle_error * dt;
//     angle_integral = constrain(angle_integral, -50, 50);
//     double angle_derivative = (angle_error - last_angle_error) / dt;
//     last_angle_error = angle_error;
//     double angle_pid = Kp_a * angle_error + Ki_a * angle_integral + Kd_a * angle_derivative;

//     // --- Combine ---
//     double correction = dist_pid + angle_pid;

//     // --- Apply to motors ---
//     int L = baseSpeed - correction;
//     int R = baseSpeed + correction;
//     // setPWM(leftMotor, 0);
//     // setPWM(rightMotor, 0);

//     setPWM(leftMotor, L);
//     setPWM(rightMotor, R);
//     // --- Optional debug ---
//     Serial.print("DistErr:");
//     Serial.print(dist_error);
//     Serial.print(" AngleErr:");
//     Serial.print(angle_error);
//     Serial.print(" Corr:");
//     Serial.println(correction);

//     delay(10);  // loop ~100 Hz
//   }
// }


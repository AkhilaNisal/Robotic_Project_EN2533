
bool linePIDEnabled = false;

long linePIDPrevMicros = 0;
double linePIDIntegral = 0;
double linePIDPrevError = 0;
double linePIDThreshold = 330;  

double linePIDIter(double kp, double ki, double kd,
                   double integral_min, double integral_max) {

  int leftIR = (analogRead(A13) < linePIDThreshold) ? 1 : 0;
  int rightIR = (analogRead(A12) < linePIDThreshold) ? 1 : 0;

  double error = (double)leftIR - (double)rightIR;

  long now = micros();

  double dt = (linePIDPrevMicros == 0) ? 0.0 : (now - linePIDPrevMicros) / 1e6;

  linePIDPrevMicros = now;

  if (dt <= 0) {
    linePIDPrevError = error;
    return kp * error;
  }

  linePIDIntegral += 0.5 * (error + linePIDPrevError) * dt * ki;
  linePIDIntegral = constrain(linePIDIntegral, integral_min, integral_max);

  double derivative = (error - linePIDPrevError) / dt;

  double output = kp * error + linePIDIntegral + kd * derivative;

  linePIDPrevError = error;

  return output;
}

void enableLinePID() {
  linePIDEnabled = true;
  linePIDPrevMicros = 0;
  linePIDIntegral = 0;
  linePIDPrevError = 0;
}

void disableLinePID() {
  linePIDEnabled = false;
}

bool isLinePIDEnabled() {
  return linePIDEnabled;
}

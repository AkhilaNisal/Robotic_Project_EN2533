bool allSensorsBlack(uint16_t threshold = 500) {
  int sensor_values[8];
  qtr.read(sensor_values);

  for (int i = 0; i < 8; i++) {
    if (sensor_values[i] <= threshold) {
      return false;
    }
  }

  return true;
}


bool allSensorsBlackEx(uint16_t threshold = 500) {
  int sensor_values[8];
  qtr.read(sensor_values);

  for (int i = 0; i < 8; i++) {
    if (sensor_values[i] <= threshold) {
      return false;
    }
  }
  if (analogRead(A12) <= tcrt_threshold || analogRead(A13) <= tcrt_threshold) return false;



  return true;
}



bool allSensorsWhite(uint16_t threshold = 500) {
  int sensor_values[8];
  qtr.read(sensor_values);

  for (int i = 0; i < 8; i++) {
    if (sensor_values[i] >= threshold) {
      return false;
    }
  }

  return true;
}


bool anySensorWhite(uint16_t threshold = 500) {
  int sensor_values[8];
  qtr.read(sensor_values);

  for (int i = 0; i < 8; i++) {
    if (sensor_values[i] <= threshold) {
      return true;
    }
  }


  return false;
}

bool anySensorWhiteEx(uint16_t threshold = 500) {
  int sensor_values[8];
  qtr.read(sensor_values);

  for (int i = 0; i < 8; i++) {
    if (sensor_values[i] <= threshold) {
      return true;
    }
  }

  if (analogRead(A12) < tcrt_threshold || analogRead(A13) < tcrt_threshold) return true;

  return false;
}
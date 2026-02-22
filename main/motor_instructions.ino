void setPWM(Motor &m, double pwm) {
  float abs_pwm = constrain(abs(pwm), 0, 255);

  if (pwm > 0) {
    digitalWrite(m.pinA, HIGH);
    digitalWrite(m.pinB, LOW);
    analogWrite(m.pwmPin, abs_pwm);

  } else if (pwm < 0) {
    digitalWrite(m.pinA, LOW);
    digitalWrite(m.pinB, HIGH);
    analogWrite(m.pwmPin, abs_pwm);
  } else {
    digitalWrite(m.pinA, LOW);
    digitalWrite(m.pinB, LOW);
    analogWrite(m.pwmPin, 0);
  }
}

void brakeLOW(Motor &m) {
  digitalWrite(m.pinA, LOW);
  digitalWrite(m.pinB, LOW);
  analogWrite(m.pwmPin, 0);
}

void brakeHIGH(Motor &m) {
  digitalWrite(m.pinA, HIGH);
  digitalWrite(m.pinB, HIGH);
  analogWrite(m.pwmPin, 0);
}
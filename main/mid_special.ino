

void checkObject() {
  while (true) {
    int flud = FLT.readRange();
    int fld = FL.readRange();
    if (flud < 50) {
      // espSerial.println(-1);
      Serial2.println('o');
      Serial.println("obstacle detected");


    } else if (fld < 50) {
      // espSerial.println(1);
      Serial2.println('b');
      Serial.println("object detected");

    } else {
      // espSerial.println(0);
      Serial2.println('f');
      Serial.println("no object detected");
    }
    delay(100);
  }
}

void checkColor() {
  
  Serial2.println('c');
}

void ballPicking() {
  Serial2.println('p');
}

void objectPicking() {
  Serial2.println('d');
}
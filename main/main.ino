#include <QTRSensors.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_VL53L0X.h"

#define INVALID_READING 1000

#define XSHUT_FL 33
#define XSHUT_BL 32
#define XSHUT_FLT 24

#define GPIO_FL 20
#define GPIO_BL 21

#define LEDpin 13

double sync_prev_err = 0;
unsigned long sync_prev_micros = 0;
double sync_integral = 0;
int tcrt_threshold = 400;

double tof_alpha = 0.3;  // smoothing factor (lower = smoother)

bool task3 = false;
double line_prev_error = 0;
double line_integral = 0;

double fl_filtered = 0;
double bl_filtered = 0;

long now_loc;
long lt = -1;
long rt = -1;
bool left_prev = false;
bool right_prev = false;

Adafruit_VL53L0X fl_tof = Adafruit_VL53L0X();
Adafruit_VL53L0X bl_tof = Adafruit_VL53L0X();

Adafruit_VL53L0X FL = Adafruit_VL53L0X();
Adafruit_VL53L0X BL = Adafruit_VL53L0X();
Adafruit_VL53L0X FLT = Adafruit_VL53L0X();


bool is_aligning = false;

long prev_loc = -10000;

uint16_t s[8];

const int TCS_LED_PIN = 46;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X);

double basePWM = 180;

struct Motor {
  int pinA;
  int pinB;
  int pwmPin;

  int encA;
  int encB;

  volatile long count;
  volatile double rpm;
};

struct Config {
  int basePWM;
};

Config config = {
  180
};

struct Physical {
  int encoderCPR = 896;
  double diameter = 68;
  double wheelDist = 162;
};

Physical physical{};

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensors[SensorCount];

struct Pos {
  int x;
  int y;

  long target_count;
  volatile long count;
};

Pos pos = { 0, 0, 0, 0 };

Motor leftMotor = {
  42, 40,  // dir pins
  12,      // PWM
  2, 3,    // enc A, B
  0, 0
};

Motor rightMotor = {
  38, 36,  // dir pins
  11,      // PWM
  18, 19,  // enc A, B
  0, 0
};

void leftA_ISR() {
  int a = digitalRead(leftMotor.encA);
  int b = digitalRead(leftMotor.encB);
  if (a == b) leftMotor.count++;
  else leftMotor.count--;
  pos.count = (leftMotor.count + rightMotor.count) / 2;
}

void leftB_ISR() {
  int a = digitalRead(leftMotor.encA);
  int b = digitalRead(leftMotor.encB);
  if (a != b) leftMotor.count++;
  else leftMotor.count--;
  pos.count = (leftMotor.count + rightMotor.count) / 2;
}

void rightA_ISR() {
  int a = digitalRead(rightMotor.encA);
  int b = digitalRead(rightMotor.encB);
  if (a == b) rightMotor.count++;
  else rightMotor.count--;
  pos.count = (leftMotor.count + rightMotor.count) / 2;
}
void rightB_ISR() {
  int a = digitalRead(rightMotor.encA);
  int b = digitalRead(rightMotor.encB);
  if (a != b) rightMotor.count++;
  else rightMotor.count--;
  pos.count = (leftMotor.count + rightMotor.count) / 2;
}

long getEncoder(const Motor &m) {
  noInterrupts();
  long c = m.count;
  interrupts();
  return c;
}


void setup() {

  Serial.begin(115200);

  Serial2.begin(115200);  // TX1=18, RX1=19
  Serial2.setTimeout(2);

  Wire.begin();

  pinMode(leftMotor.pinA, OUTPUT);
  pinMode(leftMotor.pinB, OUTPUT);
  pinMode(leftMotor.pwmPin, OUTPUT);

  pinMode(rightMotor.pinA, OUTPUT);
  pinMode(rightMotor.pinB, OUTPUT);
  pinMode(rightMotor.pwmPin, OUTPUT);

  pinMode(leftMotor.encA, INPUT_PULLUP);
  pinMode(leftMotor.encB, INPUT_PULLUP);
  pinMode(rightMotor.encA, INPUT_PULLUP);
  pinMode(rightMotor.encB, INPUT_PULLUP);

  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, LOW);

  pinMode(XSHUT_FL, OUTPUT);
  pinMode(XSHUT_FLT, OUTPUT);
  pinMode(XSHUT_BL, OUTPUT);

  digitalWrite(XSHUT_FL, LOW);
  digitalWrite(XSHUT_FLT, LOW);
  digitalWrite(XSHUT_BL, LOW);
  delay(10);

  // digitalWrite(XSHUT_FL, HIGH);
  // delay(10);
  // if (!FL.begin(0x30)) {
  //   Serial.println("FL FAIL");
  //   while (1)
  //     ;
  // }

  // digitalWrite(XSHUT_BL, HIGH);
  // delay(10);
  // if (!BL.begin(0x31)) {
  //   Serial.println("BL FAIL");
  //   while (1)
  //     ;
  // }

  // digitalWrite(XSHUT_FLT, HIGH);
  // delay(10);
  // if (!FLT.begin(0x32)) {
  //   Serial.println("FLT FAIL");
  //   while (1)
  //     ;
  // }

  attachInterrupt(digitalPinToInterrupt(leftMotor.encA), leftA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftMotor.encB), leftB_ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(rightMotor.encA), rightA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightMotor.encB), rightB_ISR, CHANGE);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);

  pinMode(TCS_LED_PIN, OUTPUT);
  digitalWrite(TCS_LED_PIN, LOW);

  //mid
  // ramp();
  // moveForwardLine();
  // wallFollow();
  // checkObject();
  // checkColor();
  // ballPicking();
  // objectPicking();

  //moveForwardEncoders

  // moveForwardDist(-200);
  // moveForwardEncoders();
  // turnLeft();
  // void moveAlongCircle(double r, double pwm1,
  //                    double kp, double ki, double kd,
  //                    double integral_min, double integral_max,
  //                    double kpw, double kiw, double kdw,
  //                    double integral_minw, double integral_maxw)
  // moveAlongCircle(300, 60,
  //                 0.1, 0.001, 0,
  //                 -100, 100,
  //                 0, 0, 0,
  //                 -100, 100);
  // moveAlongCircle(350, 545, 0.1, 4, 0, -100, 100);

  // ramp(350, 0.1, 4, 0.000,
  //      -100, 100);

  // moveForwardDist(200);
  // turnRight();
  // wallFollowLoop(50, 150);
  // wallFollow();
  // arrow_find(0)
  // arrow_task();
  // slotedLineFollow();
  // microSearch(800, 0.1, 4, 0, -100, 100);
   moveForwardLine();
  // moveForwardLineEx();
  // checkObject();

  // taskHandler();
  // turnBack();
  // align();
  // moveForwardDottedLine();
  // arrow_task();
}

void loop() {

  // int sensor_values[8];
  // qtr.read(sensor_values);
  // long weighted_sum = 0;
  // int active_sensors = 0;
  // int sensor_weights[8] = { -16, -9, -4, -1, 1, 4, 9, 16 };
  // int leftIR = (analogRead(A13) < 350) ? 1 : 0;
  // int rightIR = (analogRead(A12) < 350) ? 1 : 0;
  // for (uint16_t i = 0; i < 8; i++) {
  //   int value = (sensor_values[i] < 500) ? 1 : 0;
  //   // Serial.println(value);
  //   weighted_sum += value * sensor_weights[i];
  //   active_sensors += value;
  // }
  // double error = (active_sensors != 0) ? (double)weighted_sum / active_sensors : 0;
  // error = error + leftIR * -25 + rightIR * 25;
  // Serial.println(error);

  // int sensor_values[8];
  // qtr.read(sensor_values);


  // Serial.print(analogRead(A13));
  // Serial.print(" ");
  // for (uint16_t i = 0; i < 8; i++) {
  //   Serial.print(sensor_values[i]);
  //   Serial.print(" ");
  // }
  // Serial.print(" ");
  // Serial.println(analogRead(A12));

  // Serial.print(FLT.readRange());
  // Serial.print(" ");
  // Serial.print(FL.readRange());
  // Serial.print(" ");
  // Serial.println(BL.readRange());
  // delay10);

  // setPWM(leftMotor,50);
  // setPWM(rightMotor,90);
  // Serial.print(analogRead(A13));
  // Serial.print(" ");
  // Serial.println(analogRead(A12));
}
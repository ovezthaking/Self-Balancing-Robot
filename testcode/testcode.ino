#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = true;
uint8_t mpuIntStatus;
uint8_t devStatus; //status after each device operation
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

//PID
double originalSetpoint = 175.8;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState = 0; // we'll see

// Detekcja upadku
#define FALL_ANGLE_THRESHOLD 30.0  // kąt w stopniach - jeśli robot jest bardziej przechylony, uznaj za upadek
#define RECOVERY_ANGLE_THRESHOLD 15.0  // kąt powrotu do normalnego działania
bool robotFallen = false;
unsigned long fallStartTime = 0;
#define FALL_TIMEOUT 3000  // 3 sekundy timeout na próbę powrotu

//user's own fit
double Kp = 50;
double Kd = 1.4;
double Ki = 60;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.9;
double motorSpeedFactorRight = 0.9;

//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // WAŻNE: Offset dla obróconych osi - dostosuj te wartości do swojego modułu
  // Po obróceniu o 90° w yaw, osie X i Y są zamienione
  mpu.setXGyroOffset(76);    // była wartość Y
  mpu.setYGyroOffset(-220);  // była wartość X z przeciwnym znakiem
  mpu.setZGyroOffset(-85);   // Z pozostaje bez zmian
  mpu.setZAccelOffset(1788);

  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    // ERROR!
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop()
{
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // Sprawdź czy robot nie upadł przed obliczeniem PID
    if (!robotFallen) {
      pid.Compute();
      motorController.move(output, MIN_ABS_SPEED);
    } else {
      // Robot upadł - zatrzymaj silniki
      motorController.move(0, 0);
      
      // Sprawdź timeout - jeśli robot leży zbyt długo, zatrzymaj całkowicie
      if (millis() - fallStartTime > FALL_TIMEOUT) {
        Serial.println(F("Robot fallen - timeout reached, stopping motors"));
        motorController.move(0, 0);
        delay(100);  // krótka pauza
      }
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // PROSTSZE PODEJŚCIE: Bezpośrednie dodanie 90° do yaw
    // Standardowy odczyt YPR
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // Dodaj 90° (π/2 radianów) do yaw
    ypr[0] += M_PI/2;
    
    // Normalizuj do zakresu -π do π
    if (ypr[0] > M_PI) ypr[0] -= 2*M_PI;
    if (ypr[0] < -M_PI) ypr[0] += 2*M_PI;

    input = ypr[1] * 180 / M_PI + 180;
    
    // DETEKCJA UPADKU
    float currentAngle = abs(ypr[1] * 180 / M_PI);  // pitch w stopniach (bezwzględna wartość)
    
    if (!robotFallen) {
      // Robot normalnie działa - sprawdź czy nie upadł
      if (currentAngle > FALL_ANGLE_THRESHOLD) {
        robotFallen = true;
        fallStartTime = millis();
        Serial.print(F("Robot fallen! Angle: "));
        Serial.println(currentAngle);
        
        // Zatrzymaj silniki natychmiast
        motorController.move(0, 0);
        
        // Reset PID aby uniknąć nagromadzonego błędu
        pid.SetMode(MANUAL);
        output = 0;
        pid.SetMode(AUTOMATIC);
      }
    } else {
      // Robot upadł - sprawdź czy się podniósł
      if (currentAngle < RECOVERY_ANGLE_THRESHOLD) {
        robotFallen = false;
        Serial.println(F("Robot recovered!"));
        
        // Reset PID na świeży start
        pid.SetMode(MANUAL);
        output = 0;
        pid.SetMode(AUTOMATIC);
      }
    }

#if LOG_INPUT
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\tFallen: ");
    Serial.println(robotFallen ? "YES" : "NO");
#endif
  }
}
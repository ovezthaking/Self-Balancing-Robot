#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LOG_INPUT 0
#define MIN_ABS_SPEED 30

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
double originalSetpoint = 155.74;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState = 0; // we'll see

//user's own fit
// double Kp = 50;
// double Kd = 1.4;
// double Ki = 60;
// double Kp = 55;
// double Kd = 0.9;
// double Ki = 0.6;
double Kp = 60;
double Kd = 2.2;
double Ki = 400;
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
  // mpu.setXGyroOffset(76);    // była wartość Y
  // mpu.setYGyroOffset(-220);  // była wartość X z przeciwnym znakiem
  // mpu.setZGyroOffset(-85);   // Z pozostaje bez zmian
  // mpu.setZAccelOffset(1788);
  mpu.setXAccelOffset(-1214);
  mpu.setYAccelOffset(-4095);
  mpu.setZAccelOffset(949);
  mpu.setXGyroOffset(66);
  mpu.setYGyroOffset(21);
  mpu.setZGyroOffset(26);

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
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
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

    // GŁÓWNA ZMIANA: Rotacja kwaterniona o 90° w osi Z (yaw)
    // Dla obrotu o 90° w lewo (przeciwnie do ruchu wskazówek zegara)
    Quaternion rotationZ;
    rotationZ.w = cos(M_PI/4);  // cos(90°/2) = cos(45°)
    rotationZ.x = 0;
    rotationZ.y = 0;
    rotationZ.z = sin(M_PI/4);  // sin(90°/2) = sin(45°)
    
    // Mnożenie kwaternionów: q_rotated = rotationZ * q
    Quaternion q_rotated;
    q_rotated.w = rotationZ.w * q.w - rotationZ.z * q.z;
    q_rotated.x = rotationZ.w * q.x + rotationZ.z * q.y;
    q_rotated.y = rotationZ.w * q.y - rotationZ.z * q.x;
    q_rotated.z = rotationZ.w * q.z + rotationZ.z * q.w;
    
    // Oblicz YPR z obróconym kwaternionem
    mpu.dmpGetGravity(&gravity, &q_rotated);
    mpu.dmpGetYawPitchRoll(ypr, &q_rotated, &gravity);

#if LOG_INPUT
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
    Serial.print("\tPID_out\t");
    Serial.println(output);
    Serial.print("\tMotor_factors\t");
    Serial.print(motorSpeedFactorLeft);
    Serial.print("\t");
    Serial.println(motorSpeedFactorRight);
#endif
    input = ypr[1] * 180 / M_PI + 180;
  }
}

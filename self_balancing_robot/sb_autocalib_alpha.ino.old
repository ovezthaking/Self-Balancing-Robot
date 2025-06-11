#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20
#define LOG_INPUT 0
#define AUTO_CALIBRATE 1  // Ustaw na 1 aby włączyć autokalibrację

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
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
double originalSetpoint = 175.8;  // Możesz potrzebować dostroić tę wartość
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState = 0;

//user's own fit - możesz potrzebować dostroić te parametry
double Kp = 50;
double Kd = 1.4;
double Ki = 60;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;

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

// Funkcja do automatycznej kalibracji offsetów
void calibrateMPU() {
  Serial.println(F("Rozpoczynam automatyczną kalibrację..."));
  Serial.println(F("Ustaw robota w pozycji pionowej i nie ruszaj przez 10 sekund!"));
  
  // Czas na ustawienie robota
  for(int i = 5; i >= 1; i--) {
    Serial.print(F("Start za: "));
    Serial.println(i);
    delay(1000);
  }
  
  Serial.println(F("Kalibracja w toku..."));
  
  // Resetuj offsety na początek
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int16_t ax, ay, az, gx, gy, gz;
  
  // Zbierz 1000 próbek
  for(int i = 0; i < 1000; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(3);
  }
  
  // Oblicz średnie offsety
  int ax_offset = -ax_sum / 1000;
  int ay_offset = -ay_sum / 1000;
  int az_offset = (16384 - az_sum / 1000); // 16384 = 1g dla akcelerometru
  int gx_offset = -gx_sum / 1000;
  int gy_offset = -gy_sum / 1000;
  int gz_offset = -gz_sum / 1000;
  
  // Ustaw obliczone offsety
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  
  Serial.println(F("Kalibracja zakończona!"));
  Serial.println(F("Obliczone offsety:"));
  Serial.print(F("Akcelerometr: X="));
  Serial.print(ax_offset);
  Serial.print(F(", Y="));
  Serial.print(ay_offset);
  Serial.print(F(", Z="));
  Serial.println(az_offset);
  Serial.print(F("Żyroskop: X="));
  Serial.print(gx_offset);
  Serial.print(F(", Y="));
  Serial.print(gy_offset);
  Serial.print(F(", Z="));
  Serial.println(gz_offset);
  Serial.println(F("Możesz wpisać te wartości na stałe w kod aby przyspieszyć następne uruchomienia."));
}

// Funkcja do automatycznego znajdowania setpoint
void findSetpoint() {
  Serial.println(F("Szukam punktu równowagi..."));
  Serial.println(F("Ustaw robota w idealnej pozycji równowagi na 5 sekund!"));
  
  delay(2000); // Czas na przygotowanie
  
  float roll_sum = 0;
  int samples = 0;
  
  unsigned long start_time = millis();
  while(millis() - start_time < 5000) { // 5 sekund próbkowania
    if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      roll_sum += (ypr[2] * 180 / M_PI + 180);
      samples++;
    }
    delay(10);
  }
  
  if(samples > 0) {
    double calculated_setpoint = roll_sum / samples;
    Serial.print(F("Obliczony setpoint: "));
    Serial.println(calculated_setpoint);
    
    // Aktualizuj setpoint
    originalSetpoint = calculated_setpoint;
    setpoint = originalSetpoint;
    
    Serial.println(F("Setpoint został automatycznie ustawiony!"));
  } else {
    Serial.println(F("Błąd: Nie udało się zebrać próbek"));
  }
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

#if AUTO_CALIBRATE
  // Automatyczna kalibracja offsetów
  calibrateMPU();
#else
  // Ręczne offsety gyroskopu - możesz je zastąpić wartościami z autokalibracji
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
#endif

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
    
#if AUTO_CALIBRATE
    // Automatyczne znalezienie setpoint
    findSetpoint();
#endif
  }
  else
  {
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

#if LOG_INPUT
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

    // GŁÓWNA ZMIANA: używamy roll (ypr[2]) zamiast pitch (ypr[1]) [bo chciałbym bokiem]
    input = ypr[2] * 180 / M_PI + 180;
  }
}

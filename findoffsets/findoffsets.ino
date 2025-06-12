// PIERWSZY KROK: Kod do kalibracji offsetów MPU6050
// WAŻNE: Ten kod uwzględnia obrót MPU6050 o 90° w osi yaw
// Wgraj ten kod osobno, żeby znaleźć właściwe offsety dla twojego modułu

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  mpu.initialize();
  
  Serial.println("===== KALIBRACJA OFFSETÓW MPU6050 (OBRÓCONY 90°) =====");
  Serial.println("WAŻNE: Postaw robota z MPU6050 obróconą o 90° tak jak będzie pracować!");
  Serial.println("Powierzchnia musi być idealnie płaska!");
  Serial.println("Naciśnij dowolny klawisz aby rozpocząć...");
  
  while (Serial.available() && Serial.read()); // wyczyść bufor
  while (!Serial.available());                 // czekaj na input
  while (Serial.available() && Serial.read()); // wyczyść bufor ponownie
  
  Serial.println("Rozpoczynam kalibrację...");
  calibrate();
}

void loop() {
  // Test offsetów - sprawdź czy wartości są blisko zera dla żyroskopu
  // i czy Z-axis akcelerometru pokazuje ~16384 (1g)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  Serial.print("Akcelerometr: ");
  Serial.print("X="); Serial.print(ax);
  Serial.print(" Y="); Serial.print(ay);
  Serial.print(" Z="); Serial.print(az);
  Serial.print("  Żyroskop: ");
  Serial.print("X="); Serial.print(gx);
  Serial.print(" Y="); Serial.print(gy);
  Serial.print(" Z="); Serial.println(gz);
  
  // Sprawdź jakość kalibracji
  if (abs(gx) < 50 && abs(gy) < 50 && abs(gz) < 50) {
    Serial.println("✓ Żyroskop dobrze skalibrowany");
  } else {
    Serial.println("⚠ Żyroskop może wymagać ponownej kalibracji");
  }
  
  if (abs(az - 16384) < 500) {  // Z-axis powinien pokazywać ~16384 dla 1g
    Serial.println("✓ Akcelerometr Z dobrze skalibrowany");
  } else {
    Serial.println("⚠ Akcelerometr Z może wymagać ponownej kalibracji");
  }
  
  delay(1000);
}

void calibrate() {
  Serial.println("Kalibracja offsetów...");
  
  // Reset offsetów
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  
  for (int i = 0; i < 3; i++) {
    if (i > 0) {
      Serial.println("Kładę robota na drugiej stronie i czekam...");
      delay(3000);
    }
    
    Serial.print("Kalibracja "); Serial.print(i + 1); Serial.println("/3...");
    
    meansensors();
    calibration();
    
    Serial.println("Aktualne offsety:");
    Serial.print("  setXAccelOffset("); Serial.print(ax_offset); Serial.println(");");
    Serial.print("  setYAccelOffset("); Serial.print(ay_offset); Serial.println(");");
    Serial.print("  setZAccelOffset("); Serial.print(az_offset); Serial.println(");");
    Serial.print("  setXGyroOffset("); Serial.print(gx_offset); Serial.println(");");
    Serial.print("  setYGyroOffset("); Serial.print(gy_offset); Serial.println(");");
    Serial.print("  setZGyroOffset("); Serial.print(gz_offset); Serial.println(");");
    Serial.println();
  }
  
  Serial.println("===== FINALNE OFFSETY =====");
  Serial.println("Skopiuj te linie do swojego głównego kodu:");
  Serial.println("UWAGA: Te offsety są dla MPU6050 obróconym o 90° w yaw!");
  Serial.println();
  Serial.print("  mpu.setXAccelOffset("); Serial.print(ax_offset); Serial.println(");");
  Serial.print("  mpu.setYAccelOffset("); Serial.print(ay_offset); Serial.println(");");
  Serial.print("  mpu.setZAccelOffset("); Serial.print(az_offset); Serial.println(");");
  Serial.print("  mpu.setXGyroOffset("); Serial.print(gx_offset); Serial.println(");");
  Serial.print("  mpu.setYGyroOffset("); Serial.print(gy_offset); Serial.println(");");
  Serial.print("  mpu.setZGyroOffset("); Serial.print(gz_offset); Serial.println(");");
  Serial.println();
  Serial.println("INSTRUKCJA: W głównym kodzie użyj DOKŁADNIE tych wartości");
  Serial.println("NIE mieszaj z poprzednimi offsetami!");
  Serial.println("=============================");
}

void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  
  while (i < 1000) { // pobierz 1000 próbek
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    buff_ax += ax;
    buff_ay += ay;
    buff_az += az;
    buff_gx += gx;
    buff_gy += gy;
    buff_gz += gz;
    i++;
    delay(2);
  }
  
  mean_ax = buff_ax / 1000;
  mean_ay = buff_ay / 1000;
  mean_az = buff_az / 1000;
  mean_gx = buff_gx / 1000;
  mean_gy = buff_gy / 1000;
  mean_gz = buff_gz / 1000;
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8; // 16384 = 1g w skali akcelerometru
  
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
}
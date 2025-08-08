#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU9250_asukiaaa.h"

#define MUX_ADDR 0x70

const char* ssid = "Honor 8X";
const char* password = "qwert12345";
const char* remoteIP = "192.168.43.137";
const int remotePort = 4210;

WiFiUDP udp;

#define BUTTON_PIN 27

MPU6050 mpu6050[5];
MPU9250_asukiaaa mpu9250(0x68);

uint8_t fifoBuffers[5][64];
Quaternion quats_mpu6050[5];

float SamplePeriod = 0.02f;
float beta = 0.1f;
float zeta = 0.0f;
float q_palm[4] = {1.0f, 0.0f, 0.0f, 0.0f};

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);
}

void MadgwickAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz) {
  float recipNorm, s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q1mx, _2q2mx, _2q0my, _2q1my, _2q2my, _2q0mz, _2q1mz, _2q2mz;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, _8q3;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    qDot1 = 0.5f * (-q_palm[1] * gx - q_palm[2] * gy - q_palm[3] * gz);
    qDot2 = 0.5f * (q_palm[0] * gx + q_palm[2] * gz - q_palm[3] * gy);
    qDot3 = 0.5f * (q_palm[0] * gy - q_palm[1] * gz + q_palm[3] * gx);
    qDot4 = 0.5f * (q_palm[0] * gz + q_palm[1] * gy - q_palm[2] * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      _2q0 = 2.0f * q_palm[0];
      _2q1 = 2.0f * q_palm[1];
      _2q2 = 2.0f * q_palm[2];
      _2q3 = 2.0f * q_palm[3];
      _4q0 = 4.0f * q_palm[0];
      _4q1 = 4.0f * q_palm[1];
      _4q2 = 4.0f * q_palm[2];
      _8q1 = 8.0f * q_palm[1];
      _8q2 = 8.0f * q_palm[2];
      q0q0 = q_palm[0] * q_palm[0];
      q1q1 = q_palm[1] * q_palm[1];
      q2q2 = q_palm[2] * q_palm[2];
      q3q3 = q_palm[3] * q_palm[3];

      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q_palm[1] - _2q0 * ay - _4q1 + _2q0 * az;
      s2 = _4q2 * q3q3 + _2q3 * ay + 4.0f * q0q0 * q_palm[2] - _2q0 * ax - _4q2 + _2q0 * az;
      s3 = 4.0f * q1q1 * q_palm[3] - _2q1 * ax + 4.0f * q2q2 * q_palm[3] - _2q2 * ay;

      recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }
  } else {
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

    // ... (Full magnetometer version can be restored if needed)
  }

  q_palm[0] += qDot1 * SamplePeriod;
  q_palm[1] += qDot2 * SamplePeriod;
  q_palm[2] += qDot3 * SamplePeriod;
  q_palm[3] += qDot4 * SamplePeriod;

  recipNorm = 1.0f / sqrtf(q_palm[0]*q_palm[0] + q_palm[1]*q_palm[1] + q_palm[2]*q_palm[2] + q_palm[3]*q_palm[3]);
  q_palm[0] *= recipNorm;
  q_palm[1] *= recipNorm;
  q_palm[2] *= recipNorm;
  q_palm[3] *= recipNorm;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  for (int i = 0; i < 5; i++) {
    selectMuxChannel(i);
    Serial.printf("Initializing MPU6050 #%d...\n", i);
    mpu6050[i].initialize();
    if (!mpu6050[i].testConnection()) {
      Serial.printf("ERROR: MPU6050 #%d connection failed!\n", i);
    } else {
      Serial.printf("MPU6050 #%d connected.\n", i);
    }

    int status = mpu6050[i].dmpInitialize();
    if (status == 0) {
      mpu6050[i].setDMPEnabled(true);
      Serial.printf("MPU6050 #%d DMP enabled.\n", i);
    } else {
      Serial.printf("ERROR: MPU6050 #%d DMP init failed: %d\n", i, status);
    }
  }

  selectMuxChannel(5);
  mpu9250.setWire(&Wire);
  mpu9250.beginAccel();
  mpu9250.beginGyro();
  mpu9250.beginMag();

  udp.begin(remotePort);
  Serial.println("Setup complete.");
}

void loop() {
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && currentButtonState == LOW) {
    udp.beginPacket(remoteIP, remotePort);
    udp.write((const uint8_t*)"RESET", strlen("RESET"));
    udp.endPacket();
    Serial.println("RESET message sent via UDP");
    delay(200); // debounce
  }

  lastButtonState = currentButtonState;



  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 20) {
    SamplePeriod = (currentMillis - previousMillis) / 1000.0f;
    previousMillis = currentMillis;

    selectMuxChannel(5);
    mpu9250.accelUpdate();
    mpu9250.gyroUpdate();
    mpu9250.magUpdate();

    float ax = mpu9250.accelX();
    float ay = mpu9250.accelY();
    float az = mpu9250.accelZ();
    float gx = mpu9250.gyroX() * (PI / 180.0f);
    float gy = mpu9250.gyroY() * (PI / 180.0f);
    float gz = mpu9250.gyroZ() * (PI / 180.0f);
    float mx = mpu9250.magX();
    float my = mpu9250.magY();
    float mz = mpu9250.magZ();

    MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);

    char buffer[512];
    int len = 0;

    len += snprintf(buffer + len, sizeof(buffer) - len,
                    "Palm\n%.4f,%.4f,%.4f,%.4f;\n",
                    q_palm[0], q_palm[1], q_palm[2], q_palm[3]);

    for (int i = 0; i < 5; i++) {
      selectMuxChannel(i);
      uint16_t fifoCount = mpu6050[i].getFIFOCount();
      if (fifoCount >= 42) {
        mpu6050[i].getFIFOBytes(fifoBuffers[i], 42);
        mpu6050[i].dmpGetQuaternion(&quats_mpu6050[i], fifoBuffers[i]);
        mpu6050[i].resetFIFO();

        float norm = sqrtf(quats_mpu6050[i].w * quats_mpu6050[i].w +
                           quats_mpu6050[i].x * quats_mpu6050[i].x +
                           quats_mpu6050[i].y * quats_mpu6050[i].y +
                           quats_mpu6050[i].z * quats_mpu6050[i].z);
        if (norm != 0.0f) {
          quats_mpu6050[i].w /= norm;
          quats_mpu6050[i].x /= norm;
          quats_mpu6050[i].y /= norm;
          quats_mpu6050[i].z /= norm;
        }

        len += snprintf(buffer + len, sizeof(buffer) - len,
                        "Finger %d\n%.4f,%.4f,%.4f,%.4f;\n",
                        i + 1,
                        quats_mpu6050[i].w, quats_mpu6050[i].x,
                        quats_mpu6050[i].y, quats_mpu6050[i].z);
      } else {
        len += snprintf(buffer + len, sizeof(buffer) - len,
                        "Finger %d\n0.0000,0.0000,0.0000,0.0000;\n", i + 1);
      }
    }

    udp.beginPacket(remoteIP, remotePort);
    udp.write((uint8_t*)buffer, len);
    udp.endPacket();

    Serial.println(buffer);
  }
}
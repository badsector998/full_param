#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
MPU6050 mpu;

#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

int16_t ax, ay, az;
int16_t gx, gy, gz;

Quaternion q;
float euler[3];
float angles[3];

float lastKnownLoc[3];
float speedKmph;

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);

//volatile bool mpuInterrupt = false;
//void dmpDataReady() {
//  mpuInterrupt = true;
//}

bool done = true;
float yawOffset, pitchOffset, rollOffset;
void calibrateImu() {
  readAngles();
  for (int i = 0; i < 1000; i++) {
    yawOffset += angles[0];
    pitchOffset += angles[1];
    rollOffset += angles[2];
  }
  yawOffset = yawOffset / 1000;
  pitchOffset = pitchOffset / 1000;
  rollOffset = rollOffset / 1000;
  Serial.print("Yaw Offset : ");
  Serial.println(yawOffset);
  Serial.print("Pitch Offset : ");
  Serial.println(pitchOffset);
  Serial.print("Roll Offset : ");
  Serial.println(rollOffset);
  delay(5000);
  done = false;
}

void readAngles() {
  //mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    //Serial.print("Euler\t");
    angles[0] = euler[0] * 180 / M_PI;
    angles[1] = euler[1] * 180 / M_PI;
    angles[2] = euler[2] * 180 / M_PI;
  }

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  TWBR = 24;

  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);

  //  Serial.println(F("Initializing Device..."));
  //  mpu.initialize();
  //  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing Device Connection..."));
  if (mpu.testConnection()) {
    Serial.println("MPU6050 Connected");
  } else {
    Serial.println("MPU is not connected!");
    while (1);
  }

  Serial.println(F("\nSend any chars..."));
  while (Serial.available() && Serial.read());
  while (!Serial.available());
  while (Serial.available() && Serial.read());

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //-2326  295 1752  -67 17  -79

  mpu.setXGyroOffset(-67);
  mpu.setYGyroOffset(17);
  mpu.setZGyroOffset(-79);
  mpu.setZAccelOffset(1752);
  mpu.setXAccelOffset(-2326);
  mpu.setYAccelOffset(295);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    //    Serial.println(F("Enabling interrupt detection.."));
    //    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    //    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP Ready! Waiting for first interrupt"));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Init failed error code : "));
    Serial.print(devStatus);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  if (!dmpReady) return;

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        lastKnownLoc[0] = gps.location.lat();
        lastKnownLoc[1] = gps.location.lng();
        lastKnownLoc[2] = gps.altitude.meters();
        speedKmph = gps.speed.kmph();
      } else {
        Serial.println("Location invalid!");
      }
    }
  }

  if (!done) {
    calibrateImu();
  } else {
    readAngles();
  }

  Serial.print(lastKnownLoc[0], 6); Serial.print(" ");
  Serial.print(lastKnownLoc[1], 6); Serial.print(" ");
  Serial.print(lastKnownLoc[2]); Serial.print(" ");
  Serial.print(speedKmph); Serial.print(" ");
  Serial.print(angles[0] - yawOffset); Serial.print(" ");
  Serial.print(angles[1] - pitchOffset); Serial.print(" ");
  Serial.print(angles[2] - rollOffset); Serial.print(" ");
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.print(gz); Serial.print(" ");
  Serial.print("\n");
}

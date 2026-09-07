/*  ESP32-C3 + MPU6050 + VL53L0X → NimBLE batch @200Hz
 *  Units: 1=Right Leg, 2=Left Leg, 3=Torso
 *  Works with 1–3 units independently.
 */
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <Wire.h>

// ---------- CONFIG ----------
#define UNIT_ID         1
#define IMU_HZ          200
#define BATCH_SIZE      10
#define TOF_ADDR        0x29
#define IMU_ADDR        0x68
#define SDA_PIN         8
#define SCL_PIN         9
// -----------------------------

#define MPU_REG_WHO     0x75
#define MPU_REG_PWR1    0x6B
#define MPU_REG_ACCEL   0x3B
#define MPU_REG_GYRO    0x43
#define MPU_REG_SMPLRT  0x19
#define MPU_REG_DLPF    0x1A

// VL53L0X
#define VL53_ID         0x00
#define VL53_DISTANCE   0x1E
#define VL53_RANG_STAT  0x14

#define SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CTRL_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic *pChar;
NimBLECharacteristic *pControl;

volatile int16_t imuBuf[6];
volatile bool imuReady = false;
volatile uint16_t tofDist = 0;
volatile uint32_t tofTimestamp = 0;

struct Sample { uint32_t ts; int16_t ax,ay,az,gx,gy,gz; uint16_t d; };
Sample batch[BATCH_SIZE];
volatile uint8_t batchCount = 0;
volatile uint32_t seq = 0;
uint64_t epochStart = 0;

const uint32_t PERIOD_US = 1000000L / IMU_HZ;
volatile uint32_t lastSampleUs = 0;

// ---- I2C ----
void mpuWrite(uint8_t reg, uint8_t v){
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}
uint8_t mpuRead(uint8_t reg){
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.endTransmission();
  Wire.requestFrom((uint8_t)IMU_ADDR,(uint8_t)1); return Wire.read();
}
void tofWrite16(uint16_t reg, uint16_t v){
  Wire.beginTransmission(TOF_ADDR); Wire.write(reg>>8); Wire.write(reg&0xFF);
  Wire.write(v>>8); Wire.write(v&0xFF); Wire.endTransmission();
}
uint16_t tofRead16(uint16_t reg){
  Wire.beginTransmission(TOF_ADDR); Wire.write(reg>>8); Wire.write(reg&0xFF); Wire.endTransmission();
  Wire.requestFrom((uint8_t)TOF_ADDR,(uint8_t)2);
  return (Wire.read()<<8)|Wire.read();
}
void tofWrite8(uint16_t reg, uint8_t v){
  Wire.beginTransmission(TOF_ADDR); Wire.write(reg>>8); Wire.write(reg&0xFF); Wire.write(v); Wire.endTransmission();
}
uint8_t tofRead8(uint16_t reg){
  Wire.beginTransmission(TOF_ADDR); Wire.write(reg>>8); Wire.write(reg&0xFF); Wire.endTransmission();
  Wire.requestFrom((uint8_t)TOF_ADDR,(uint8_t)1); return Wire.read();
}

// ---- VL53L0X init (verified minimal ST sequence) ----
void initTOF(){
  uint8_t id = tofRead8(VL53_ID);
  if(id != 0xEE){ Serial.printf("VL53L0X id error: %02X\n", id); return; }

  tofWrite8(0x80, 0x01);       // reset
  delay(1);
  tofWrite8(0x88, 0x01);       // I2C standard mode
  delay(1);
  tofWrite8(0xFF, 0x01); tofWrite8(0x00, 0x00); tofWrite8(0xFF, 0x00);
  tofWrite8(0xC0, 0x01);       // OS cal
  delay(10);
  tofWrite8(0x00, 0x40);       // start ranging
  delay(2);
  Serial.println("VL53L0X init OK");
}

void initIMU(){
  Wire.beginTransmission(IMU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  delay(10);
  mpuWrite(MPU_REG_DLPF, 0x01);
  mpuWrite(MPU_REG_SMPLRT, 0);
  mpuWrite(0x1C, 0x18);
  mpuWrite(0x1B, 0x18);
}

// ---- BLE ----
class CtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c){
    std::string s = c->getValue();
    if(s.size() >= 4){
      uint32_t phoneEpoch = 0;
      memcpy(&phoneEpoch, s.data(), 4);
      epochStart = (uint64_t)phoneEpoch * 1000ULL - (uint64_t)millis();
    }
  }
};
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s){}
  void onDisconnect(NimBLEServer* s){ NimBLEDevice::startAdvertising(); }
};

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  initIMU();
  initTOF();

  NimBLEDevice::init(std::string("ESP32C3_Unit") + std::to_string(UNIT_ID));
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService *pSvc = pServer->createService(SERVICE_UUID);
  pChar = pSvc->createCharacteristic(CHAR_UUID,
              NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  pControl = pSvc->createCharacteristic(CTRL_UUID, NIMBLE_PROPERTY::WRITE);
  pControl->setCallbacks(new CtrlCallbacks());

  pSvc->start();
  NimBLEDevice::startAdvertising();

  lastSampleUs = micros();
  Serial.printf("Unit %d ready\n", UNIT_ID);
}

void loop(){
  uint32_t now = micros();
  if((int32_t)(now - lastSampleUs) >= (int32_t)PERIOD_US){
    lastSampleUs += PERIOD_US;

    Wire.beginTransmission(IMU_ADDR); Wire.write(MPU_REG_ACCEL); Wire.endTransmission();
    Wire.requestFrom((uint8_t)IMU_ADDR,(uint8_t)12);
    for(int i=0;i<6;i++){
      uint8_t hi=Wire.read(), lo=Wire.read();
      imuBuf[i] = (int16_t)((hi<<8)|lo);
    }

    if((seq % 4) == 0){
      Wire.beginTransmission(TOF_ADDR); Wire.write(0x00); Wire.write(VL53_DISTANCE); Wire.endTransmission();
      Wire.requestFrom((uint8_t)TOF_ADDR,(uint8_t)2);
      if(Wire.available()>=2){
        tofDist = (Wire.read()<<8)|Wire.read();
        tofTimestamp = millis();
      }
    }

    uint16_t d = (millis() - tofTimestamp < 50) ? tofDist : 0;

    if(batchCount < BATCH_SIZE){
      batch[batchCount++] = {millis(), imuBuf[0],imuBuf[1],imuBuf[2],
                                      imuBuf[3],imuBuf[4],imuBuf[5], d};
    }

    if(batchCount >= BATCH_SIZE){
      uint8_t buf[1 + 2 + 4 + BATCH_SIZE*18];  // 187 bytes
      buf[0] = UNIT_ID;
      buf[1] = (seq>>8)&0xFF; buf[2] = seq&0xFF;
      uint32_t netTs = (uint32_t)((millis() + epochStart)/1000);
      memcpy(&buf[3], &netTs, 4);
      int off = 7;
      for(int i=0;i<BATCH_SIZE;i++){
        memcpy(&buf[off], &batch[i], 18); off += 18;
      }
      if(pChar->notify()) seq += BATCH_SIZE;
      batchCount = 0;
    }
  }
  yield();  // yield to BLE task instead of light sleep
}

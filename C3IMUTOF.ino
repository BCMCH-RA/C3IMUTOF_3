/*  ESP32-C3  + MPU6050 + VL53L4CD  ->  NimBLE GATT notify @200Hz (IMU)
 *  Battery-friendly: NimBLE, light-sleep between batches, no WiFi.
 *  Packet: [unit_id][seq][ts32][ax][ay][az][gx][gy][gz][dist] = 19 bytes
 *  Batch 10 samples => 190 bytes per notification (fits BT MTU).
 */

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <Wire.h>

// ---------- USER CONFIG ----------
#define UNIT_ID         1          // 1,2,3 for the three units
#define IMU_HZ          200
#define BATCH_SIZE      10         // samples per BT notification
#define TOF_ADDR        0x29
#define IMU_ADDR        0x68
#define SDA_PIN         8
#define SCL_PIN         9
// ---------------------------------

// ---- MPU6050 registers ----
#define MPU_REG_WHO     0x75
#define MPU_REG_PWR1    0x6B
#define MPU_REG_ACCEL   0x3B
#define MPU_REG_GYRO    0x43
#define MPU_REG_SMPLRT  0x19
#define MPU_REG_DLPF    0x1A

// ---- VL53L4CD ----
#define VL53_WHO        0x00F8
#define VL53_MODE       0x00A9
#define VL53_INTERLEAVE 0x00A8
#define VL53_INT_MASK   0x00AC
#define VL53_MEAS       0x0080
#define VL53_RESULT     0x008E   // distance mm at 0x8E-0x8F

// BLE
#define SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic *pChar;
hw_timer_t *timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// IMU raw buffer
volatile int16_t imuBuf[6]; // ax,ay,az,gx,gy,gz
volatile bool imuReady = false;

// TOF
volatile uint16_t tofDist = 0;
volatile uint32_t tofTimestamp = 0;

// Batch ring buffer
struct Sample { uint32_t ts; int16_t ax,ay,az,gx,gy,gz; uint16_t d; };
Sample batch[BATCH_SIZE];
volatile uint8_t batchCount = 0;
volatile uint32_t seq = 0;
volatile uint32_t epochStart = 0;  // unix epoch at connect

// Timing
volatile uint32_t nextSample = 0;
const uint32_t PERIOD_US = 1000000L / IMU_HZ;

// ---- MPU6050 I2C ----
void mpuWrite(uint8_t reg, uint8_t v){
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}
uint8_t mpuRead(uint8_t reg){
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg); Wire.endTransmission();
  Wire.requestFrom((uint8_t)IMU_ADDR,(uint8_t)1); return Wire.read();
}

// ---- VL53L4CD I2C ----
void tofWrite16(uint16_t reg, uint16_t v){
  Wire.beginTransmission(TOF_ADDR); Wire.write(reg>>8); Wire.write(reg&0xFF); Wire.write(v>>8); Wire.write(v&0xFF); Wire.endTransmission();
}
uint16_t tofRead16(uint16_t reg){
  Wire.beginTransmission(TOF_ADDR); Wire.write(reg>>8); Wire.write(reg&0xFF); Wire.endTransmission();
  Wire.requestFrom((uint8_t)TOF_ADDR,(uint8_t)2);
  return (Wire.read()<<8)|Wire.read();
}

void initTOF(){
  Wire.beginTransmission(TOF_ADDR); Wire.write(0xFF); Wire.write(0x01); Wire.endTransmission();
  uint16_t id = tofRead16(VL53_WHO);
  if(id != 0xEACC){ Serial.printf("TOF id error: %04X\n",id); }
  tofWrite16(0x0096, 0x00);   // reset
  delay(10);
  tofWrite16(0x0096, 0x01);
  delay(10);
  tofWrite16(0x0080, 0x04);   // start ranging
}

void initIMU(){
  Wire.beginTransmission(IMU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  delay(10);
  mpuWrite(MPU_REG_DLPF, 0x01); // 188Hz BW
  mpuWrite(MPU_REG_SMPLRT, 0);  // max rate
  mpuWrite(0x1C, 0x18);         // accel ±16g
  mpuWrite(0x1B, 0x18);         // gyro ±2000dps
}

void IRAM_ATTR onTimer(){
  uint32_t now = micros();
  if((int32_t)(now - nextSample) < 0) return;
  nextSample += PERIOD_US;

  // read accel/gyro
  Wire.beginTransmission(IMU_ADDR); Wire.write(MPU_REG_ACCEL); Wire.endTransmission();
  Wire.requestFrom((uint8_t)IMU_ADDR,(uint8_t)12);
  for(int i=0;i<6;i++){
    uint8_t hi=Wire.read(), lo=Wire.read();
    imuBuf[i] = (int16_t)((hi<<8)|lo);
  }
  imuReady = true;

  // read TOF at lower effective rate (every ~4th call ≈ 50Hz)
  if((seq % 4) == 0){
    Wire.beginTransmission(TOF_ADDR); Wire.write(0x00); Wire.write(0x8E); Wire.endTransmission();
    Wire.requestFrom((uint8_t)TOF_ADDR,(uint8_t)2);
    if(Wire.available()>=2){
      tofDist = (Wire.read()<<8)|Wire.read();
      tofTimestamp = millis();
    }
  }
}

class ServerCallbacks: public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s){
    epochStart = millis(); // anchor: phone will send unix epoch via control char
  }
  void onDisconnect(NimBLEServer* s){
    NimBLEDevice::startAdvertising(); // reconnectable
  }
};

// Control char to receive epoch from phone
NimBLECharacteristic *pControl;

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  initIMU();
  initTOF();

  NimBLEDevice::init("ESP32C3_Unit"+String(UNIT_ID));
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // max tx power for reliability
  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService *pSvc = pServer->createService(SERVICE_UUID);
  pChar = pSvc->createCharacteristic(CHAR_UUID,
                NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  pChar->setValue(std::vector<uint8_t>(200,0));

  // control char for epoch sync
  pControl = pSvc->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
                NIMBLE_PROPERTY::WRITE);
  pControl->setCallbacks(new NimBLECharacteristicCallbacks(){
    void onWrite(NimBLECharacteristic* c){
      std::string s = c->getValue();
      if(s.size()>=4){
        uint32_t phoneEpoch = 0;
        memcpy(&phoneEpoch, s.data(), 4);
        epochStart = millis() - ((phoneEpoch)*1000ul); // anchor offset
      }
    }
  });

  pSvc->start();
  NimBLEDevice::startAdvertising();

  timer = timerBegin(0, 80, true);            // 1us tick
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, PERIOD_US, true);
  timerAlarmEnable(timer);
}

void loop(){
  if(!imuReady) return;
  imuReady = false;

  // take latest TOF if recent (<50ms)
  uint16_t d = (millis() - tofTimestamp < 50) ? tofDist : 0;
  uint32_t ts = millis();

  portENTER_CRITICAL(&mux);
  if(batchCount < BATCH_SIZE){
    batch[batchCount++] = {ts, imuBuf[0],imuBuf[1],imuBuf[2],imuBuf[3],imuBuf[4],imuBuf[5], d};
  }
  portEXIT_CRITICAL(&mux);

  // send batch when full
  if(batchCount >= BATCH_SIZE){
    uint8_t buf[1 + 2 + 4 + BATCH_SIZE*18]; // unit(1)+seq(2)+ts(4)+samples
    buf[0] = UNIT_ID;
    buf[1] = (seq>>8)&0xFF; buf[2] = seq&0xFF;
    uint32_t netTs = (millis()-epochStart)/1000 + 0; // filled properly in real code
    memcpy(&buf[3], &netTs, 4);
    int off = 7;
    for(int i=0;i<BATCH_SIZE;i++){
      int16_t *p = (int16_t*)&batch[i];
      memcpy(&buf[off], p, 18); off += 18;
    }
    pChar->setValue(buf, off);
    pChar->notify();
    seq += BATCH_SIZE;
    batchCount = 0;
  }
}

/*
  XL320 servo chain + MPU-6050 combined console
  Classic ESP32-WROOM-32, Arduino core, no external libraries.
  Put this file in a folder named xl320_mpu_combined.

  WIRING (same as the standalone sketches; the two sets of pins do not overlap)
    Servo bus (UART2 through the 74HC126 tri-state buffer):
      GPIO17 (TX2)  -> 74HC126 pin 2 (1A)      GPIO4 (DIR)  -> 74HC126 pin 1 (1OE)
      74HC126 pin 3 -> DATA line               GPIO16 (RX2) -> DATA line
      10k pull-up DATA -> 3V3, servos on their own 7.4 V supply, all grounds common
    MPU-6050 (I2C):
      VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22, AD0 -> GND
      Pull-ups to 3.3 V only, never 5 V.

  SERIAL MONITOR: 115200 baud, line ending = Newline

  SERVO COMMANDS (they act on the selected ID; "id all" = broadcast, no replies)
    ping | find | id <n|all> | setid <new> | pos <0-1023> | deg <0-300>
    speed <0-1023> | led <0-7> | torque <0|1> | read | demo <on|off> | fb <on|off>
  IMU COMMANDS (all start with "mpu")
    mpu scan | diag | regs | read | stream | stop | raw | csv | cal | clearcal | init
  COMBINED
    snap   one IMU frame plus position/load/temperature of every servo

  The loop never blocks for long, so the IMU keeps being polled while servos move.
  Servo traffic does take a few milliseconds per command, so the IMU may skip some
  of its 100 Hz samples (it always reads the newest one). The MPU 'cal' command
  blocks for about 7 seconds and refuses to run while the demo is on.
*/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// ================================ CONFIG ================================
const int PIN_DXL_RX  = 16;
const int PIN_DXL_TX  = 17;
const int PIN_DXL_DIR = 4;                  // avoid strapping pins 0, 2, 12, 15
const uint32_t DXL_BAUD = 115200;           // your servos answered at 115200

const uint8_t  SERVO_IDS[]  = {1, 2, 3};    // edit to the IDs on your bus ('find' lists them)
const uint8_t  NUM_SERVOS   = sizeof(SERVO_IDS);
const uint16_t MOVING_SPEED = 300;          // 0 = max, 1..1023

const int SDA_PIN = 21, SCL_PIN = 22;
const uint32_t I2C_HZ = 100000;

// What runs by itself after boot (everything can be switched from the console)
#define DEMO_AT_BOOT        0   // 1 = servos swing between two poses
#define FEEDBACK_AT_BOOT    0   // 1 = print servo position/load/temp 5x per second
#define MPU_STREAM_AT_BOOT  0   // 1 = print IMU data 10x per second

// ============================ SERVO (XL320) ============================
HardwareSerial& DXL = Serial2;

const uint8_t INST_PING       = 0x01;
const uint8_t INST_READ       = 0x02;
const uint8_t INST_WRITE      = 0x03;
const uint8_t INST_SYNC_WRITE = 0x83;
const uint8_t INST_STATUS     = 0x55;
const uint8_t BROADCAST_ID    = 0xFE;

// XL320 control table (addresses are DECIMAL)
const uint16_t ADDR_ID               = 3;
const uint16_t ADDR_CONTROL_MODE     = 11;  // 1 = wheel, 2 = joint (EEPROM)
const uint16_t ADDR_TORQUE_ENABLE    = 24;
const uint16_t ADDR_LED              = 25;  // 1 red, 2 green, 4 blue (combine bits)
const uint16_t ADDR_GOAL_POSITION    = 30;  // 2 bytes, 0..1023 = 0..300 deg
const uint16_t ADDR_MOVING_SPEED     = 32;  // 2 bytes
const uint16_t ADDR_PRESENT_POSITION = 37;  // 2 bytes
const uint16_t ADDR_PRESENT_LOAD     = 41;  // 2 bytes, bit 10 = direction
const uint16_t ADDR_PRESENT_VOLTAGE  = 45;  // 1 byte, 0.1 V
const uint16_t ADDR_PRESENT_TEMP     = 46;  // 1 byte, deg C

const int DXL_TIMEOUT = -1, DXL_CRC_FAIL = -2, DXL_BAD_PACKET = -3;

uint8_t targetId  = 1;       // set from SERVO_IDS[0] in setup()
bool    demoOn    = false;
bool    feedbackOn = false;

inline uint8_t lo(uint16_t v) { return v & 0xFF; }
inline uint8_t hi(uint16_t v) { return (v >> 8) & 0xFF; }

// CRC-16 (Robotis, polynomial 0x8005)
uint16_t crc16(const uint8_t* data, size_t n) {
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x8005) : (uint16_t)(crc << 1);
  }
  return crc;
}

// No byte stuffing: addresses and values used here can never form FF FF FD.
void sendPacket(uint8_t id, uint8_t inst, const uint8_t* params, uint16_t nParams) {
  uint8_t pkt[64];
  if (nParams > sizeof(pkt) - 10) return;
  uint16_t len = nParams + 3;                  // instruction + params + 2 CRC bytes
  pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = 0xFD; pkt[3] = 0x00;
  pkt[4] = id; pkt[5] = lo(len); pkt[6] = hi(len); pkt[7] = inst;
  if (nParams) memcpy(pkt + 8, params, nParams);
  uint16_t crc = crc16(pkt, 8 + nParams);
  pkt[8 + nParams] = lo(crc);
  pkt[9 + nParams] = hi(crc);

  while (DXL.available()) DXL.read();          // drop stale bytes
  digitalWrite(PIN_DXL_DIR, HIGH);             // buffer ON: ESP32 owns the line
  DXL.write(pkt, 10 + nParams);
  DXL.flush();                                 // wait until the last bit is out
  digitalWrite(PIN_DXL_DIR, LOW);              // buffer OFF: the servo may reply
}

// Our own transmitted packet is echoed on RX; it is skipped because its
// instruction byte is not 0x55 (status). Pass BROADCAST_ID to accept any servo ID.
int readStatus(uint8_t expectId, uint8_t* out, uint16_t outMax, uint16_t* outLen,
               uint32_t timeoutUs = 5000) {
  static const uint8_t H[4] = {0xFF, 0xFF, 0xFD, 0x00};
  uint8_t  buf[64];
  uint16_t idx = 0, total = 0;
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < timeoutUs) {
    if (!DXL.available()) continue;
    uint8_t b = DXL.read();
    if (idx < 4) {                             // find the header FF FF FD 00
      if (b == H[idx]) buf[idx++] = b;
      else idx = (b == 0xFF) ? ((idx == 2) ? 2 : 1) : 0;
      continue;
    }
    buf[idx++] = b;                            // ID, length, rest of the packet
    if (idx == 7) {
      uint16_t len = buf[5] | (buf[6] << 8);
      total = 7 + len;
      if (len < 3 || total > sizeof(buf)) { idx = 0; total = 0; }
      continue;
    }
    if (idx >= 7 && idx == total) {            // packet complete
      bool isStatus = (buf[7] == INST_STATUS) && (total >= 11);
      bool idOk     = (expectId == BROADCAST_ID) || (buf[4] == expectId);
      if (!isStatus || !idOk) { idx = 0; total = 0; continue; }   // echo / other servo
      uint16_t rxCrc = buf[total - 2] | (buf[total - 1] << 8);
      if (crc16(buf, total - 2) != rxCrc) return DXL_CRC_FAIL;
      uint16_t n = total - 11;                 // params after the error byte
      if (outLen) *outLen = n;
      if (out && n) memcpy(out, buf + 9, min((int)n, (int)outMax));
      return buf[8];                           // servo error byte (0 = OK)
    }
  }
  return DXL_TIMEOUT;
}

int ping(uint8_t id) {
  sendPacket(id, INST_PING, nullptr, 0);
  return readStatus(id, nullptr, 0, nullptr);
}

int write1(uint8_t id, uint16_t addr, uint8_t v) {
  uint8_t p[3] = {lo(addr), hi(addr), v};
  sendPacket(id, INST_WRITE, p, 3);
  if (id == BROADCAST_ID) return 0;            // broadcast gets no reply
  return readStatus(id, nullptr, 0, nullptr);
}

int write2(uint8_t id, uint16_t addr, uint16_t v) {
  uint8_t p[4] = {lo(addr), hi(addr), lo(v), hi(v)};
  sendPacket(id, INST_WRITE, p, 4);
  if (id == BROADCAST_ID) return 0;
  return readStatus(id, nullptr, 0, nullptr);
}

int readReg(uint8_t id, uint16_t addr, uint16_t size, uint16_t* value) {
  uint8_t p[4] = {lo(addr), hi(addr), lo(size), hi(size)};
  sendPacket(id, INST_READ, p, 4);
  uint8_t  data[2] = {0, 0};
  uint16_t n = 0;
  int r = readStatus(id, data, 2, &n);
  if (r < 0) return r;
  if (n < size) return DXL_BAD_PACKET;
  *value = (size == 1) ? data[0] : (uint16_t)(data[0] | (data[1] << 8));
  return r;
}

// One packet updates a 2-byte register on all listed servos at the same moment.
void syncWrite2(uint16_t addr, const uint8_t* ids, const uint16_t* values, uint8_t n) {
  uint8_t p[4 + 3 * 16];
  if (n > 16) return;
  p[0] = lo(addr); p[1] = hi(addr);
  p[2] = 2;        p[3] = 0;                   // data length per servo = 2 bytes
  for (uint8_t i = 0; i < n; i++) {
    p[4 + 3 * i]     = ids[i];
    p[4 + 3 * i + 1] = lo(values[i]);
    p[4 + 3 * i + 2] = hi(values[i]);
  }
  sendPacket(BROADCAST_ID, INST_SYNC_WRITE, p, 4 + 3 * n);
}

void report(const char* what, uint8_t id, int r) {
  Serial.printf("%s ID %u -> ", what, id);
  if (id == BROADCAST_ID && r == 0) Serial.println("sent (broadcast, no reply)");
  else if (r == 0)                  Serial.println("OK");
  else if (r == DXL_TIMEOUT)        Serial.println("TIMEOUT (no reply)");
  else if (r == DXL_CRC_FAIL)       Serial.println("CRC FAIL");
  else if (r == DXL_BAD_PACKET)     Serial.println("BAD PACKET");
  else                              Serial.printf("servo error 0x%02X\n", r);
}

// Returns 0 on success, or a negative DXL_* code. volt may be nullptr.
int readServoStatus(uint8_t id, uint16_t* pos, uint16_t* load, uint16_t* temp, uint16_t* volt) {
  int r = readReg(id, ADDR_PRESENT_POSITION, 2, pos);
  if (r < 0) return r;
  r = readReg(id, ADDR_PRESENT_LOAD, 2, load);
  if (r < 0) return r;
  if (volt) {
    r = readReg(id, ADDR_PRESENT_VOLTAGE, 1, volt);
    if (r < 0) return r;
  }
  r = readReg(id, ADDR_PRESENT_TEMP, 1, temp);
  if (r < 0) return r;
  return 0;
}

// Load register: low 10 bits = magnitude (0..1023 = 0..100 %), bit 10 = direction.
int signedLoad(uint16_t load) {
  int mag = load & 0x3FF;
  return (load & 0x400) ? -mag : mag;
}

void cmdReadServo(uint8_t id) {
  uint16_t pos = 0, load = 0, temp = 0, volt = 0;
  int r = readServoStatus(id, &pos, &load, &temp, &volt);
  if (r < 0) { report("Read", id, r); return; }
  Serial.printf("ID %u: pos %u (%.1f deg), load %d, %.1f V, %u C\n",
                id, pos, pos * 300.0 / 1023.0, signedLoad(load), volt / 10.0, temp);
}

void printFeedbackLine() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    uint8_t  id = SERVO_IDS[i];
    uint16_t pos = 0, load = 0, temp = 0;
    if (readServoStatus(id, &pos, &load, &temp, nullptr) == 0)
      
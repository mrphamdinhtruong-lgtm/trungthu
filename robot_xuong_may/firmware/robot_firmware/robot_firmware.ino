/*
 * =====================================================================
 *  ROBOT THU GOM RÁC / VẢI VỤN XƯỞNG MAY – FIRMWARE ARDUINO UNO R3
 * =====================================================================
 *  Nhiệm vụ: nhận tốc độ đặt 4 bánh mecanum từ Raspberry Pi 4 qua USB,
 *            điều khiển tốc độ từng động cơ JGB37-520 bằng SAC + DOB,
 *            gửi tốc độ đo / điện áp pin / trạng thái về Pi.
 *
 *  KHÔNG mở file này bằng Arduino IDE để sửa thông số!
 *  -> Sửa robot/config.py rồi chạy:  python flash_arduino.py
 *     (Python tự sinh config_gen.h, biên dịch và nạp bằng arduino-cli)
 *
 *  ---------------------- SƠ ĐỒ CHÂN ---------------------------------
 *   D0/D1 : USB-Serial tới Raspberry Pi (115200 8N1)
 *   D2    : Encoder kênh A – bánh FL (INT0)
 *   D3    : Encoder kênh A – bánh FR (INT1)
 *   A2    : Encoder kênh A – bánh RL (PCINT10)
 *   A3    : Encoder kênh A – bánh RR (PCINT11)
 *   L298N #1 (trước): ENA=D5  IN1=D4  IN2=D7  (FL) | ENB=D6  IN3=D8  IN4=D11 (FR)
 *   L298N #2 (sau)  : ENA=D9  IN1=D12 IN2=D13 (RL) | ENB=D10 IN3=A0  IN4=A1  (RR)
 *   A4    : Nút dừng khẩn NC về GND (tùy chọn, ESTOP_ENABLE)
 *   A5    : Đo áp pin 3S qua cầu phân áp 10k/4.7k (tùy chọn, BATT_ENABLE)
 *
 *  ---------------------- GIAO THỨC (ASCII, kết thúc '\n') ------------
 *   Pi -> Arduino
 *     W,fl,fr,rl,rr   tốc độ đặt 4 bánh (RPM, có dấu)  – cũng là tín hiệu "còn sống"
 *     O,fl,fr,rl,rr   chạy vòng hở duty -1..1 (chỉ để thử nghiệm / nhận dạng)
 *     S               dừng (tốc độ đặt = 0)
 *     E,0 | E,1       xóa / kích hoạt dừng khẩn phần mềm
 *     C,m             chọn bộ điều khiển: 0=PI 1=SAC 2=SAC+DOB
 *     R               đặt lại độ lợi thích nghi về K0
 *     P,ten,giatri    chỉnh tham số khi đang chạy (xem setParam), vd: P,gie,8
 *     Q               in toàn bộ tham số
 *     D,i             gửi dữ liệu debug chi tiết bánh i mỗi chu kỳ (-1 = tắt)
 *     ?               thông tin firmware
 *   Arduino -> Pi
 *     I,ROBOT_FW,ver,hash,Ts_ms,mode
 *     T,ms,rpm0..3,u0..3,flags,mode,vbat      (mỗi TELEMETRY_PERIOD_MS)
 *        flags: bit0 = dừng khẩn, bit1 = mất lệnh (timeout), bit2 = vòng hở
 *     X,ms,i,ref_rpm,ym_rpm,y_rpm,u,dhat,Ke,Kx,Ku   (khi bật D,i)
 *     A,OK,... | A,ERR,...   phản hồi lệnh
 *     P,ten,giatri           (trả lời lệnh Q)
 * =====================================================================
 */
#include <Arduino.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <string.h>
#include "config_gen.h"
#include "sac_dob.h"

#define FW_VERSION "1.0.0"
#define NUM_MOTORS 4

// ------------------------- Chân -------------------------------------
static const uint8_t PIN_EN[NUM_MOTORS]  = {5, 6, 9, 10};
static const uint8_t PIN_IN1[NUM_MOTORS] = {4, 8, 12, A0};
static const uint8_t PIN_IN2[NUM_MOTORS] = {7, 11, 13, A1};
static const bool MOTOR_INV[NUM_MOTORS]  = {MOTOR_INVERT_FL, MOTOR_INVERT_FR,
                                            MOTOR_INVERT_RL, MOTOR_INVERT_RR};
#define PIN_ENC_FL 2
#define PIN_ENC_FR 3
#define PIN_ENC_RL A2   // PC2 – PCINT10
#define PIN_ENC_RR A3   // PC3 – PCINT11
#define PIN_ESTOP  A4
#define PIN_VBAT   A5

// ------------------------- Hằng số dẫn xuất -------------------------
#define CTRL_PERIOD_US   ((uint32_t)CONTROL_PERIOD_MS * 1000UL)
#define CTRL_TS          ((float)CONTROL_PERIOD_MS / 1000.0f)
#define INV_RPM_NOM      (1.0f / RPM_NOMINAL)
#define IDLE_STEPS       5
#define FLAG_ESTOP       0x01
#define FLAG_TIMEOUT     0x02
#define FLAG_OPENLOOP    0x04

// ------------------------- Encoder (ngắt) ---------------------------
volatile int32_t encCount[NUM_MOTORS] = {0, 0, 0, 0};
volatile int8_t encDir[NUM_MOTORS] = {1, 1, 1, 1};   // dấu đếm (encoder chỉ 1 kênh)
volatile uint8_t lastPinC = 0;

void isrEncFL() { encCount[0] += encDir[0]; }
void isrEncFR() { encCount[1] += encDir[1]; }

ISR(PCINT1_vect) {
  uint8_t pins = PINC;
  uint8_t changed = (pins ^ lastPinC) & 0x0C;   // chỉ PC2, PC3
  lastPinC = pins;
  if (changed & 0x04) encCount[2] += encDir[2];
  if (changed & 0x08) encCount[3] += encDir[3];
}

// ------------------------- Biến toàn cục ----------------------------
SacDobParams P;
SacDobCtrl ctrl[NUM_MOTORS];

float targetRpm[NUM_MOTORS] = {0, 0, 0, 0};   // tốc độ đặt từ Pi
float refN[NUM_MOTORS] = {0, 0, 0, 0};        // tốc độ đặt sau giới hạn gia tốc (chuẩn hóa)
float speedN[NUM_MOTORS] = {0, 0, 0, 0};      // tốc độ đo đã lọc (chuẩn hóa)
float uOut[NUM_MOTORS] = {0, 0, 0, 0};        // tín hiệu điều khiển -1..1
float openDuty[NUM_MOTORS] = {0, 0, 0, 0};
int32_t lastCnt[NUM_MOTORS] = {0, 0, 0, 0};
uint8_t idleCnt[NUM_MOTORS] = {0, 0, 0, 0};

float accelLimitN = ACCEL_LIMIT_RPM_S / RPM_NOMINAL;   // (1/s) chỉnh được bằng P,acc
float speedLpf = SPEED_LPF_ALPHA;
float pwmDeadzone = PWM_DEADZONE;                      // chỉnh được bằng P,dz (đo vùng chết)

bool openLoop = false;
bool estopLatched = false;
bool cmdTimeout = true;
int8_t debugMotor = -1;
float vbat = 0.0f;

uint32_t lastCmdMs = 0;
uint32_t lastTelMs = 0;
uint32_t nextCtrlUs = 0;
uint32_t lastStepUs = 0;

char rxBuf[64];
uint8_t rxLen = 0;

// =====================================================================
//  THAM SỐ
// =====================================================================
void loadDefaultParams() {
  P.Ts = CTRL_TS;
  P.tau_m = SAC_TAU_M;
  P.gi[0] = SAC_GI_E;   P.gi[1] = SAC_GI_X;   P.gi[2] = SAC_GI_U;
  P.gp[0] = SAC_GP_E;   P.gp[1] = SAC_GP_X;   P.gp[2] = SAC_GP_U;
  P.sigma = SAC_SIGMA;
  P.k0[0] = SAC_K0_E;   P.k0[1] = SAC_K0_X;   P.k0[2] = SAC_K0_U;
  P.kmin[0] = SAC_KMIN_E; P.kmin[1] = SAC_KMIN_X; P.kmin[2] = SAC_KMIN_U;
  P.kmax[0] = SAC_KMAX_E; P.kmax[1] = SAC_KMAX_X; P.kmax[2] = SAC_KMAX_U;
  P.e_db = SAC_E_DEADBAND;
  P.kn = DOB_KN;
  P.tau_n = DOB_TAU_N;
  P.tau_q = DOB_TAU_Q;
  P.d_lim = DOB_LIMIT;
  P.pi_kp = PI_KP;
  P.pi_ki = PI_KI;
  P.mode = CTRL_MODE_DEFAULT;
  sacdob_prepare(&P);
}

#define PARAM(nm, field)                         \
  if (!strcmp_P(name, PSTR(nm))) {               \
    field = v;                                   \
    ok = true;                                   \
  }

// P,ten,giatri – chỉnh tham số tạm thời (mất khi reset, muốn lưu thì sửa config.py)
void setParam(char* s) {
  char* comma = strchr(s, ',');
  if (!comma) { Serial.println(F("A,ERR,P")); return; }
  *comma = 0;
  const char* name = s;
  char* end;
  float v = (float)strtod(comma + 1, &end);
  if (end == comma + 1) { Serial.println(F("A,ERR,P")); return; }
  bool ok = false;
  PARAM("taum", P.tau_m)
  PARAM("gie", P.gi[0])
  PARAM("gix", P.gi[1])
  PARAM("giu", P.gi[2])
  PARAM("gpe", P.gp[0])
  PARAM("gpx", P.gp[1])
  PARAM("gpu", P.gp[2])
  PARAM("sig", P.sigma)
  PARAM("k0e", P.k0[0])
  PARAM("k0x", P.k0[1])
  PARAM("k0u", P.k0[2])
  PARAM("edb", P.e_db)
  PARAM("kn", P.kn)
  PARAM("taun", P.tau_n)
  PARAM("tauq", P.tau_q)
  PARAM("dlim", P.d_lim)
  PARAM("pikp", P.pi_kp)
  PARAM("piki", P.pi_ki)
  PARAM("acc", accelLimitN)
  PARAM("lpf", speedLpf)
  PARAM("dz", pwmDeadzone)
  if (!ok) { Serial.print(F("A,ERR,PNAME,")); Serial.println(name); return; }
  sacdob_prepare(&P);
  speedLpf = sd_clamp(speedLpf, 0.05f, 1.0f);
  if (accelLimitN < 0.1f) accelLimitN = 0.1f;
  pwmDeadzone = sd_clamp(pwmDeadzone, 0.0f, 200.0f);
  Serial.print(F("A,OK,P,")); Serial.print(name); Serial.print(',');
  Serial.println(v, 4);
}

void printParam(const __FlashStringHelper* name, float v) {
  Serial.print(F("P,")); Serial.print(name); Serial.print(','); Serial.println(v, 4);
}

void printParams() {
  printParam(F("taum"), P.tau_m);
  printParam(F("gie"), P.gi[0]);  printParam(F("gix"), P.gi[1]);  printParam(F("giu"), P.gi[2]);
  printParam(F("gpe"), P.gp[0]);  printParam(F("gpx"), P.gp[1]);  printParam(F("gpu"), P.gp[2]);
  printParam(F("sig"), P.sigma);
  printParam(F("k0e"), P.k0[0]);  printParam(F("k0x"), P.k0[1]);  printParam(F("k0u"), P.k0[2]);
  printParam(F("edb"), P.e_db);
  printParam(F("kn"), P.kn);      printParam(F("taun"), P.tau_n); printParam(F("tauq"), P.tau_q);
  printParam(F("dlim"), P.d_lim);
  printParam(F("pikp"), P.pi_kp); printParam(F("piki"), P.pi_ki);
  printParam(F("acc"), accelLimitN);
  printParam(F("lpf"), speedLpf);
  printParam(F("dz"), pwmDeadzone);
  printParam(F("mode"), (float)P.mode);
  Serial.println(F("A,OK,Q"));
}

void printInfo() {
  Serial.print(F("I,ROBOT_FW," FW_VERSION ","));
  Serial.print(F(FW_CONFIG_HASH));
  Serial.print(',');
  Serial.print(CONTROL_PERIOD_MS);
  Serial.print(',');
  Serial.println(P.mode);
}

// =====================================================================
//  ĐỘNG CƠ
// =====================================================================
/* Xuất u (-1..1) ra L298N, có bù vùng chết PWM. u > 0 = bánh quay tiến */
void setMotor(uint8_t i, float u) {
  if (MOTOR_INV[i]) u = -u;
  float a = fabsf(u);
  if (a < 0.002f) {                         // thả trôi
    analogWrite(PIN_EN[i], 0);
    digitalWrite(PIN_IN1[i], LOW);
    digitalWrite(PIN_IN2[i], LOW);
    return;
  }
  const float A_LIN = 0.03f;                // vùng chuyển tiếp mượt quanh 0
  float pwmF;
  if (a < A_LIN) pwmF = a / A_LIN * (pwmDeadzone + (255.0f - pwmDeadzone) * A_LIN);
  else pwmF = pwmDeadzone + (255.0f - pwmDeadzone) * a;
  int pwm = (int)(pwmF + 0.5f);
  if (pwm > 255) pwm = 255;
  if (u > 0) {
    digitalWrite(PIN_IN1[i], HIGH);
    digitalWrite(PIN_IN2[i], LOW);
  } else {
    digitalWrite(PIN_IN1[i], LOW);
    digitalWrite(PIN_IN2[i], HIGH);
  }
  analogWrite(PIN_EN[i], pwm);
}

/* Phanh nhanh L298N: IN1 = IN2 = HIGH, EN = HIGH */
void brakeAll() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(PIN_IN1[i], HIGH);
    digitalWrite(PIN_IN2[i], HIGH);
    analogWrite(PIN_EN[i], 255);
  }
}

/* Encoder 1 kênh không biết chiều quay -> lấy chiều theo lệnh,
   nhưng CHỈ đổi chiều khi bánh đã gần dừng (tránh đếm sai lúc đảo chiều). */
void updateEncoderDir(uint8_t i, float cmd) {
  int8_t want = (cmd > 0.001f) ? 1 : ((cmd < -0.001f) ? -1 : 0);
  if (want != 0 && want != encDir[i] &&
      fabsf(speedN[i]) * RPM_NOMINAL < DIR_SWITCH_RPM) {
    encDir[i] = want;   // ghi 1 byte – nguyên tử trên AVR
  }
}

// =====================================================================
//  VÒNG ĐIỀU KHIỂN (CONTROL_PERIOD_MS)
// =====================================================================
void controlStep() {
  uint32_t nowUs = micros();
  float dt = (float)(nowUs - lastStepUs) * 1e-6f;
  lastStepUs = nowUs;
  if (dt < 0.25f * CTRL_TS || dt > 4.0f * CTRL_TS) dt = CTRL_TS;

  // 1) Đọc encoder (chép nguyên tử)
  int32_t cnt[NUM_MOTORS];
  noInterrupts();
  for (uint8_t i = 0; i < NUM_MOTORS; i++) cnt[i] = encCount[i];
  interrupts();

  // 2) Tốc độ đo (chuẩn hóa) + lọc thông thấp
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    int32_t d = cnt[i] - lastCnt[i];
    lastCnt[i] = cnt[i];
    float rpm = (float)d * (60.0f / COUNTS_PER_REV) / dt;
    speedN[i] += speedLpf * (rpm * INV_RPM_NOM - speedN[i]);
  }

  // 3) An toàn: nút dừng khẩn + mất lệnh từ Pi
#if ESTOP_ENABLE
  if (digitalRead(PIN_ESTOP) == HIGH) estopLatched = true;   // NC hở mạch -> dừng
#endif
  cmdTimeout = (millis() - lastCmdMs) > CMD_TIMEOUT_MS;
  if (cmdTimeout) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) targetRpm[i] = 0.0f;
    openLoop = false;
  }
  if (estopLatched) {
    brakeAll();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      ctrl[i].hold(speedN[i]);
      refN[i] = 0.0f;
      uOut[i] = 0.0f;
      targetRpm[i] = 0.0f;
    }
    openLoop = false;
    return;
  }

  // 4) Điều khiển từng bánh
  const float stepMax = accelLimitN * CTRL_TS;
  const float idleN = IDLE_SPEED_RPM * INV_RPM_NOM;
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    float u;
    if (openLoop) {
      u = sd_clamp(openDuty[i], -1.0f, 1.0f);
      ctrl[i].hold(speedN[i]);
      refN[i] = speedN[i];
      updateEncoderDir(i, u);
    } else {
      float tgt = sd_clamp(targetRpm[i] * INV_RPM_NOM, -1.0f, 1.0f);
      refN[i] += sd_clamp(tgt - refN[i], -stepMax, stepMax);   // giới hạn gia tốc
      updateEncoderDir(i, refN[i]);
      bool idle = fabsf(tgt) < 1e-4f && fabsf(refN[i]) < 1e-4f && fabsf(speedN[i]) < idleN;
      if (idle) {
        if (idleCnt[i] < 255) idleCnt[i]++;
      } else {
        idleCnt[i] = 0;
      }
      if (idleCnt[i] > IDLE_STEPS) {                           // đứng yên -> ngắt PWM
        u = 0.0f;
        ctrl[i].hold(speedN[i]);
      } else {
        u = ctrl[i].update(refN[i], speedN[i]);
      }
    }
    uOut[i] = u;
    setMotor(i, u);
  }

  // 5) Debug chi tiết 1 bánh (để vẽ đồ thị trong báo cáo)
  if (debugMotor >= 0 && debugMotor < NUM_MOTORS) {
    uint8_t i = (uint8_t)debugMotor;
    Serial.print(F("X,"));
    Serial.print(millis());              Serial.print(',');
    Serial.print(i);                     Serial.print(',');
    Serial.print(refN[i] * RPM_NOMINAL, 1);      Serial.print(',');
    Serial.print(ctrl[i].ym * RPM_NOMINAL, 1);   Serial.print(',');
    Serial.print(speedN[i] * RPM_NOMINAL, 1);    Serial.print(',');
    Serial.print(uOut[i], 3);            Serial.print(',');
    Serial.print(ctrl[i].dhat, 3);       Serial.print(',');
    Serial.print(ctrl[i].K[0], 3);       Serial.print(',');
    Serial.print(ctrl[i].K[1], 3);       Serial.print(',');
    Serial.println(ctrl[i].K[2], 3);
  }
}

// =====================================================================
//  TELEMETRY
// =====================================================================
void sendTelemetry() {
#if BATT_ENABLE
  float v = analogRead(PIN_VBAT) * (5.0f / 1023.0f) * BATT_DIVIDER_RATIO;
  vbat = (vbat < 0.5f) ? v : (vbat + 0.2f * (v - vbat));
#endif
  uint8_t flags = 0;
  if (estopLatched) flags |= FLAG_ESTOP;
  if (cmdTimeout) flags |= FLAG_TIMEOUT;
  if (openLoop) flags |= FLAG_OPENLOOP;
  Serial.print(F("T,"));
  Serial.print(millis());
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    Serial.print(',');
    Serial.print(speedN[i] * RPM_NOMINAL, 1);
  }
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    Serial.print(',');
    Serial.print(uOut[i], 2);
  }
  Serial.print(',');
  Serial.print(flags);
  Serial.print(',');
  Serial.print(P.mode);
  Serial.print(',');
  Serial.println(vbat, 2);
}

// =====================================================================
//  NHẬN LỆNH
// =====================================================================
/* Đọc n số thực dạng ",v1,v2,..." */
bool parseFloats(char* s, float* out, uint8_t n) {
  for (uint8_t k = 0; k < n; k++) {
    if (*s != ',') return false;
    s++;
    char* end;
    double v = strtod(s, &end);
    if (end == s) return false;
    out[k] = (float)v;
    s = end;
  }
  return true;
}

void handleLine(char* line) {
  float v[4];
  switch (line[0]) {
    case 'W':                                     // tốc độ đặt 4 bánh (RPM)
      if (parseFloats(line + 1, v, 4)) {
        lastCmdMs = millis();
        if (!estopLatched) {
          for (uint8_t i = 0; i < NUM_MOTORS; i++)
            targetRpm[i] = sd_clamp(v[i], -RPM_NOMINAL, RPM_NOMINAL);
          openLoop = false;
        }
      } else {
        Serial.println(F("A,ERR,W"));
      }
      break;

    case 'O':                                     // vòng hở (thử nghiệm)
      if (parseFloats(line + 1, v, 4)) {
        lastCmdMs = millis();
        if (!estopLatched) {
          for (uint8_t i = 0; i < NUM_MOTORS; i++) openDuty[i] = sd_clamp(v[i], -1.0f, 1.0f);
          openLoop = true;
        }
      } else {
        Serial.println(F("A,ERR,O"));
      }
      break;

    case 'S':                                     // dừng
      lastCmdMs = millis();
      for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        targetRpm[i] = 0.0f;
        openDuty[i] = 0.0f;
      }
      openLoop = false;
      break;

    case 'E':                                     // dừng khẩn phần mềm
      if (parseFloats(line + 1, v, 1)) {
        if (v[0] > 0.5f) {
          estopLatched = true;
          Serial.println(F("A,OK,E,1"));
        } else {
#if ESTOP_ENABLE
          if (digitalRead(PIN_ESTOP) == HIGH) {   // nút vẫn đang nhấn -> không cho xóa
            Serial.println(F("A,ERR,ESTOP_ACTIVE"));
            break;
          }
#endif
          estopLatched = false;
          for (uint8_t i = 0; i < NUM_MOTORS; i++) setMotor(i, 0.0f);
          Serial.println(F("A,OK,E,0"));
        }
      } else {
        Serial.println(F("A,ERR,E"));
      }
      break;

    case 'C':                                     // chọn bộ điều khiển
      if (parseFloats(line + 1, v, 1)) {
        P.mode = (int16_t)(v[0] + 0.5f);
        sacdob_prepare(&P);
        for (uint8_t i = 0; i < NUM_MOTORS; i++) ctrl[i].hold(speedN[i]);
        Serial.print(F("A,OK,C,"));
        Serial.println(P.mode);
      } else {
        Serial.println(F("A,ERR,C"));
      }
      break;

    case 'R':                                     // đặt lại độ lợi thích nghi
      for (uint8_t i = 0; i < NUM_MOTORS; i++) ctrl[i].resetGains();
      Serial.println(F("A,OK,R"));
      break;

    case 'P':
      if (line[1] == ',') setParam(line + 2);
      else Serial.println(F("A,ERR,P"));
      break;

    case 'Q':
      printParams();
      break;

    case 'D':
      if (parseFloats(line + 1, v, 1)) {
        debugMotor = (v[0] < -0.5f || v[0] > 3.5f) ? -1 : (int8_t)(v[0] + 0.5f);
        Serial.print(F("A,OK,D,"));
        Serial.println(debugMotor);
      } else {
        Serial.println(F("A,ERR,D"));
      }
      break;

    case '?':
      printInfo();
      break;

    default:
      Serial.println(F("A,ERR,CMD"));
      break;
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = 0;
        handleLine(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;                                  // tràn bộ đệm -> bỏ dòng
      Serial.println(F("A,ERR,OVF"));
    }
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  // Đưa động cơ về trạng thái an toàn ngay khi khởi động
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(PIN_EN[i], OUTPUT);
    digitalWrite(PIN_EN[i], LOW);
    pinMode(PIN_IN1[i], OUTPUT);
    pinMode(PIN_IN2[i], OUTPUT);
    digitalWrite(PIN_IN1[i], LOW);
    digitalWrite(PIN_IN2[i], LOW);
  }
#if PWM_TIMER1_FAST
  // Timer1 (D9, D10): Fast PWM 8-bit, prescaler 64 -> ~976 Hz giống D5/D6
  TCCR1A = (TCCR1A & ~_BV(WGM11)) | _BV(WGM10);
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
#endif

  // Encoder
  pinMode(PIN_ENC_FL, INPUT_PULLUP);
  pinMode(PIN_ENC_FR, INPUT_PULLUP);
  pinMode(PIN_ENC_RL, INPUT_PULLUP);
  pinMode(PIN_ENC_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_FL), isrEncFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_FR), isrEncFR, CHANGE);
  lastPinC = PINC;
  PCMSK1 |= _BV(PCINT10) | _BV(PCINT11);
  PCIFR |= _BV(PCIF1);
  PCICR |= _BV(PCIE1);

#if ESTOP_ENABLE
  pinMode(PIN_ESTOP, INPUT_PULLUP);
#endif

  loadDefaultParams();
  for (uint8_t i = 0; i < NUM_MOTORS; i++) ctrl[i].init(&P);

  Serial.begin(SERIAL_BAUD);
  printInfo();

  lastCmdMs = millis() - CMD_TIMEOUT_MS - 1;     // chưa có lệnh -> đứng yên
  lastTelMs = millis();
  lastStepUs = micros();
  nextCtrlUs = lastStepUs + CTRL_PERIOD_US;

#if HW_WATCHDOG
  wdt_enable(WDTO_1S);
#endif
}

void loop() {
#if HW_WATCHDOG
  wdt_reset();
#endif
  readSerial();

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextCtrlUs) >= 0) {
    nextCtrlUs += CTRL_PERIOD_US;
    if ((int32_t)(nowUs - nextCtrlUs) >= 0) nextCtrlUs = nowUs + CTRL_PERIOD_US;  // bị trễ -> đồng bộ lại
    controlStep();
  }

  uint32_t nowMs = millis();
  if (nowMs - lastTelMs >= TELEMETRY_PERIOD_MS) {
    lastTelMs = nowMs;
    sendTelemetry();
  }
}

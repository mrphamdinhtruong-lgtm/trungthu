// =====================================================================
//  FILE TỰ SINH bởi flash_arduino.py từ robot/config.py – KHÔNG SỬA TAY
// =====================================================================
#ifndef CONFIG_GEN_H
#define CONFIG_GEN_H

#define FW_CONFIG_HASH         "b4960a64"

// ---- Truyền thông & thời gian ----
#define SERIAL_BAUD            115200UL
#define CONTROL_PERIOD_MS      20
#define TELEMETRY_PERIOD_MS    50
#define CMD_TIMEOUT_MS         500
#define HW_WATCHDOG            1

// ---- Động cơ & encoder ----
#define RPM_NOMINAL            110.0f
#define COUNTS_PER_REV         1980.0f
#define MOTOR_INVERT_FL        0
#define MOTOR_INVERT_FR        1
#define MOTOR_INVERT_RL        0
#define MOTOR_INVERT_RR        1
#define PWM_DEADZONE           28
#define PWM_TIMER1_FAST        1
#define ACCEL_LIMIT_RPM_S      300.0f
#define SPEED_LPF_ALPHA        0.5f
#define IDLE_SPEED_RPM         2.0f
#define DIR_SWITCH_RPM         4.0f

// ---- An toàn & pin ----
#define ESTOP_ENABLE           0
#define BATT_ENABLE            0
#define BATT_DIVIDER_RATIO     3.1276595744680846f

// ---- Con quay MPU-6050 (I2C: SDA = A4, SCL = A5) ----
#define IMU_ENABLE             1
#define IMU_I2C_ADDR           0x68
#define IMU_YAW_SIGN           1.0f
#define IMU_STILL_DPS          1.0f

// ---- Bộ điều khiển SAC + DOB ----
#define CTRL_MODE_DEFAULT      2
#define SAC_TAU_M              0.15f
#define SAC_GI_E               5.0f
#define SAC_GI_X               10.0f
#define SAC_GI_U               10.0f
#define SAC_GP_E               1.0f
#define SAC_GP_X               0.3f
#define SAC_GP_U               0.3f
#define SAC_SIGMA              0.1f
#define SAC_K0_E               1.0f
#define SAC_K0_X               0.4f
#define SAC_K0_U               0.8f
#define SAC_KMIN_E             0.0f
#define SAC_KMIN_X             -1.5f
#define SAC_KMIN_U             -1.5f
#define SAC_KMAX_E             8.0f
#define SAC_KMAX_X             3.0f
#define SAC_KMAX_U             3.0f
#define SAC_E_DEADBAND         0.01f
#define DOB_KN                 0.9f
#define DOB_TAU_N              0.1f
#define DOB_TAU_Q              0.12f
#define DOB_LIMIT              0.6f
#define PI_KP                  1.0f
#define PI_KI                  8.0f

#endif  // CONFIG_GEN_H

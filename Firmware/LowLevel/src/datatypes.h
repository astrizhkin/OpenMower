// Created by Clemens Elflein on 3/07/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#ifndef _DATATYPES_H
#define _DATATYPES_H

#include <stdint.h>

#define PACKET_ID_LL_STATUS 1
#define PACKET_ID_LL_IMU 2
#define PACKET_ID_LL_UI_EVENT 3
#define PACKET_ID_LL_HEARTBEAT 0x42
#define PACKET_ID_LL_HIGH_LEVEL_STATE 0x43
#define PACKET_ID_LL_MOTOR_STATE 0x44

enum HighLevelMode {
    MODE_IDLE = 1, // ROS connected, idle mode
    MODE_AUTONOMOUS = 2, // ROS connected, Autonomous mode, either mowing or docking or undocking
    MODE_RECORDING = 3 // ROS connected, Manual mode during recording etc
};

typedef enum StatusBits {
    STATUS_INIT_BIT = 0,
    STATUS_RASPI_POWER_BIT = 1,
    STATUS_CHARGING_ALLOWED_BIT = 2,
    STATUS_ESC_ENABLED_BIT = 3,
    STATUS_RAIN_BIT = 4,
    STATUS_USS_TIMEOUT_BIT = 5,
    STATUS_IMU_TIMEOUT_BIT = 6,
    STATUS_BATTERY_EMPTY_BIT = 7,
    STATUS_BMS_TIMEOUT_BIT = 8
} StatusBits;

#define EMERGENCY_BUTTON1_BIT 1
#define EMERGENCY_BUTTON2_BIT 2
#define EMERGENCY_LIFT1_BIT 3
#define EMERGENCY_LIFT2_BIT 4
#define EMERGENCY_ROS_TIMEOUT 5
#define EMERGENCY_HIGH_LEVEL 6

// motor status bits
#define MOTOR_STATUS_DISABLED               0
#define MOTOR_STATUS_BAD_CTRL_MODE          1
#define MOTOR_STATUS_LEFT_MOTOR_ERR         2
#define MOTOR_STATUS_RIGHT_MOTOR_ERR        3

#define MOTOR_STATUS_PCB_TEMP_WARN          4
#define MOTOR_STATUS_PCB_TEMP_ERR           5
#define MOTOR_STATUS_LEFT_MOTOR_TEMP_ERR    6
#define MOTOR_STATUS_RIGHT_MOTOR_TEMP_ERR   7

#define MOTOR_STATUS_CONN_TIMEOUT           8
#define MOTOR_STATUS_ADC_TIMEOUT            9
#define MOTOR_STATUS_GEN_TIMEOUT            10

#define MOTOR_STATUS_BATTERY_DEAD           12
#define MOTOR_STATUS_BATTERY_L1             13
#define MOTOR_STATUS_BATTERY_L2             14

//XESC fault codes
typedef enum XescFaultCode {
    XESC_FAULT_UNINITIALIZED=1,
    XESC_FAULT_WATCHDOG=2,
    XESC_FAULT_UNDERVOLTAGE=4,
    XESC_FAULT_OVERVOLTAGE=8,
    XESC_FAULT_OVERCURRENT=16,
    XESC_FAULT_OVERTEMP_MOTOR=32,
    XESC_FAULT_OVERTEMP_PCB=64,
    XESC_FAULT_INVALID_HALL=128
} XescFaultCode;

// ############################### BATTERY ###############################
/* Battery voltage calibration: connect power source.
 * see How to calibrate.
 * Write debug output value nr 5 to BAT_CALIB_ADC. make and flash firmware.
 * Then you can verify voltage on debug output value 6 (to get calibrated voltage multiplied by 100).
*/
//3277 - 5 sec, 655 - 20sec, 65 - 100sec
#define BAT_FILT_COEF           655       // battery voltage filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 3277 = 0.05 * 2^16. if 0.01 - it converges in 10 sec, if 0.001 it converges in 100sec
#define BAT_CALIB_REAL_VOLTAGE  4160      // input voltage measured by multimeter (multiplied by 100). In this case 41.60 V * 100 = 4160
#define BAT_CALIB_ADC           3325      // adc-value measured by mainboard (value nr 5 on UART debug output)

#define BAT_CELLS               12        // battery number of cells. Normal Hoverboard battery: 10s
#define BAT_LIFEPO4                         //battry type BAT_LIION or BAT_LIFEPO4

#ifdef BAT_LIION
    #define BAT_MAX             (4.25 * BAT_CELLS)    // Max voltage during charge
    #define BAT_STOP_CHARGE     (4.20 * BAT_CELLS)    // Stop charge
    #define BAT_FULL1           (4.15 * BAT_CELLS)    // Full right after charge
    #define BAT_FULL2           (4.10 * BAT_CELLS)    // Full
    #define BAT_OK1             (4.05 * BAT_CELLS)    // ok
    #define BAT_OK2             (4.00 * BAT_CELLS)    // ok
    #define BAT_OK3             (3.90 * BAT_CELLS)    // ok
    #define BAT_OK4             (3.80 * BAT_CELLS)    // ok
    #define BAT_WARN1           (3.70 * BAT_CELLS)    // warning 1
    #define BAT_WARN2           (3.60 * BAT_CELLS)    // warning 2
    #define BAT_WARN3           (3.50 * BAT_CELLS)    // almost empty. Charge now!
    #define BAT_EMPTY           (3.37 * BAT_CELLS)    // empty
#endif

#ifdef BAT_LIFEPO4
    #define BAT_CHARGER_MAX     (3.75 * BAT_CELLS)    // Charger voltage limit
    #define BAT_MAX             (3.60 * BAT_CELLS)    // Max voltage during charge
    #define BAT_STOP_CHARGE     (3.55 * BAT_CELLS)    // Stop charge
    #define BAT_FULL_CHARGE     (3.50 * BAT_CELLS)    // Full during charge
    #define BAT_FULL1           (3.45 * BAT_CELLS)    // Full right after charge
    #define BAT_FULL2           (3.40 * BAT_CELLS)    // Full
    #define BAT_OK1             (3.30 * BAT_CELLS)    // ok
    #define BAT_OK2             (3.20 * BAT_CELLS)    // ok
    #define BAT_OK3             (3.10 * BAT_CELLS)    // ok
    #define BAT_OK4             (3.00 * BAT_CELLS)    // ok
    #define BAT_WARN1           (2.90 * BAT_CELLS)    // warning 1
    #define BAT_WARN2           (2.80 * BAT_CELLS)    // warning 2
    #define BAT_WARN3           (2.70 * BAT_CELLS)    // almost empty. Charge now!
    #define BAT_EMPTY           (2.50 * BAT_CELLS)    // empty
#endif

#define CHARGE_MAX_VOLT         (4.35 * BAT_CELLS)    // Charger max voltage limit
#define CHARGE_MIN_VOLT         5.0                   // Charger min voltage limit

#define CHARGE_FILT_COEF           6553 //0.1 * 2^16
#define CHARGE_CALIB_REAL_VOLTAGE  4140
#define CHARGE_CALIB_ADC           3382

#define CURRENT_FILT_COEF           3277 //0.1 * 2^16
#define CURRENT_CALIB_REAL_CURRENT  110
#define CURRENT_CALIB_ADC           275

// ######################## END OF BATTERY ###############################

#pragma pack(push, 1)
struct ll_status {
    // Type of this message. Has to be PACKET_ID_LL_STATUS.
    uint8_t type;
    // Bitmask for rain, sound, powers etc
    // Bit 0: Initialized (i.e. setup() was a success). If this is 0, all other bits are meaningless.
    // Bit 1: Raspberry Power
    // Bit 2: Charging enabled
    // Bit 3: ESC power enabled
    // Bit 4: Rain detected
    // Bit 5: don't care
    // Bit 6: don't care
    // Bit 7: don't care
    uint16_t status_bitmask;
    // USS range in m
    float uss_ranges_m[5];
    // USS measurement age in ms (no more than UINT16_MAX)
    uint16_t uss_age_ms[5];
    // Emergency bitmask:
    // Bit 0: Emergency latch
    // Bit 1: Emergency 0 active
    // Bit 2: Emergency 1 active
    // Bit 3: Emergency 2 active
    // Bit 4: Emergency 3 active
    // Bit 5: Emergency USS tiemout
    // Bit 6: Emergency IMU tiemout
    // Bit 7: Battery empty
    uint8_t emergency_bitmask;
    // Charge voltage
    float v_charge;
    // System voltage
    float v_battery;
    // Charge current
    float charging_current;
    uint8_t batt_percentage;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_imu {
    // Type of this message. Has to be PACKET_ID_LL_IMU.
    uint8_t type;
    // Time since last message in milliseconds.
    uint16_t dt_millis;
    // Acceleration[m^s2], Gyro[rad/s] and magnetic field[uT]
    float acceleration_mss[3];
    float gyro_rads[3];
    float mag_uT[3];
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_heartbeat {
    // Type of this message. Has to be PACKET_ID_LL_HEARTBEAT.
    uint8_t type;
    //high level emergency bits (e.g. navigation error, ...)
    uint8_t emergency_high_level;
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)


#pragma pack(push, 1)
struct ll_high_level_state {
    // Type of this message. Has to be PACKET_ID_LL_HIGH_LEVEL_STATE
    uint8_t type;
    uint8_t current_mode; // see HighLevelMode
    uint8_t gps_quality;   // GPS quality in percent (0-100)
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct ll_motor_state {
    // Type of this message. Has to be PACKET_ID_LL_HIGH_LEVEL_STATE
    uint8_t type;
    //uint8_t motor_id;

    //rad/s
    //float cmd[5];
    //rad/s
    //float speed_meas[5];
    //rad
    //float wheel_cnt[5];
    //float curr_meas[5];
    //float motor_temp[5];
    //float boardTemp[3];
    uint16_t status[3]; //See motor status bits
    uint8_t status_age_s[3];
    uint16_t crc;
} __attribute__((packed));
#pragma pack(pop)

#endif

#define DISPLAY_NO_BLINK    0b0000
#define DISPLAY_FAST_BLINK  0b0001
#define DISPLAY_NORM_BLINK  0b0010
#define DISPLAY_SLOW_BLINK  0b0100
#define DISPLAY_BLINK_CYCLE 0b1000





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
#include <Arduino.h>
#include <FastCRC.h>
#include <PacketSerial.h>
#include "datatypes.h"
#include "defines.h"
#include "pins.h"
#include "imu.h"
#include <U8x8lib.h>
//#include <U8g2lib.h>
#include <Wire.h>

#define IMU_CYCLETIME_MS 20              // cycletime for refresh IMU data
#define STATUS_CYCLETIME_MS 100          // cycletime for refresh analog and digital Statusvalues
#define DISPLAY_CYCLETIME_MS 200      // cycletime for refresh UI status LEDs

#define TILT_EMERGENCY_MS 2500  // Time for a single wheel to be lifted in order to count as emergency. This is to filter uneven ground.
#define LIFT_EMERGENCY_MS 100  // Time for both wheels to be lifted in order to count as emergency. This is to filter uneven ground.
#define BUTTON_EMERGENCY_MS 20 // Time for button emergency to activate. This is to debounce the button if triggered on bumpy surfaces

// Emergency will be engaged, if no heartbeat was received in this time frame.
#define HEARTBEAT_TIMEOUT_MS 500
#define HIGH_LEVEL_TIMEOUT_MS 5000
#define MOTOR_STATUS_TIMEOUT_MS 500
#define IMU_TIMEOUT_MS 200

#define USS_TIMEOUT_MS 1000
#define USS_COUNT 5
#define USS_ENABLED 0b11000
#define USS_CYCLETIME_MS 50
#define USS_ECHO_TIMEOUT_US 50000UL

// Define to stream debugging messages via USB
// #define USB_DEBUG

// Only define DEBUG_SERIAL if USB_DEBUG is actually enabled.
// This enforces compile errors if it's used incorrectly.
#ifdef USB_DEBUG
    #define DEBUG_SERIAL Serial
#endif

#define PACKET_SERIAL Serial1

// Millis after charging is retried
#define CHARGING_RETRY_MS 10000

/**
 * @brief Some hardware parameters
 */
#define VIN_R1 10000.0f
#define VIN_R2 1000.0f
#define R_SHUNT 0.003f
#define CURRENT_SENSE_GAIN 100.0f

PacketSerial packetSerial; // COBS communication PICO <> Raspi
FastCRC16 CRC16;

unsigned long last_imu_ms = 0;
unsigned long last_status_update_ms = 0;
unsigned long last_heartbeat_ms = 0;
unsigned long last_high_level_ms = 0;
unsigned long last_motor_ms = 0;
unsigned long last_display_ms = 0;

unsigned long lift_emergency_started_ms = 0;
unsigned long tilt_emergency_started_ms = 0;
unsigned long button_emergency_started_ms = 0;

// Predefined message buffers, so that we don't need to allocate new ones later.
struct ll_imu imu_message = {0};
struct ll_status status_message = {0};
// current high level state
struct ll_high_level_state last_high_level_state = {0};
struct ll_motor_state last_motor_state = {0};

// A mutex which is used by core1 each time status_message is modified.
// We can lock it during message transmission to prevent core1 to modify data in this time.
auto_init_mutex(mtx_status_message);

bool emergency_latch = true;
bool charging_allowed = false;
bool ROS_running = false;
unsigned long charging_disabled_time = 0;

float imu_temp[9];

//display status messages
char    status_line[16+1];
uint8_t status_line_blink[16];

char    display_motor_status[8+1];
uint8_t display_motor_status_blink[8];

//char    ll_display_emerg[8+1]; //8 bit chars or char enchoded messages
//uint8_t ll_display_emerg_blink[8];
uint8_t displayUpdateCounter = 0;
//temp buffer
char    display_line[16+1]; //16 chars


unsigned long last_uss_sensor_result_ms[USS_COUNT];
unsigned long last_uss_trigger=0;

int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

#define DISPLAY_WIRE Wire
U8X8_SSD1309_128X64_NONAME0_HW_I2C display(U8X8_PIN_NONE);
//U8G2_SSD1309_128X64_NONAME0_2_HW_I2C display(U8G2_R0, PIN_DISPLAY_RESET, PIN_DISPLAY_SCK, PIN_DISPLAY_SDA);

void sendMessage(void *message, size_t size);
void updateDisplay(bool forceDisplay);
void updateBlinkState(char *message, uint8_t blinkState, int size, int currentBlinkState);
void onPacketReceived(const uint8_t *buffer, size_t size);

void setRaspiPower(bool power) {
    // Update status bits in the status message
    status_message.status_bitmask &= ~(1 << STATUS_RASPI_POWER_BIT);
    status_message.status_bitmask |= (power & 0b1) << STATUS_RASPI_POWER_BIT;
    digitalWrite(PIN_RASPI_POWER, power);
}

void setEscEnable(bool enable) {
    // Update status bits in the status message
    status_message.status_bitmask &= ~(1 << STATUS_ESC_ENABLED_BIT);
    status_message.status_bitmask |= (enable & 1) << STATUS_ESC_ENABLED_BIT;
    digitalWrite(PIN_ESC_ENABLE, enable);
}

void updateEmergency() {
    unsigned long now = millis();
    if (now - last_heartbeat_ms > HEARTBEAT_TIMEOUT_MS) {
        emergency_latch = true;
        ROS_running = false;
    }
    uint8_t last_emergency = status_message.emergency_bitmask & (1<<EMERGENCY_LATCH_BIT);

    // Mask the emergency bits. 2x Lift sensor, 2x Emergency Button
    bool emergency1 = !gpio_get(PIN_EMERGENCY_1);
    bool emergency2 = !gpio_get(PIN_EMERGENCY_2);
    bool emergency3 = !gpio_get(PIN_EMERGENCY_3);
    bool emergency4 = !gpio_get(PIN_EMERGENCY_4);

    now = millis();
    uint8_t emergency_state = 0;

    bool is_tilted = emergency1 || emergency2;
    bool is_lifted = emergency1 && emergency2;
    bool stop_pressed = emergency3 || emergency4;

    if (is_lifted) {
        // We just lifted, store the timestamp
        if (lift_emergency_started_ms == 0) {
            lift_emergency_started_ms = now;
        }
    } else {
        // Not lifted, reset the time
        lift_emergency_started_ms = 0;
    }

    if (stop_pressed) {
        // We just pressed, store the timestamp
        if (button_emergency_started_ms == 0) {
            button_emergency_started_ms = now;
        }
    } else {
        // Not pressed, reset the time
        button_emergency_started_ms = 0;
    }

    if (lift_emergency_started_ms > 0 && (now - lift_emergency_started_ms) >= LIFT_EMERGENCY_MS) {
        // Emergency bit 2 (lift wheel 1)set?
        if (emergency1)
            emergency_state |= 1<<EMERGENCY_LIFT1_BIT;
        // Emergency bit 1 (lift wheel 2)set?
        if (emergency2)
            emergency_state |= 1<<EMERGENCY_LIFT2_BIT;
    }

    if (is_tilted) {
        // We just tilted, store the timestamp
        if (tilt_emergency_started_ms == 0) {
            tilt_emergency_started_ms = now;
        }
    } else {
        // Not tilted, reset the time
        tilt_emergency_started_ms = 0;
    }

    if (tilt_emergency_started_ms > 0 && (now - tilt_emergency_started_ms) >= TILT_EMERGENCY_MS) {
        // Emergency bit 2 (lift wheel 1)set?
        if (emergency1)
            emergency_state |= 1<<EMERGENCY_LIFT1_BIT;
        // Emergency bit 1 (lift wheel 2)set?
        if (emergency2)
            emergency_state |= 1<<EMERGENCY_LIFT2_BIT;
    }
    if (button_emergency_started_ms > 0 && (now - button_emergency_started_ms) >= BUTTON_EMERGENCY_MS) {
        // Emergency bit 2 (stop button) set?
        if (emergency3)
            emergency_state |= 1<<EMERGENCY_BUTTON1_BIT;
        // Emergency bit 1 (stop button)set?
        if (emergency4)
            emergency_state |= 1<<EMERGENCY_BUTTON2_BIT;
    }

    //ESC must be enabled 
    setEscEnable(emergency_state==0 && ROS_running);

    if (emergency_state || emergency_latch) {
        emergency_latch = true;
        emergency_state |= 1<<EMERGENCY_LATCH_BIT;
    }

    status_message.emergency_bitmask = emergency_state;

    // If it's a new emergency, instantly send the message. This is to not spam the channel during emergencies.
    if (last_emergency != (emergency_state & (1<<EMERGENCY_LATCH_BIT))) {
        sendMessage(&status_message, sizeof(struct ll_status));

        // Update LEDs instantly
        updateDisplay(true);
    }
}

/*double kalman(double U){
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P*H/(H*P*H+R);
  U_hat += + K*(U-H*U_hat);
  P = (1-K*H)*P+Q;
  return U_hat;
}*/

void setup1() {
    // Core
    digitalWrite(LED_BUILTIN, HIGH);
}
void loop1() {
    // Loop through the mux and query actions. Store the result in the multicore fifo
    bool state;
    double distance;
    unsigned long duration;
    unsigned long wait_us,response_us,uss_measurement_age;   

    for (uint8_t mux_address = 0; mux_address < 7; mux_address++) {
        gpio_put_masked(0b111 << 13, mux_address << 13);
        delay(1);
        
        if(mux_address<USS_COUNT) {
            if ( (USS_ENABLED & (1 << mux_address))==0 ) {
                duration = 0;
                distance = 0;
                continue;
            }
            digitalWrite(PIN_MUX_OUT, LOW); 
            unsigned long prev_uss_delta = millis()-last_uss_trigger;
            if(prev_uss_delta < USS_CYCLETIME_MS) {
                wait_us = (USS_CYCLETIME_MS-prev_uss_delta)*1000;
            }else{
                wait_us = 5;
            }
            delayMicroseconds(wait_us);
            // Sets the trigPin on HIGH state for 20 micro seconds at least
            digitalWrite(PIN_MUX_OUT, HIGH); 
            delayMicroseconds(25);
            last_uss_trigger = millis();
            digitalWrite(PIN_MUX_OUT, LOW);
            duration = pulseIn(PIN_MUX_IN, HIGH,USS_ECHO_TIMEOUT_US); // 35000UL for full cycle, Reads the echoPin, returns the sound wave travel time in microseconds
            distance = duration*0.343/2;//0.343mm per 1 microsecond
            //distance = kalman(distance);
            /*if(distance < 2000) {
                distance = 0;
            }*/
            response_us = millis()-last_uss_trigger;
            #ifdef USB_DEBUG
                DEBUG_SERIAL.printf("%i %icm %ius wt %ims rsp %ims\t",mux_address,(int)(distance/10),duration,wait_us/1000,response_us);
            #endif
        } else {
            state = gpio_get(PIN_MUX_IN);
        }

        mutex_enter_blocking(&mtx_status_message);
        switch (mux_address) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                if(duration!=0) {
                    status_message.uss_ranges_m[mux_address]=distance/1000;
                    last_uss_sensor_result_ms[mux_address]=last_uss_trigger;
                }
                uss_measurement_age = millis() - last_uss_sensor_result_ms[mux_address];
                if(uss_measurement_age > UINT16_MAX) {
                    uss_measurement_age = UINT16_MAX;
                }
                status_message.uss_age_ms[mux_address] = uss_measurement_age;
                break;
            case 5:
                //inverted
                if (state) {
                    status_message.status_bitmask &= ~(1 << STATUS_RAIN_BIT);
                } else {
                    status_message.status_bitmask |= 1 << STATUS_RAIN_BIT;
                }
                break;
            default:
                break;
        }
        mutex_exit(&mtx_status_message);
    }
    #ifdef USB_DEBUG
        DEBUG_SERIAL.println();
    #endif

    delay(10);
}

void setup() {
    //  We do hardware init in this core, so that we don't get invalid states.
    //  Therefore, we pause the other core until setup() was a success
    rp2040.idleOtherCore();

#ifdef USB_DEBUG
    int wait_count = 500;
    DEBUG_SERIAL.begin(9600);
    while(!DEBUG_SERIAL && wait_count-- > 0){
        delay(10);
    }
#endif
    
#ifdef USB_DEBUG
    DEBUG_SERIAL.println("Begin initialization");
#endif

    emergency_latch = true;
    ROS_running = false;

    lift_emergency_started_ms = 0;
    button_emergency_started_ms = 0;
    // Initialize messages
    imu_message = {0};
    status_message = {0};
    imu_message.type = PACKET_ID_LL_IMU;
    status_message.type = PACKET_ID_LL_STATUS;

    // Setup pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_ENABLE_CHARGE, OUTPUT);
    pinMode(PIN_RASPI_POWER,OUTPUT);
    pinMode(PIN_ESC_ENABLE, OUTPUT);

    DISPLAY_WIRE.setSDA(PIN_DISPLAY_SDA);
    DISPLAY_WIRE.setSCL(PIN_DISPLAY_SCK);
#ifdef USB_DEBUG
    DEBUG_SERIAL.println("Display init");
#endif
    display.begin();
#ifdef USB_DEBUG
    DEBUG_SERIAL.println("Load font");
#endif
    //display.setFont(u8x8_font_artossans8_r);//capital A - 7
    display.setFont(u8x8_font_chroma48medium8_r);//capital A - 6
   
    display.drawString(0,0,"Mover v0.1");
    delay(100);
    // Enable raspi power
#ifdef USB_DEBUG
    DEBUG_SERIAL.println("Power RPi");
#endif
    display.drawString(0,1,"Power pins...");
    delay(100);
    setRaspiPower(true);
    setEscEnable(false);

    pinMode(PIN_MUX_IN, INPUT);
    pinMode(PIN_MUX_OUT, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_0, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_1, OUTPUT);
    pinMode(PIN_MUX_ADDRESS_2, OUTPUT);

    pinMode(PIN_EMERGENCY_1, INPUT);
    pinMode(PIN_EMERGENCY_2, INPUT);
    pinMode(PIN_EMERGENCY_3, INPUT);
    pinMode(PIN_EMERGENCY_4, INPUT);

    analogReadResolution(12);

    // init serial com to RasPi
    PACKET_SERIAL.begin(115200);
    packetSerial.setStream(&PACKET_SERIAL);
    packetSerial.setPacketHandler(&onPacketReceived);
#ifdef USB_DEBUG
    DEBUG_SERIAL.println("IMU init");
#endif

    /*
     * IMU INITIALIZATION
     */
    display.drawString(0,2,"IMU init...");
    delay(100);
    bool init_imu_success = false;
    int init_imu_tries = 1000;
    while(init_imu_tries --> 0) {
#ifdef USB_DEBUG
        if(init_imu(&DEBUG_SERIAL)) {
#else
        if(init_imu(0)) {
#endif
            init_imu_success = true;
            break;
        } 
#ifdef USB_DEBUG
        DEBUG_SERIAL.println("IMU initialization unsuccessful, retrying in 1 sec");
#endif
        display.drawString(0,3,"IMU fail! Retry");
    }

    if (!init_imu_success) {
#ifdef USB_DEBUG
        DEBUG_SERIAL.println("IMU initialization unsuccessful");
        DEBUG_SERIAL.println("Check IMU wiring or try cycling power");
#endif
        display.drawString(0,3,"IMU fail!!!");
        status_message.status_bitmask = 0;
        while (1) { // Blink RED for IMU failure
            updateDisplay(true);
            delay(200);
        }
    }

    display.drawString(0,3,"IMU success");
#ifdef USB_DEBUG
    DEBUG_SERIAL.println("IMU initialized");
#endif
    delay(100);

    status_message.status_bitmask |= 1 << STATUS_INIT_BIT;

    rp2040.resumeOtherCore();

    display.drawString(0,4,"Init completed");
    delay(500);
    display.clear();
}

void onPacketReceived(const uint8_t *buffer, size_t size) {
    // sanity check for CRC to work (1 type, 1 data, 2 CRC)
    if (size < 4)
        return;

    // check the CRC
    uint16_t crc = CRC16.ccitt(buffer, size - 2);

    if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
        buffer[size - 2] != (crc & 0xFF))
        return;

    if (buffer[0] == PACKET_ID_LL_HEARTBEAT && size == sizeof(struct ll_heartbeat)) {

        // CRC and packet is OK, reset watchdog
        last_heartbeat_ms = millis();
        ROS_running = true;
        struct ll_heartbeat *heartbeat = (struct ll_heartbeat *) buffer;
        if (heartbeat->emergency_release_requested) {
            emergency_latch = false;
        }
        // Check in this order, so we can set it again in the same packet if required.
        if (heartbeat->emergency_requested) {
            emergency_latch = true;
        }
    } else if (buffer[0] == PACKET_ID_LL_HIGH_LEVEL_STATE && size == sizeof(struct ll_high_level_state)) {
        // copy the state
        last_high_level_state = *((struct ll_high_level_state *) buffer);
        last_high_level_ms = millis();
    } else if (buffer[0] == PACKET_ID_LL_MOTOR_STATE && size == sizeof(struct ll_motor_state)) {
        // copy the state
        last_motor_state = *((struct ll_motor_state *) buffer);
        last_motor_ms = millis();
    }
}

// returns true, if it's a good idea to charge the battery (current, voltages, ...)
bool checkShouldCharge() {
    return status_message.v_charge < 30.0 && status_message.charging_current < 1.5 && status_message.v_battery < 29.0;
}

void updateChargingEnabled() {
    if (charging_allowed) {
        if (!checkShouldCharge()) {
            digitalWrite(PIN_ENABLE_CHARGE, LOW);
            charging_allowed = false;
            charging_disabled_time = millis();
        }
    } else {
        // enable charging after CHARGING_RETRY_MILLIS
        if (millis() - charging_disabled_time > CHARGING_RETRY_MS) {
            if (!checkShouldCharge()) {
                digitalWrite(PIN_ENABLE_CHARGE, LOW);
                charging_allowed = false;
                charging_disabled_time = millis();
            } else {
                digitalWrite(PIN_ENABLE_CHARGE, HIGH);
                charging_allowed = true;
            }
        }
    }
}

/* =========================== Filtering Functions =========================== */

  /* Low pass filter fixed-point 32 bits: fixdt(1,32,16)
  * Max:  32767.99998474121
  * Min: -32768
  * Res:  1.52587890625e-05
  * 
  * Inputs:       u     = int16 or int32
  * Outputs:      y     = fixdt(1,32,16)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  * 
  * Example: 
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
  */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp;  
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);  // Overflow protection: 2147483647LL = 2^31 - 1
  *y = (int32_t)tmp + (*y);
}


void loop() {
    packetSerial.update();
    imu_loop();
    updateChargingEnabled();
    updateEmergency();

    unsigned long now = millis();
    if (now - last_imu_ms > IMU_CYCLETIME_MS) {
        // we have to copy to the temp data structure due to alignment issues
        if(imu_read(imu_temp, imu_temp + 3, imu_temp + 6)) {
            last_imu_ms = now;
            imu_message.acceleration_mss[0] = imu_temp[0];
            imu_message.acceleration_mss[1] = imu_temp[1];
            imu_message.acceleration_mss[2] = imu_temp[2];
            imu_message.gyro_rads[0] = imu_temp[3];
            imu_message.gyro_rads[1] = imu_temp[4];
            imu_message.gyro_rads[2] = imu_temp[5];
            imu_message.mag_uT[0] = imu_temp[6];
            imu_message.mag_uT[1] = imu_temp[7];
            imu_message.mag_uT[2] = imu_temp[8];
        }

        imu_message.dt_millis = now - last_imu_ms;
        sendMessage(&imu_message, sizeof(struct ll_imu));
    }

    if (now - last_status_update_ms > STATUS_CYCLETIME_MS) {
        uint16_t raw = analogRead(PIN_ANALOG_BATTERY_VOLTAGE);
        filtLowPass32(raw, BAT_FILT_COEF, &batVoltageFixdt);
        batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer

        status_message.v_battery = batVoltage/100.0;
        // status_message.v_battery = 
        //         (float) analogRead(PIN_ANALOG_BATTERY_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
        status_message.v_charge =
                (float) analogRead(PIN_ANALOG_CHARGE_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
#ifndef IGNORE_CHARGING_CURRENT
        status_message.charging_current =
                (float) analogRead(PIN_ANALOG_CHARGE_CURRENT) * (3.3f / 4096.0f) / (CURRENT_SENSE_GAIN * R_SHUNT);
#else
        status_message.charging_current = -1.0f;
#endif
        status_message.status_bitmask &= ~(1 << STATUS_CHARGING_ALLOWED_BIT);
        status_message.status_bitmask |= (charging_allowed & 0b1) << STATUS_CHARGING_ALLOWED_BIT;

        // Check USS timouts
        status_message.status_bitmask &= ~(1 << STATUS_USS_BIT);
        for(int i=0;i<USS_COUNT;i++) {
            if( (USS_ENABLED & (1<<i)) && (status_message.uss_age_ms[i] > USS_TIMEOUT_MS)) {
                status_message.status_bitmask |= 1<<STATUS_USS_BIT;
            }
        }

        // Check IMU temeout
        status_message.status_bitmask &= ~(1 << STATUS_IMU_BIT);
        if( now - last_imu_ms > IMU_TIMEOUT_MS ) {
            status_message.status_bitmask |= 1<<STATUS_IMU_BIT;
        }

        // Check battery empty
        status_message.status_bitmask &= ~(1 << STATUS_BATTERY_EMPTY_BIT);

        // calculate percent value accu filling
        float delta = BAT_FULL - BAT_DEAD;
        float vo = status_message.v_battery - BAT_DEAD;
        status_message.batt_percentage = vo / delta * 100;
        if (status_message.batt_percentage > 100)
            status_message.batt_percentage = 100;

        mutex_enter_blocking(&mtx_status_message);
        sendMessage(&status_message, sizeof(struct ll_status));
        mutex_exit(&mtx_status_message);

        last_status_update_ms = now;
#ifdef USB_DEBUG
        /*DEBUG_SERIAL.print("status: 0b");
        DEBUG_SERIAL.print(status_message.status_bitmask, BIN);
        DEBUG_SERIAL.print("\t");

        DEBUG_SERIAL.print("vin: ");
        DEBUG_SERIAL.print(status_message.v_battery, 3);
        DEBUG_SERIAL.print(" V\t");
        DEBUG_SERIAL.print("vcharge: ");
        DEBUG_SERIAL.print(status_message.v_charge, 3);
        DEBUG_SERIAL.print(" V\t");
        DEBUG_SERIAL.print("charge_current: ");
        DEBUG_SERIAL.print(status_message.charging_current, 3);
        DEBUG_SERIAL.print(" A\t");
        DEBUG_SERIAL.print("emergency: 0b");
        DEBUG_SERIAL.print(status_message.emergency_bitmask, BIN);
        DEBUG_SERIAL.println();

        DEBUG_SERIAL.print("acc x");
        DEBUG_SERIAL.print(imu_message.acceleration_mss[0], 2);
        DEBUG_SERIAL.print("\ty");
        DEBUG_SERIAL.print(imu_message.acceleration_mss[1], 2);
        DEBUG_SERIAL.print("\tz");
        DEBUG_SERIAL.print(imu_message.acceleration_mss[2], 2);
        DEBUG_SERIAL.print("\tgyro x");
        DEBUG_SERIAL.print(imu_message.gyro_rads[0], 2);
        DEBUG_SERIAL.print("\ty");
        DEBUG_SERIAL.print(imu_message.gyro_rads[1], 2);
        DEBUG_SERIAL.print("\tz");
        DEBUG_SERIAL.print(imu_message.gyro_rads[2], 2);
        DEBUG_SERIAL.println();*/

        /*DEBUG_SERIAL.print("USS: ");
        DEBUG_SERIAL.print(status_message.uss_ranges_m[0], 3);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(status_message.uss_ranges_m[1], 3);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(status_message.uss_ranges_m[2], 3);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(status_message.uss_ranges_m[3], 3);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(status_message.uss_ranges_m[4], 3);
        DEBUG_SERIAL.println();*/

#endif
    }

    if (now - last_display_ms > DISPLAY_CYCLETIME_MS) {
        updateDisplay(false);
        last_display_ms = now;
    }
}

void sendMessage(void *message, size_t size) {
    // Only send messages, if ROS is running, else Raspi sometimes doesn't boot
    if (!ROS_running)
        return;

    // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
    if (size < 4) {
        return;
    }
    uint8_t *data_pointer = (uint8_t *) message;

    // calculate the CRC
    uint16_t crc = CRC16.ccitt((uint8_t *) message, size - 2);
    data_pointer[size - 1] = (crc >> 8) & 0xFF;
    data_pointer[size - 2] = crc & 0xFF;

    packetSerial.send((uint8_t *) message, size);
}

void updateBlink(char *message, uint8_t *blinkState, int size,int currentBlinkCounter) {
    for(int i=0;i<size;i++){
        if(currentBlinkCounter & blinkState[i]) {
            message[i] = ' ';
        }
    }
}

void updateDisplay(bool forceDisplay) {
    long now = millis();

    displayUpdateCounter++;
    if(displayUpdateCounter & DISPLAY_BLINK_CYCLE) {
        displayUpdateCounter = 0;
    }                                         

    // Show Info Docking LED
    /*if ((status_message.charging_current > 0.80f) && (status_message.v_charge > 20.0f))
        setLed(leds_message, LED_CHARGING, LED_blink_fast);
    else if ((status_message.charging_current <= 0.80f) && (status_message.charging_current >= 0.15f) &&
             (status_message.v_charge > 20.0f))
        setLed(leds_message, LED_CHARGING, LED_blink_slow);
    else if ((status_message.charging_current < 0.15f) && (status_message.v_charge > 20.0f))
        setLed(leds_message, LED_CHARGING, LED_on);
    else
        setLed(leds_message, LED_CHARGING, LED_off);
    // Show Info Battery state
    if (status_message.v_battery >= (BATT_EMPTY + 2.0f))
        setLed(leds_message, LED_BATTERY_LOW, LED_off);
    else
        setLed(leds_message, LED_BATTERY_LOW, LED_on);
    if (status_message.v_charge < 10.0f) // activate only when undocked
    {
        // use the first LED row as bargraph
        setBars7(leds_message, status_message.batt_percentage / 100.0);
    } else {
        setBars7(leds_message, 0);
        setBars4(leds_message, 0);
    }*/

    //High level group 0-4 (5 symbols)
    //Communication heartbit status
    status_line[0]         = 'T';
    status_line_blink[0] = ROS_running ? DISPLAY_NO_BLINK : DISPLAY_SLOW_BLINK;

    //GPS status
    status_line[1]         = now - last_high_level_ms < HIGH_LEVEL_TIMEOUT_MS ? 'G' : 'g';
    if ( (now - last_high_level_ms) > HIGH_LEVEL_TIMEOUT_MS) { status_line_blink[1]=DISPLAY_SLOW_BLINK; }
    else if (last_high_level_state.gps_quality < 25) { status_line_blink[1]=DISPLAY_FAST_BLINK; }
    else if (last_high_level_state.gps_quality < 50) { status_line_blink[1]=DISPLAY_NORM_BLINK; }
    else if (last_high_level_state.gps_quality < 75) { status_line_blink[1]=DISPLAY_SLOW_BLINK; }
    else { status_line_blink[1]=DISPLAY_NO_BLINK; }

    //High level mode status
    if(now - last_high_level_ms < HIGH_LEVEL_TIMEOUT_MS) {
        uint8_t highLevelMode = last_high_level_state.current_mode & 0b111111;
        uint8_t highLevelSubMode = (last_high_level_state.current_mode >> 6) & 0b11;
        switch (highLevelMode) {
            case HighLevelMode::MODE_IDLE:
                status_line[2]         = 'I';
                status_line_blink[2]   = DISPLAY_NO_BLINK;
                break;
            case HighLevelMode::MODE_AUTONOMOUS:
                status_line[2]         = 'A';
                status_line_blink[2]   = DISPLAY_FAST_BLINK;
                break;
            case HighLevelMode::MODE_RECORDING:
                status_line[2]         = 'R';
                status_line_blink[2]   = DISPLAY_NO_BLINK;
                break;
            default:
                status_line[2]         = 'N';
                status_line_blink[2]   = DISPLAY_FAST_BLINK;
                break;
        }
        status_line[3]         = '0'+highLevelSubMode;
        status_line_blink[3]   = status_line_blink[2];
    } else {
        status_line[2]         = 'n';
        status_line_blink[2]   = DISPLAY_SLOW_BLINK;
        status_line[3]         = 'n';
        status_line_blink[3]   = status_line_blink[2];
    }

    //Emergency group 4-10 (7 symbols)
    status_line[4]         = status_message.status_bitmask & (1<<STATUS_BATTERY_EMPTY_BIT) ? 'V' : ' ';
    status_line_blink[4]   = DISPLAY_FAST_BLINK;
    //Stop button(s) pressed after timeout (PIN_EMERGENCY_3 PIN_EMERGENCY_4)
    status_line[5]         = status_message.emergency_bitmask & (1<<EMERGENCY_LIFT1_BIT | 1<<EMERGENCY_LIFT2_BIT) ? 'L' : ' ';
    status_line_blink[5]   = DISPLAY_FAST_BLINK;
    //Lift/Tilt occurs after timeout (PIN_EMERGENCY_1 PIN_EMERGENCY_2)
    status_line[6]         = status_message.emergency_bitmask & (1<<EMERGENCY_BUTTON1_BIT | 1<<EMERGENCY_BUTTON2_BIT) ? 'B' : ' ';
    status_line_blink[6]   = DISPLAY_FAST_BLINK;
    //Internal or external global emergency
    status_line[7]         = status_message.emergency_bitmask & (1<<EMERGENCY_LATCH_BIT) ? 'E' : ' ';
    status_line_blink[7]   = DISPLAY_FAST_BLINK;
    //Rain sensor
    status_line[8]         = status_message.status_bitmask & (1<<STATUS_RAIN_BIT) ? 'R' : ' ';
    status_line_blink[8]   = DISPLAY_FAST_BLINK;
    //USS timeout
    status_line[9]         = status_message.status_bitmask & (1<<STATUS_USS_BIT) ? 'U' : ' ';
    status_line_blink[9]   = DISPLAY_FAST_BLINK;
    //IMU timeout
    status_line[10]         = status_message.status_bitmask & (1<<STATUS_IMU_BIT) ? 'I' : ' ';
    status_line_blink[10]   = DISPLAY_FAST_BLINK;

    //Output status group 11-12 (2 sybmols)
    //Power pin
    status_line[11]        = 'P';
    status_line_blink[12]        = status_message.status_bitmask & (1<<STATUS_RASPI_POWER_BIT) ? DISPLAY_NO_BLINK : DISPLAY_SLOW_BLINK;
    //ESC enable
    status_line[12]        = 'M';
    status_line_blink[12]        = status_message.status_bitmask & (1<<STATUS_ESC_ENABLED_BIT) ? DISPLAY_NO_BLINK : DISPLAY_SLOW_BLINK;
    
    //Charging and battery status 12-15 (4 symbols)
    status_line[13]        = status_message.status_bitmask & (1<<STATUS_CHARGING_ALLOWED_BIT) ? 'C' : ' ';
    status_line[14]        = ' ';
    status_line[15]        = ' ';
    updateBlink(status_line,status_line_blink,16,displayUpdateCounter);

    snprintf(display_line,17, "%.16s",status_line);
    display.drawString(0, 0, display_line);

    //ll_display_emerg[0]         = ' ';//status_message.emergency_bitmask & (1<<EMERGENCY_BATTERY_EMPTY_BIT) ? 'V' : ' ';
    //ll_display_emerg[1]         = ' ';//status_message.emergency_bitmask & (1<<EMERGENCY_IMU_BIT) ? 'I' : ' ';
    //ll_display_emerg[2]         = ' ';//status_message.emergency_bitmask & (1<<EMERGENCY_USS_BIT) ? 'U' : ' ';
    //ll_display_emerg[3]         = status_message.emergency_bitmask & (1<<EMERGENCY_LIFT2_BIT) ? 'L' : ' ';
    //ll_display_emerg[4]         = status_message.emergency_bitmask & (1<<EMERGENCY_LIFT1_BIT) ? 'L' : ' ';
    //ll_display_emerg[5]         = status_message.emergency_bitmask & (1<<EMERGENCY_BUTTON2_BIT) ? 'B' : ' ';
    //ll_display_emerg[6]         = status_message.emergency_bitmask & (1<<EMERGENCY_BUTTON1_BIT) ? 'B' : ' ';
    //ll_display_emerg[7]         = status_message.emergency_bitmask & (1<<EMERGENCY_LATCH_BIT) ? 'E' : '0';
    //updateBlink(ll_display_emerg,ll_display_emerg_blink,8,displayUpdateCounter);

    //snprintf(display_line,17, "LL Emer %.8s",ll_display_emerg);
    //display.drawString(0, 2, display_line);

    // Rear motor
    display_motor_status[0]        = last_motor_state.status[0] & (1<<MOTOR_STATUS_BATTERY_DEAD | 1<<MOTOR_STATUS_BATTERY_L1  | 1<<MOTOR_STATUS_BATTERY_L2) ? 'V' : ' ';
    display_motor_status_blink[0]  = last_motor_state.status[0] & (1<<MOTOR_STATUS_BATTERY_DEAD) ? DISPLAY_FAST_BLINK : last_motor_state.status[0] & (1<<MOTOR_STATUS_BATTERY_L1) ? DISPLAY_NORM_BLINK : DISPLAY_SLOW_BLINK;
    display_motor_status[1]        = last_motor_state.status[0] & (1<<MOTOR_STATUS_GEN_TIMEOUT) ? 'G' : last_motor_state.status[0] & (1<<MOTOR_STATUS_ADC_TIMEOUT) ? 'A' : ' ';
    display_motor_status_blink[1]  = last_motor_state.status[0] & (1<<MOTOR_STATUS_GEN_TIMEOUT | 1<<MOTOR_STATUS_ADC_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[2]       = 'C';//connected
    display_motor_status_blink[2] = last_motor_state.status[0] & (1<<MOTOR_STATUS_CONN_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[3]       = last_motor_state.status[0] & (1<<MOTOR_STATUS_PCB_TEMP_ERR | 1<<MOTOR_STATUS_PCB_TEMP_WARN) ? 'T' : ' ';
    display_motor_status_blink[3] = last_motor_state.status[0] & (1<<MOTOR_STATUS_PCB_TEMP_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[4]       = last_motor_state.status[0] & (1<<MOTOR_STATUS_RIGHT_MOTOR_ERR | 1<<MOTOR_STATUS_RIGHT_MOTOR_TEMP_ERR) ? 'R' : ' ';
    display_motor_status_blink[4] = last_motor_state.status[0] & (1<<MOTOR_STATUS_RIGHT_MOTOR_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[5]       = last_motor_state.status[0] & (1<<MOTOR_STATUS_LEFT_MOTOR_ERR | 1<<MOTOR_STATUS_LEFT_MOTOR_TEMP_ERR) ? 'L' : ' ';
    display_motor_status_blink[5] = last_motor_state.status[0] & (1<<MOTOR_STATUS_LEFT_MOTOR_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[6]       = last_motor_state.status[0] & (1<<MOTOR_STATUS_BAD_CTRL_MODE) ? 'C' : ' ';
    display_motor_status_blink[6] = last_motor_state.status[0] & (1<<MOTOR_STATUS_BAD_CTRL_MODE) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[7]       = 'D';//drive on/disabled blink
    display_motor_status_blink[7] = last_motor_state.status[0] & (1<<MOTOR_STATUS_DISABLED) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    if(now - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS || last_motor_state.status_age_s[0]*1000 > MOTOR_STATUS_TIMEOUT_MS) {
        if(now - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS) {
            memcpy(display_motor_status," NO DATA",8);
        }
        for(int i=0;i<8;i++) display_motor_status_blink[i]=DISPLAY_SLOW_BLINK;
    }
    updateBlink(display_motor_status,display_motor_status_blink,8,displayUpdateCounter);

    snprintf(display_line,17, "Rear    %.8s",display_motor_status);
    display.drawString(0, 1, display_line);

    // Front motor
    display_motor_status[0]        = last_motor_state.status[1] & (1<<MOTOR_STATUS_BATTERY_DEAD | 1<<MOTOR_STATUS_BATTERY_L1  | 1<<MOTOR_STATUS_BATTERY_L2) ? 'V' : ' ';
    display_motor_status_blink[0]  = last_motor_state.status[1] & (1<<MOTOR_STATUS_BATTERY_DEAD) ? DISPLAY_FAST_BLINK : last_motor_state.status[1] & (1<<MOTOR_STATUS_BATTERY_L1) ? DISPLAY_NORM_BLINK : DISPLAY_SLOW_BLINK;
    display_motor_status[1]        = last_motor_state.status[1] & (1<<MOTOR_STATUS_GEN_TIMEOUT) ? 'G' : last_motor_state.status[1] & (1<<MOTOR_STATUS_ADC_TIMEOUT) ? 'A' : ' ';
    display_motor_status_blink[1]  = last_motor_state.status[1] & (1<<MOTOR_STATUS_GEN_TIMEOUT | 1<<MOTOR_STATUS_ADC_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[2]       = 'C';//connected
    display_motor_status_blink[2] = last_motor_state.status[1] & (1<<MOTOR_STATUS_CONN_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[3]       = last_motor_state.status[1] & (1<<MOTOR_STATUS_PCB_TEMP_ERR | 1<<MOTOR_STATUS_PCB_TEMP_WARN) ? 'T' : ' ';
    display_motor_status_blink[3] = last_motor_state.status[1] & (1<<MOTOR_STATUS_PCB_TEMP_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[4]       = last_motor_state.status[1] & (1<<MOTOR_STATUS_RIGHT_MOTOR_ERR | 1<<MOTOR_STATUS_RIGHT_MOTOR_TEMP_ERR) ? 'R' : ' ';
    display_motor_status_blink[4] = last_motor_state.status[1] & (1<<MOTOR_STATUS_RIGHT_MOTOR_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[5]       = last_motor_state.status[1] & (1<<MOTOR_STATUS_LEFT_MOTOR_ERR | 1<<MOTOR_STATUS_LEFT_MOTOR_TEMP_ERR) ? 'L' : ' ';
    display_motor_status_blink[5] = last_motor_state.status[1] & (1<<MOTOR_STATUS_LEFT_MOTOR_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[6]       = last_motor_state.status[1] & (1<<MOTOR_STATUS_BAD_CTRL_MODE) ? 'C' : ' ';
    display_motor_status_blink[6] = last_motor_state.status[1] & (1<<MOTOR_STATUS_BAD_CTRL_MODE) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[7]       = 'D';//drive on/disabled blink
    display_motor_status_blink[7] = last_motor_state.status[1] & (1<<MOTOR_STATUS_DISABLED) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    if(now - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS || last_motor_state.status_age_s[1]*1000 > MOTOR_STATUS_TIMEOUT_MS) {
        if(now - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS) {
            memcpy(display_motor_status," NO DATA",8);
        }
        for(int i=0;i<8;i++) display_motor_status_blink[i]=DISPLAY_SLOW_BLINK;
    }
    updateBlink(display_motor_status,display_motor_status_blink,8,displayUpdateCounter);

    snprintf(display_line,17, "Front    %.8s",display_motor_status);
    display.drawString(0, 2, display_line);

    //snprintf(display_line,17, "Main    %.8s",ll_display_emerg);
    //display.drawString(0, 2, display_line);

    snprintf(display_line,17, "V %4.1f %4.1f %4.1f",status_message.v_battery,status_message.v_charge,status_message.charging_current);
    display.drawString(0, 3, display_line);

    snprintf(display_line,17, "X %4.1f %4.1f %4.1f",imu_message.acceleration_mss[0],imu_message.acceleration_mss[1],imu_message.acceleration_mss[2]);
    display.drawString(0, 4, display_line);

    snprintf(display_line,17, "G %4.1f %4.1f %4.1f",imu_message.gyro_rads[0],imu_message.gyro_rads[1],imu_message.gyro_rads[2]);
    display.drawString(0, 5, display_line);

    snprintf(display_line,17, "USS %3d %3d %3d",(int)(status_message.uss_ranges_m[0]*100),(int)(status_message.uss_ranges_m[1]*100),(int)(status_message.uss_ranges_m[2]*100));
    display.drawString(0, 6, display_line);

    snprintf(display_line,17, "USS %3d %3d",(int)(status_message.uss_ranges_m[3]*100),(int)(status_message.uss_ranges_m[4]*100));
    display.drawString(0, 7, display_line);
}

/*void onUIPacketReceived(const uint8_t *buffer, size_t size) {

    u_int16_t *crc_pointer = (uint16_t *) (buffer + (size - 2));
    u_int16_t readcrc = *crc_pointer;

    // check structure size
    if (size < 4)
        return;

    // check the CRC
    uint16_t crc = CRC16.ccitt(buffer, size - 2);

    if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
        buffer[size - 2] != (crc & 0xFF))
        return;

    if (buffer[0] == Get_Button && size == sizeof(struct msg_event_button))
    {
        struct msg_event_button *msg = (struct msg_event_button *)buffer;
        struct ll_ui_event ui_event;
        ui_event.type = PACKET_ID_LL_UI_EVENT;
        ui_event.button_id = msg->button_id;
        ui_event.press_duration = msg->press_duration;
        sendMessage(&ui_event, sizeof(ui_event));
    }
}*/

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
#include "font.h"
#include "ant_bms/ant_bms.h"
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

#define UPLINK_SERIAL Serial1

#define BMS_SERIAL Serial2
//SerialPIO bmsSerial(PIN_BMS_TX, PIN_BMS_RX, 250);
//#define BMS_SERIAL bmsSerial

// Millis after charging is retried
#define CHARGING_RETRY_MS 10000

/**
 * @brief Some hardware parameters
 */
#define R_SHUNT 0.003f
#define CURRENT_SENSE_GAIN 60.0f

PacketSerial uplinkPacketSerial; // COBS communication PICO <> Raspi
FastCRC16 CRC16;

#ifdef BMS_SERIAL
    #define BMS_UPDATE_PERIOD_MS 2000
    #define BMS_RX_TIMEOUT_MS 200
    #define BMS_TIMEOUT_MS 5000

    ant_bms::AntBms ant_bms_parser;
    unsigned long last_bms_update_ms = 0;
#endif

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

//bool emergency_latch = true;
//bool ROS_running = false;
uint8_t emergency_high_level = 0;

// A mutex which is used by core1 each time status_message is modified.
// We can lock it during message transmission to prevent core1 to modify data in this time.
auto_init_mutex(mtx_status_message);

bool charging_allowed = false;
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
unsigned long last_uss_trigger_ms=0;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point
static int32_t chargeVoltageFixdt  = (0 * BAT_CELLS * CHARGE_CALIB_ADC) / CHARGE_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 0V converted to fixed-point
static int32_t chargeCurrentFixdt  = (0 * CURRENT_CALIB_ADC) / CURRENT_CALIB_REAL_CURRENT << 16;  // Fixed-point filter output initialized at 0 amps converted to fixed-point

#define DISPLAY_WIRE Wire
U8X8_SSD1309_128X64_NONAME0_HW_I2C display(U8X8_PIN_NONE);
//U8G2_SSD1309_128X64_NONAME0_2_HW_I2C display(U8G2_R0, PIN_DISPLAY_RESET, PIN_DISPLAY_SCK, PIN_DISPLAY_SDA);

void sendUplinkMessage(void *message, size_t size);
void updateDisplay(bool forceDisplay);
void updateBlinkState(char *message, uint8_t blinkState, int size, int currentBlinkState);
void onUplinkPacketReceived(const uint8_t *buffer, size_t size);

void bmsLogFunc(int level, const char * format, ...) {
#ifdef USB_DEBUG
    char *string;
    va_list args;
    va_start(args, format);
    // variadic printf with allocated string. must free()
    vasiprintf(&string, format, args);
    DEBUG_SERIAL.println(string);
    free(string);
    va_end(args);
#endif //USB_DEBUG
}

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
    uint8_t last_emergency_low_leve_state = status_message.emergency_bitmask;
    uint8_t emergency_low_level_state = 0;

    unsigned long now = millis();
    if (now - last_heartbeat_ms > HEARTBEAT_TIMEOUT_MS) {
        emergency_low_level_state |= 1<<EMERGENCY_ROS_TIMEOUT;
    }
    if (emergency_high_level!=0) {
        emergency_low_level_state |= 1<<EMERGENCY_HIGH_LEVEL;
    }

    // Mask the emergency bits. 2x Lift sensor, 2x Emergency Button
    bool emergency1 = !gpio_get(PIN_EMERGENCY_1);
    bool emergency2 = !gpio_get(PIN_EMERGENCY_2);
    bool emergency3 = !gpio_get(PIN_EMERGENCY_3);
    bool emergency4 = !gpio_get(PIN_EMERGENCY_4);

    /*#ifdef USB_DEBUG
        DEBUG_SERIAL.printf("Emergency %d %d %d %d\n",emergency1,emergency2,emergency3,emergency4);
    #endif*/
    now = millis();

    bool is_tilted = emergency3 || emergency4;
    bool is_lifted = emergency3 && emergency4;
    bool stop_pressed = emergency1 || emergency2;

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
        if (emergency3)
            emergency_low_level_state |= 1<<EMERGENCY_LIFT1_BIT;
        // Emergency bit 1 (lift wheel 2)set?
        if (emergency4)
            emergency_low_level_state |= 1<<EMERGENCY_LIFT2_BIT;
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
        if (emergency3)
            emergency_low_level_state |= 1<<EMERGENCY_LIFT1_BIT;
        // Emergency bit 1 (lift wheel 2)set?
        if (emergency4)
            emergency_low_level_state |= 1<<EMERGENCY_LIFT2_BIT;
    }
    if (button_emergency_started_ms > 0 && (now - button_emergency_started_ms) >= BUTTON_EMERGENCY_MS) {
        // Emergency bit 2 (stop button) set?
        if (emergency1)
            emergency_low_level_state |= 1<<EMERGENCY_BUTTON1_BIT;
        // Emergency bit 1 (stop button)set?
        if (emergency2)
            emergency_low_level_state |= 1<<EMERGENCY_BUTTON2_BIT;
    }

    //ESC must be enabled 
    setEscEnable(emergency_low_level_state==0);

    status_message.emergency_bitmask = emergency_low_level_state;

    // If emergency bits are chnaged, instantly send the message.
    if (last_emergency_low_leve_state != emergency_low_level_state) {
        sendUplinkMessage(&status_message, sizeof(struct ll_status));

        // Update LEDs instantly
        // updateDisplay(true);
    }
}

void setup1() {
    // Core
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop1_short_job(int max_job_length_us) {
    //here we can run some short (leass than USS_CYCLETIME_MS) tasks
}

void loop1() {
    // Loop through the mux and query actions. Store the result in the multicore fifo
    bool state;
    double distance;
    unsigned long duration;
    unsigned long wait_us,response_us,uss_measurement_age_ms;   

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
            unsigned long prev_uss_delta_ms = millis()-last_uss_trigger_ms;
            if(prev_uss_delta_ms < USS_CYCLETIME_MS) {
                wait_us = (USS_CYCLETIME_MS-prev_uss_delta_ms)*1000;

                //we can do a short job here ...
                loop1_short_job(wait_us);
                // and test again, how long we should sleep before next uss request
                prev_uss_delta_ms = millis()-last_uss_trigger_ms;
                if(prev_uss_delta_ms < USS_CYCLETIME_MS) {
                    wait_us = (USS_CYCLETIME_MS-prev_uss_delta_ms)*1000;
                } else {
                    //unfortunately short job was not short actually
                    wait_us = 5;
                }
            }else{
                wait_us = 5;
            }
            
            delayMicroseconds(wait_us);
            // Sets the trigPin on HIGH state for 20 micro seconds at least
            digitalWrite(PIN_MUX_OUT, HIGH); 
            delayMicroseconds(25);
            last_uss_trigger_ms = millis();
            digitalWrite(PIN_MUX_OUT, LOW);
            duration = pulseIn(PIN_MUX_IN, HIGH,USS_ECHO_TIMEOUT_US); // 35000UL for full cycle, Reads the echoPin, returns the sound wave travel time in microseconds
            distance = duration*0.343/2;//0.343mm per 1 microsecond
            response_us = millis()-last_uss_trigger_ms;
            #ifdef USB_DEBUG
                //DEBUG_SERIAL.printf("%i %icm %ius wt %ims rsp %ims\t",mux_address,(int)(distance/10),duration,wait_us/1000,response_us);
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
                    last_uss_sensor_result_ms[mux_address]=last_uss_trigger_ms;
                }
                uss_measurement_age_ms = millis() - last_uss_sensor_result_ms[mux_address];
                if(uss_measurement_age_ms > UINT16_MAX) {
                    uss_measurement_age_ms = UINT16_MAX;
                }
                status_message.uss_age_ms[mux_address] = uss_measurement_age_ms;
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

        DEBUG_SERIAL.println("Begin initialization");
    #endif

    lift_emergency_started_ms = 0;
    button_emergency_started_ms = 0;
    // Initialize messages
    imu_message = {0};
    status_message = {0};
    
    //initially there is no connection to ROS
    status_message.emergency_bitmask |= (1<<EMERGENCY_ROS_TIMEOUT);
    imu_message.type = PACKET_ID_LL_IMU;
    status_message.type = PACKET_ID_LL_STATUS;

    // Setup pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_ENABLE_CHARGE, OUTPUT);
    pinMode(PIN_RASPI_POWER,OUTPUT);
    pinMode(PIN_ESC_ENABLE, OUTPUT);

    //Init display I2C
    DISPLAY_WIRE.setSDA(PIN_DISPLAY_SDA);
    DISPLAY_WIRE.setSCL(PIN_DISPLAY_SCK);
    #ifdef USB_DEBUG
        DEBUG_SERIAL.println("Display init");
    #endif
    display.begin();
    #ifdef USB_DEBUG
        DEBUG_SERIAL.println("Load font");
    #endif
    display.setFont(u8x8_font_chroma48medium8_r);
    //display.setFont(mowerfont_u8x8_font_chroma48medium8_r);
    //display.setFont(mowerfont);//capital A - 6
   
    display.drawString(0,0,"Mover v0.2");
    delay(100);
    // Enable raspi power
    #ifdef USB_DEBUG
        DEBUG_SERIAL.println("Power RPi & ESC");
    #endif
    display.drawString(0,1,"Power RPi & ESC");
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
    UPLINK_SERIAL.begin(115200);
    uplinkPacketSerial.setStream(&UPLINK_SERIAL);
    uplinkPacketSerial.setPacketHandler(&onUplinkPacketReceived);

#ifdef BMS_SERIAL    
    /*if(!BMS_SERIAL.setPinout(PIN_BMS_TX,PIN_BMS_RX)){
        #ifdef USB_DEBUG
            DEBUG_SERIAL.println("Unable to set ");
        #endif
    }*/
    BMS_SERIAL.begin(19200);
    ant_bms_parser.set_stream(&BMS_SERIAL);
    ant_bms_parser.set_rx_timeout(BMS_RX_TIMEOUT_MS);
    ant_bms_parser.set_status_timeout(BMS_TIMEOUT_MS);
    ant_bms_parser.set_logFunc(bmsLogFunc);
#endif

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

void onUplinkPacketReceived(const uint8_t *buffer, size_t size) {
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
        struct ll_heartbeat *heartbeat = (struct ll_heartbeat *) buffer;
        last_heartbeat_ms = millis();
        emergency_high_level = heartbeat->emergency_high_level;
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
bool checkShouldStartCharge() {
    return status_message.v_battery < status_message.v_charge
        && status_message.v_charge < CHARGE_MAX_VOLT 
        && status_message.v_charge > CHARGE_MIN_VOLT 
        && (ant_bms_parser.is_online() 
                ? status_message.batt_percentage < BAT_SOC_START_CHARGE 
                : status_message.v_battery < BAT_OK_START_CHARGE);
}

bool checkShouldStopCharge() {
    return status_message.v_battery > status_message.v_charge
        || status_message.v_charge > CHARGE_MAX_VOLT 
        || status_message.v_charge < CHARGE_MIN_VOLT 
        //|| status_message.charging_current < 0.5
        || (ant_bms_parser.is_online() 
                ? status_message.batt_percentage > BAT_SOC_STOP_CHARGE 
                : status_message.v_battery > BAT_CHARGE_STOP);
}

void updateChargingEnabled() {
    if (charging_allowed) {
        if (checkShouldStopCharge()) {
            digitalWrite(PIN_ENABLE_CHARGE, LOW);
            charging_allowed = false;
            charging_disabled_time = millis();
        }
    } else {
        // enable charging after CHARGING_RETRY_MILLIS
        if (millis() - charging_disabled_time > CHARGING_RETRY_MS) {
            if (checkShouldStartCharge()) {
                digitalWrite(PIN_ENABLE_CHARGE, HIGH);
                charging_allowed = true;
            } else {
                digitalWrite(PIN_ENABLE_CHARGE, LOW);
                charging_allowed = false;
                charging_disabled_time = millis();
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

void readBatteryCurrent() {
    if(status_message.v_charge > CHARGE_MIN_VOLT) { //charger powers current sensor. no reason to read unpowered sensor
        uint16_t raw = analogRead(PIN_ANALOG_BATTERY_VOLTAGE);
        raw = analogRead(PIN_ANALOG_CHARGE_CURRENT);
        filtLowPass32(raw, CURRENT_FILT_COEF, &chargeCurrentFixdt);
        status_message.battery_current = (float)(chargeCurrentFixdt >> 16) * CURRENT_CALIB_REAL_CURRENT / (CURRENT_CALIB_ADC * 100.0);
    } else {
        //we don't know real battery discharge current
        status_message.battery_current = -0.2;
    }
}

void readBatteryVoltage() {
    uint16_t raw = analogRead(PIN_ANALOG_BATTERY_VOLTAGE);
    filtLowPass32(raw, BAT_FILT_COEF, &batVoltageFixdt);
    status_message.v_battery = (float)(batVoltageFixdt >> 16) * BAT_CALIB_REAL_VOLTAGE / (BAT_CALIB_ADC * 100.0);
}


void readChargerVoltage() {
    uint16_t raw = analogRead(PIN_ANALOG_CHARGE_VOLTAGE);
    filtLowPass32(raw, CHARGE_FILT_COEF, &chargeVoltageFixdt);
    status_message.v_charge = (float)(chargeVoltageFixdt >> 16) * CHARGE_CALIB_REAL_VOLTAGE / (CHARGE_CALIB_ADC * 100.0);
}


void loop() {
    uplinkPacketSerial.update();
#ifdef BMS_SERIAL
    ant_bms_parser.loop();
#endif
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
        sendUplinkMessage(&imu_message, sizeof(struct ll_imu));
    }
                
    if (now - last_status_update_ms > STATUS_CYCLETIME_MS) {
        status_message.status_bitmask &= ~(1 << STATUS_CHARGING_BIT);
        status_message.status_bitmask |= (charging_allowed & 0b1) << STATUS_CHARGING_BIT;

        // Check USS timouts
        status_message.status_bitmask &= ~(1 << STATUS_USS_TIMEOUT_BIT);
        for(int i=0;i<USS_COUNT;i++) {
            if( (USS_ENABLED & (1<<i)) && (status_message.uss_age_ms[i] > USS_TIMEOUT_MS)) {
                status_message.status_bitmask |= 1<<STATUS_USS_TIMEOUT_BIT;
            }
        }

        // Check IMU temeout
        status_message.status_bitmask &= ~(1 << STATUS_IMU_TIMEOUT_BIT);
        if( now - last_imu_ms > IMU_TIMEOUT_MS ) {
            status_message.status_bitmask |= 1<<STATUS_IMU_TIMEOUT_BIT;
        }

        // Check BMS temeout
#ifdef BMS_SERIAL        
        status_message.status_bitmask &= ~(1 << STATUS_BMS_TIMEOUT_BIT);
        if( !ant_bms_parser.is_online() ) {
            status_message.status_bitmask |= 1<<STATUS_BMS_TIMEOUT_BIT;
#endif //BMS_SERIAL        
            readChargerVoltage();
            readBatteryVoltage();
            readBatteryCurrent();

            // calculate percent value accu filling
            float delta = BAT_FULL - BAT_DISCHARGE_CUT_OFF;
            float vo = status_message.v_battery - BAT_DISCHARGE_CUT_OFF;
            status_message.batt_percentage = vo / delta * 100;
#ifdef BMS_SERIAL        
        } else {
            readChargerVoltage();
            readBatteryVoltage();
            //status_message.v_battery = ant_bms_parser.total_voltage_sensor_;
            status_message.battery_current = -ant_bms_parser.current_sensor_;//reverse current sensor
            status_message.batt_percentage = ant_bms_parser.soc_sensor_;
        }
#endif //BMS_SERIAL
        if (status_message.batt_percentage > 100) {
            status_message.batt_percentage = 100;
        }

        // Check battery empty
        status_message.status_bitmask &= ~(1 << STATUS_BATTERY_EMPTY_BIT);
        if (status_message.v_battery < BAT_WARN3) {
            status_message.status_bitmask |= 1<<STATUS_BATTERY_EMPTY_BIT;
        }

        mutex_enter_blocking(&mtx_status_message);
        sendUplinkMessage(&status_message, sizeof(struct ll_status));
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

#endif //USB_DEBUG
    }

    if (now - last_display_ms > DISPLAY_CYCLETIME_MS) {
        updateDisplay(false);
        last_display_ms = now;
    }

#ifdef BMS_SERIAL
    if (now - last_bms_update_ms > BMS_UPDATE_PERIOD_MS) {
        ant_bms_parser.update();
        last_bms_update_ms = now;
    }
#endif //BMS_SERIAL
}

void sendUplinkMessage(void *message, size_t size) {
    // Only send messages, if ROS is running, else Raspi sometimes doesn't boot
    if (status_message.emergency_bitmask&(1<<EMERGENCY_ROS_TIMEOUT)) {
        return;
    }

    // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC
    if (size < 4) {
        return;
    }
    uint8_t *data_pointer = (uint8_t *) message;

    // calculate the CRC
    uint16_t crc = CRC16.ccitt((uint8_t *) message, size - 2);
    data_pointer[size - 1] = (crc >> 8) & 0xFF;
    data_pointer[size - 2] = crc & 0xFF;

    uplinkPacketSerial.send((uint8_t *) message, size);
}

void updateBlink(char *message, uint8_t *blinkState, int size,int currentBlinkCounter) {
    for(int i=0;i<size;i++){
        if(currentBlinkCounter & blinkState[i]) {
            message[i] = ' ';
        }
    }
}

void updateDisplayLine1(uint32_t now_ms) {
    uint32_t high_level_age_ms = now_ms - last_high_level_ms;
    
    status_line[0]         = emergency_high_level!=0 ? 'E' : 'T';
    status_line_blink[0]   = status_message.emergency_bitmask & (1<<EMERGENCY_ROS_TIMEOUT) ? DISPLAY_SLOW_BLINK : emergency_high_level!=0 ? DISPLAY_FAST_BLINK : DISPLAY_NO_BLINK;

    //GPS status
    status_line[1]         = high_level_age_ms < HIGH_LEVEL_TIMEOUT_MS ? 'G' : 'g';
    if ( high_level_age_ms > HIGH_LEVEL_TIMEOUT_MS) { status_line_blink[1]=DISPLAY_SLOW_BLINK; }
    else if (last_high_level_state.gps_quality < 25) { status_line_blink[1]=DISPLAY_FAST_BLINK; }
    else if (last_high_level_state.gps_quality < 50) { status_line_blink[1]=DISPLAY_NORM_BLINK; }
    else if (last_high_level_state.gps_quality < 75) { status_line_blink[1]=DISPLAY_SLOW_BLINK; }
    else { status_line_blink[1]=DISPLAY_NO_BLINK; }

    //High level mode status
    if(high_level_age_ms < HIGH_LEVEL_TIMEOUT_MS) {
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
    status_line[7]         = status_message.emergency_bitmask !=0 ? 'E' : ' ';
    status_line_blink[7]   = DISPLAY_FAST_BLINK;
    //Rain sensor
    status_line[8]         = status_message.status_bitmask & (1<<STATUS_RAIN_BIT) ? 'R' : ' ';
    status_line_blink[8]   = DISPLAY_FAST_BLINK;
    //USS timeout
    status_line[9]         = status_message.status_bitmask & (1<<STATUS_USS_TIMEOUT_BIT) ? 'U' : ' ';
    status_line_blink[9]   = DISPLAY_SLOW_BLINK;
    //IMU timeout
    status_line[10]         = status_message.status_bitmask & (1<<STATUS_IMU_TIMEOUT_BIT) ? 'I' : ' ';
    status_line_blink[10]   = DISPLAY_FAST_BLINK;

    //Output status group 11-12 (2 sybmols)
    //Power pin
    status_line[11]        = 'P';
    status_line_blink[12]        = status_message.status_bitmask & (1<<STATUS_RASPI_POWER_BIT) ? DISPLAY_NO_BLINK : DISPLAY_SLOW_BLINK;
    //ESC enable
    status_line[12]        = 'M';
    status_line_blink[12]        = status_message.status_bitmask & (1<<STATUS_ESC_ENABLED_BIT) ? DISPLAY_NO_BLINK : DISPLAY_SLOW_BLINK;
    
    //Charging and battery status 12-15 (4 symbols)
    status_line[13]        = status_message.status_bitmask & (1<<STATUS_CHARGING_BIT) ? 'C' : ' ';
    uint8_t batt_percentage = status_message.batt_percentage >= 100 ? 99 : status_message.batt_percentage < 0 ? 0 : status_message.batt_percentage;
    status_line[14]        = '0'+batt_percentage/10;
    status_line[15]        = '0'+batt_percentage%10;
    status_line_blink[14]  = status_message.status_bitmask & (1<<STATUS_BMS_TIMEOUT_BIT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    status_line_blink[15]  = status_message.status_bitmask & (1<<STATUS_BMS_TIMEOUT_BIT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    updateBlink(status_line,status_line_blink,16,displayUpdateCounter);

    snprintf(display_line,17, "%.16s",status_line);
    display.drawString(0, 0, display_line);
}

void updateDisplayLine2(uint32_t now_ms){
    display_motor_status[0]       = last_motor_state.status[0] & (1<<MOTOR_STATUS_BATTERY_DEAD | 1<<MOTOR_STATUS_BATTERY_L1  | 1<<MOTOR_STATUS_BATTERY_L2) ? 'V' : ' ';
    display_motor_status_blink[0] = last_motor_state.status[0] & (1<<MOTOR_STATUS_BATTERY_DEAD) ? DISPLAY_FAST_BLINK : last_motor_state.status[0] & (1<<MOTOR_STATUS_BATTERY_L1) ? DISPLAY_NORM_BLINK : DISPLAY_SLOW_BLINK;
    display_motor_status[1]       = last_motor_state.status[0] & (1<<MOTOR_STATUS_GEN_TIMEOUT) ? 'G' : last_motor_state.status[0] & (1<<MOTOR_STATUS_ADC_TIMEOUT) ? 'A' : ' ';
    display_motor_status_blink[1] = last_motor_state.status[0] & (1<<MOTOR_STATUS_GEN_TIMEOUT | 1<<MOTOR_STATUS_ADC_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
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
    if(now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS || last_motor_state.status_age_s[0]*1000 > MOTOR_STATUS_TIMEOUT_MS) {
        if(now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS) {
            memcpy(display_motor_status," NO DATA",8);
        }
        for(int i=0;i<8;i++) display_motor_status_blink[i]=DISPLAY_SLOW_BLINK;
    }
    updateBlink(display_motor_status,display_motor_status_blink,8,displayUpdateCounter);

    snprintf(display_line,17, "Rear    %.8s",display_motor_status);
    display.drawString(0, 1, display_line);
}

void updateDisplayLine3(uint32_t now_ms) {
    display_motor_status[0]       = last_motor_state.status[1] & (1<<MOTOR_STATUS_BATTERY_DEAD | 1<<MOTOR_STATUS_BATTERY_L1  | 1<<MOTOR_STATUS_BATTERY_L2) ? 'V' : ' ';
    display_motor_status_blink[0] = last_motor_state.status[1] & (1<<MOTOR_STATUS_BATTERY_DEAD) ? DISPLAY_FAST_BLINK : last_motor_state.status[1] & (1<<MOTOR_STATUS_BATTERY_L1) ? DISPLAY_NORM_BLINK : DISPLAY_SLOW_BLINK;
    display_motor_status[1]       = last_motor_state.status[1] & (1<<MOTOR_STATUS_GEN_TIMEOUT) ? 'G' : last_motor_state.status[1] & (1<<MOTOR_STATUS_ADC_TIMEOUT) ? 'A' : ' ';
    display_motor_status_blink[1] = last_motor_state.status[1] & (1<<MOTOR_STATUS_GEN_TIMEOUT | 1<<MOTOR_STATUS_ADC_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
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
    if( now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS || last_motor_state.status_age_s[1]*1000 > MOTOR_STATUS_TIMEOUT_MS) {
        if(now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS) {
            memcpy(display_motor_status," NO DATA",8);
        }
        for(int i=0;i<8;i++) display_motor_status_blink[i]=DISPLAY_SLOW_BLINK;
    }
    updateBlink(display_motor_status,display_motor_status_blink,8,displayUpdateCounter);

    snprintf(display_line,17, "Front   %.8s",display_motor_status);
    display.drawString(0, 2, display_line);
}

void updateDisplayLine4(uint32_t now_ms) {
    display_motor_status[0]       = last_motor_state.status[2] & (XESC_FAULT_OVERVOLTAGE | XESC_FAULT_UNDERVOLTAGE)? 'V' : ' ';
    display_motor_status_blink[0] = last_motor_state.status[2] & XESC_FAULT_OVERVOLTAGE ? DISPLAY_FAST_BLINK : DISPLAY_SLOW_BLINK;
    display_motor_status[1]       = last_motor_state.status[2] & XESC_FAULT_OVERCURRENT ? 'A' : ' ';
    display_motor_status_blink[1] = DISPLAY_FAST_BLINK;
    display_motor_status[2]       = last_motor_state.status[2] & XESC_FAULT_UNINITIALIZED ? 'E' : ' ';
    display_motor_status_blink[2] = DISPLAY_FAST_BLINK;
    display_motor_status[3]       = last_motor_state.status[2] & XESC_FAULT_OVERTEMP_PCB ? 'T' : ' ';
    display_motor_status_blink[3] = DISPLAY_FAST_BLINK;
    display_motor_status[4]       = last_motor_state.status[2] & XESC_FAULT_OVERTEMP_MOTOR ? 'M' : ' ';
    display_motor_status_blink[4] = DISPLAY_FAST_BLINK;
    display_motor_status[5]       = last_motor_state.status[2] & XESC_FAULT_INVALID_HALL ? 'H' : ' ';
    display_motor_status_blink[5] = DISPLAY_FAST_BLINK;
    display_motor_status[6]       = last_motor_state.status[2] & XESC_FAULT_WATCHDOG ? 'W' : ' ';
    display_motor_status_blink[6] = DISPLAY_FAST_BLINK;
    display_motor_status[7]       = 'D';//drive on/disabled blink
    display_motor_status_blink[7] = last_motor_state.status[2] != 0 ? DISPLAY_FAST_BLINK : DISPLAY_NO_BLINK;
    if( now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS || last_motor_state.status_age_s[2]*1000 > MOTOR_STATUS_TIMEOUT_MS) {
        if(now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS) {
            memcpy(display_motor_status," NO DATA",8);
        }
        for(int i=0;i<8;i++) display_motor_status_blink[i]=DISPLAY_SLOW_BLINK;
    }
    updateBlink(display_motor_status,display_motor_status_blink,8,displayUpdateCounter);

    snprintf(display_line,17, "Mow     %.8s",display_motor_status);
    display.drawString(0, 3, display_line);
}

void updateDisplayLine5(uint32_t now_ms) {
    snprintf(display_line,17, "V %4.1f %4.1f %4.2f",status_message.v_battery,status_message.v_charge,status_message.battery_current);
    display.drawString(0, 4, display_line);
}

void updateDisplayLine6(uint32_t now_ms) {
    double acceleration = sqrt(imu_message.acceleration_mss[0]*imu_message.acceleration_mss[0]+
                               imu_message.acceleration_mss[1]*imu_message.acceleration_mss[1]+
                               imu_message.acceleration_mss[2]*imu_message.acceleration_mss[2]);
    double gyro = abs(imu_message.gyro_rads[0]) + abs(imu_message.gyro_rads[1]) + abs(imu_message.gyro_rads[2]);
    snprintf(display_line,17, "X %4.1f G %4.1f   ",acceleration,gyro);
    display.drawString(0, 5, display_line);
}

void updateDisplayLine78(uint32_t now_ms) {
    snprintf(display_line,17, "USS %3d %3d %3d",(int)(status_message.uss_ranges_m[0]*100),(int)(status_message.uss_ranges_m[1]*100),(int)(status_message.uss_ranges_m[2]*100));
    display.drawString(0, 6, display_line);

    snprintf(display_line,17, "USS %3d %3d",(int)(status_message.uss_ranges_m[3]*100),(int)(status_message.uss_ranges_m[4]*100));
    display.drawString(0, 7, display_line);
}

void updateDisplay(bool forceDisplay) {
    uint32_t now_ms = millis();

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
    updateDisplayLine1(now_ms);

    // Rear motor
    updateDisplayLine2(now_ms);

    // Front motor
    updateDisplayLine3(now_ms);

    // Mower motor
    updateDisplayLine4(now_ms);

    //snprintf(display_line,17, "Main    %.8s",ll_display_emerg);
    //display.drawString(0, 2, display_line);
    updateDisplayLine5(now_ms);

    updateDisplayLine6(now_ms);

    //snprintf(display_line,17, "ADC %d %d ",batteryADCRaw,chargerADCRaw);
    //display.drawString(0, 6, display_line);

    //snprintf(display_line,17, "ADC %d       ",currentADCRaw);
    //display.drawString(0, 7, display_line);

    updateDisplayLine78(now_ms);

    #ifdef USB_DEBUG
        DEBUG_SERIAL.printf("Update display %dms\n",millis()-now_ms);
    #endif
}

/*void setUSSChars(int distance, char *c1,char *c2, bool up) {
    int EMPTY = 95;
    int HIGH_CHAR = 104;
    int LOW_CHAR = 97;
    if(up) {
        
    }
}*/

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

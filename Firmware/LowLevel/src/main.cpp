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
#include "EEPROMFixed.h"
#include "datatypes.h"
#include "defines.h"
#include "pins.h"
#include "imu.h"
#include "font.h"
#include "conf.h"
#include "ant_bms/ant_bms.h"
#include <U8x8lib.h>
//#include <U8g2lib.h>
#include <Wire.h>

#define IMU_CYCLETIME_MS 20              // cycletime for refresh IMU data
#define STATUS_CYCLETIME_MS 100          // cycletime for refresh analog and digital Statusvalues
#define DISPLAY_CYCLETIME_MS 200        // cycletime for refresh UI display

// Emergency will be engaged, if no heartbeat was received in this time frame.
#define HEARTBEAT_TIMEOUT_MS 500
#define HIGH_LEVEL_TIMEOUT_MS 5000
#define MOTOR_STATUS_TIMEOUT_MS 500
#define IMU_TIMEOUT_MS 200

#define USS_TIMEOUT_MS 500
#define USS_CYCLETIME_MS 50
#define USS_ECHO_TIMEOUT_US 35000UL

//#define USS_TRIGGER_INACTIVE_LOW

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

uint32_t last_imu_ms = 0;
uint32_t last_status_update_ms = 0;
uint32_t last_heartbeat_ms = 0;
uint32_t last_high_level_ms = 0;
uint32_t last_motor_ms = 0;
uint32_t last_display_ms = 0;
uint8_t last_display_line = 7;

uint32_t contact_started_ms[CONTACT_COUNT] = {0,0,0,0};
uint8_t contact_pins[CONTACT_COUNT] = {PIN_EMERGENCY_1,PIN_EMERGENCY_2,PIN_EMERGENCY_3,PIN_EMERGENCY_4};
uint8_t contact_emergency_bits[CONTACT_COUNT] = {EMERGENCY_CONTACT1_BIT,EMERGENCY_CONTACT2_BIT,EMERGENCY_CONTACT3_BIT,EMERGENCY_CONTACT4_BIT};

// Predefined message buffers, so that we don't need to allocate new ones later.
struct ll_imu imu_message = {0};
struct ll_status status_message = {0};
// current high level state
struct ll_high_level_state last_high_level_state = {0};
struct ll_motor_state last_motor_state = {0};

//bool emergency_latch = true;
uint8_t power_request = 0;
uint8_t emergency_high_level = 0;

// A mutex which is used by core1 each time status_message is modified.
// We can lock it during message transmission to prevent core1 to modify data in this time.
auto_init_mutex(mtx_status_message);

bool charging_allowed = false;
uint32_t charging_disabled_time = 0;

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

//configuration parameters
uint8_t conf_charge_start_soc = 0;
float conf_charge_start_voltage = 0;//BAT_OK_START_CHARGE

uint8_t conf_charge_stop_soc = 0;
float conf_charge_stop_voltage = 0;//BAT_CHARGE_STOP

float conf_charge_max_current = 0;
float conf_charge_stop_current = 0;

float conf_charger_max_voltage = 0;
float conf_charger_min_voltage = 0;

float conf_charge_min_battery_temperature = 0;
float conf_charge_max_battery_temperature = 0;
float conf_charge_stop_balancer_temperature = 0;

float conf_battery_full_voltage = 0;//BAT_FULL
float conf_battery_empty_voltage = 0;//BAT_DISCHARGE_CUT_OFF

uint8_t conf_battery_shutdown_soc = 0;
float conf_battery_shutdown_voltage = 0;//BAT_WARN3

uint8_t conf_uss_enabled[USS_COUNT] = {false,false,false,false,false};
bool conf_contact_active_low[CONTACT_COUNT] = {true,true,true,true};
uint8_t conf_contact_mode[CONTACT_COUNT] = {ContactMode::OFF,ContactMode::OFF,ContactMode::OFF,ContactMode::OFF};
uint8_t conf_contact_timeout_ms[CONTACT_COUNT] = {20,20,20,20};

bool conf_valid;

uint32_t last_uss_sensor_result_ms[USS_COUNT];
uint32_t last_uss_trigger_ms=0;
static int32_t batVoltageFixdt  = (4000 * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 40V converted to fixed-point
static int32_t chargeVoltageFixdt  = (0 * CHARGE_CALIB_ADC) / CHARGE_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 0V converted to fixed-point
static int32_t chargeCurrentFixdt  = (0 * CURRENT_CALIB_ADC) / CURRENT_CALIB_REAL_CURRENT << 16;  // Fixed-point filter output initialized at 0 amps converted to fixed-point

#define DISPLAY_WIRE Wire
U8X8_SSD1309_128X64_NONAME0_HW_I2C display(U8X8_PIN_NONE);
//U8G2_SSD1309_128X64_NONAME0_2_HW_I2C display(U8G2_R0, PIN_DISPLAY_RESET, PIN_DISPLAY_SCK, PIN_DISPLAY_SDA);

void sendUplinkMessage(void *message, size_t size);
void updateDisplay(uint32_t now_ms);
void updateBlinkState(char *message, uint8_t blinkState, int size, int currentBlinkState);
void onUplinkPacketReceived(const uint8_t *buffer, size_t size);
bool loadConfiguration(char *conf_message);
void resetConfiguration();
ConfigValue &eepromGet(const uint8_t address,const uint8_t address2,ConfigValue &val);
ConfigValue &eepromGet(const uint8_t address,ConfigValue &val);
const ConfigValue &eepromPut(const uint8_t address,const uint8_t address2,const ConfigValue &val);
const ConfigValue &eepromPut(const uint8_t address,const ConfigValue &val);


void bmsLogFunc(int level, const char * format, ...) {
    if(level > ESPHOME_LOG_LEVEL_CONFIG) {
        return;
    }
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

void updateEscEnabled() {
    //ESC must be enabled 
    uint8_t espPowerRequest = (power_request >> (POWER_REQUEST_MOTOR<<1)) & 0x3;//two bits at desired position
    setEscEnable(
        status_message.emergency_bitmask==0 && 
        espPowerRequest == POWER_REQUEST_BITS_ON /*&& 
        last_high_level_state.current_mode != HighLevelMode::MODE_IDLE*/
    );
}

void updateEmergency() {
    uint8_t last_emergency_low_leve_state = status_message.emergency_bitmask;
    uint8_t emergency_low_level_state = 0;

    uint32_t now = millis();
    if (now - last_heartbeat_ms > HEARTBEAT_TIMEOUT_MS) {
        emergency_low_level_state |= 1<<EMERGENCY_ROS_TIMEOUT;
    }
    if (now - last_high_level_ms > HIGH_LEVEL_TIMEOUT_MS) {
        emergency_low_level_state |= 1<<EMERGENCY_ROS_TIMEOUT;
    }
    if (emergency_high_level!=0) {
        emergency_low_level_state |= 1<<EMERGENCY_HIGH_LEVEL;
    }

    // Check contact and set emergency or contact bits.
    for(int i=0;i<CONTACT_COUNT;i++) {
        if(conf_contact_mode[i]==ContactMode::OFF) {
            continue;
        }
        bool contact = gpio_get(contact_pins[i]) ^ conf_contact_active_low[i];
        if(contact) {
            if(contact_started_ms[i]==0){
                contact_started_ms[i] = now;
            }
        } else {
            contact_started_ms[i] = 0;
            status_message.contacts &= ~ (1<<i);
        }
        if(contact_started_ms[i] > 0 && (now - contact_started_ms[i]) >= conf_contact_timeout_ms[i]){
            //real event
            if(conf_contact_mode[i]==ContactMode::EMERGENCY_STOP) {
                emergency_low_level_state |= 1<<contact_emergency_bits[i];
            } else if(conf_contact_mode[i]==ContactMode::MONITOR) {
                //Do nothing, just report in status_message.contacts
            }
            status_message.contacts |= (1<<i);
        }
    }

    /*#ifdef USB_DEBUG
        DEBUG_SERIAL.printf("Emergency %d %d %d %d\n",emergency1,emergency2,emergency3,emergency4);
    #endif*/

    status_message.emergency_bitmask = emergency_low_level_state;
    updateEscEnabled();

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

void loop1_short_job(uint32_t now_ms, uint32_t max_job_length_us) {
    //here we can run some short (less than USS_CYCLETIME_MS) tasks
    //if (max_job_length_us>10000) {
    //updateDisplay(now_ms);
    //}
}

void loop1() {
    // Loop through the mux and query actions. Store the result in the multicore fifo
    bool state;
    double distance;
    uint32_t duration;
    uint32_t wait_us,uss_measurement_age_ms;
    //uint32_t response_us;   

    for (uint8_t mux_address = 0; mux_address < 7; mux_address++) {
       
        if(mux_address<USS_COUNT) {
            if ( !conf_uss_enabled[mux_address] ) {
                duration = 0;
                distance = 0;
                continue;
            }
            gpio_put_masked(0b111 << 13, mux_address << 13);

            uint32_t now = millis();
            uint32_t prev_uss_delta_ms = now - last_uss_trigger_ms;
            if(prev_uss_delta_ms < USS_CYCLETIME_MS) {
                wait_us = (USS_CYCLETIME_MS-prev_uss_delta_ms)*1000;

                //we can do a short job here ...
                loop1_short_job(now, wait_us);
                // and test again, how long we should sleep before next uss request
                now = millis();
                prev_uss_delta_ms = now - last_uss_trigger_ms;
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

            #ifdef USS_TRIGGER_INACTIVE_LOW    
                //delay(1);
                //digitalWrite(PIN_MUX_OUT, LOW); 
                //delayMicroseconds(USS_ECHO_TIMEOUT_US*2);
                // Sets the trigPin on HIGH state for 20 micro seconds at least
                digitalWrite(PIN_MUX_OUT, HIGH); 
                delayMicroseconds(25);
                digitalWrite(PIN_MUX_OUT, LOW);
            #else
                //delay(1);
                digitalWrite(PIN_MUX_OUT, LOW); 
            #endif

            last_uss_trigger_ms = millis();
            duration = pulseIn(PIN_MUX_IN, HIGH,USS_ECHO_TIMEOUT_US); // 35000UL for full cycle, Reads the echoPin, returns the sound wave travel time in microseconds
            //distance = duration*0.1725;//distance in meters (0.343mm per 1 microsecond / 2)
            distance = duration*0.01725;//distance in centimeters (0.0343cm per 1 microsecond / 2)
            #ifdef USS_TRIGGER_INACTIVE_LOW
            #else
                digitalWrite(PIN_MUX_OUT, HIGH); 
            #endif

            //response_us = millis() - last_uss_trigger_ms;
            //#ifdef USB_DEBUG
            //    DEBUG_SERIAL.printf("%i %icm %ius wt %ims rsp %ims\t",mux_address,(int)(distance/10),duration,wait_us/1000,response_us);
            //#endif
        } else {
            gpio_put_masked(0b111 << 13, mux_address << 13);
            delay(1);

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
                    distance = distance;
                    if(distance>255){
                        distance = 255;
                    }
                    status_message.uss_ranges_cm[mux_address]=(uint8_t)(lround(distance));
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
    //#ifdef USB_DEBUG
    //    DEBUG_SERIAL.println();
    //#endif

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

    EEPROMFixed.begin(256*sizeof(union ConfigValue));
    ConfigValue eeprom_config_version;
    eepromGet(ConfigAddress::EEPROM_STATUS,eeprom_config_version);
    if(eeprom_config_version.int32Value!=EEPROM_CONFIG_VERSION) {
        #ifdef USB_DEBUG
            DEBUG_SERIAL.println("Reset coniguration");
        #endif
        resetConfiguration();
        eeprom_config_version.int32Value=EEPROM_CONFIG_VERSION;
        eepromPut(ConfigAddress::EEPROM_STATUS,eeprom_config_version);
        #ifdef USB_DEBUG
            DEBUG_SERIAL.println("Commit");
        #endif
        if(!EEPROMFixed.commitFixed()){
            #ifdef USB_DEBUG
                DEBUG_SERIAL.println("Fail to commit");
            #endif
        }
    }else{
        #ifdef USB_DEBUG
            DEBUG_SERIAL.println("Coniguration version match");
        #endif
    }

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
            updateDisplay(millis());
            delay(20);
        }
    }

    display.drawString(0,3,"IMU success");
#ifdef USB_DEBUG
    DEBUG_SERIAL.println("IMU initialized");
#endif
    delay(100);

    char conf_message[17];
    if(loadConfiguration(conf_message)) {
        display.drawString(0,4,"Valid config");    
    }else{
        display.drawString(0,4,conf_message);    
        delay(5000);
    }

    status_message.status_bitmask |= 1 << STATUS_INIT_BIT;

    rp2040.resumeOtherCore();

    display.drawString(0,5,"Init completed");
    delay(500);
    display.clear();
    #ifdef USB_DEBUG
        DEBUG_SERIAL.println("Init completed");
    #endif
}

void onConfigPacketReceived(ll_high_level_config *request) {
    //int realAddress = (int)(request->address) + 4*request->address2;
    //realAddress*=sizeof(union ConfigValue);
    if (request->address==ConfigAddress::USS_ACTIVE 
        && request->address2>USS_COUNT) {
        request->type==PACKET_ID_LL_HIGH_LEVEL_CONFIG_ERR;
        #ifdef USB_DEBUG
            DEBUG_SERIAL.printf("ConfigRsp: incorrect USS address %d,%d\n",request->address,request->address2);
        #endif
        sendUplinkMessage(request,sizeof(ll_high_level_config));
        return;
    } else if ((request->address==ConfigAddress::CONTACT_ACTIVE_LOW 
            || request->address==ConfigAddress::CONTACT_MODE 
            || request->address==ConfigAddress::CONTACT_TIMEOUT)
             && request->address2>CONTACT_COUNT) {
        request->type==PACKET_ID_LL_HIGH_LEVEL_CONFIG_ERR;
        #ifdef USB_DEBUG
            DEBUG_SERIAL.printf("ConfigRsp: incorrect contact address %d,%d\n",request->address,request->address2);
        #endif
        sendUplinkMessage(request,sizeof(ll_high_level_config));
        return;
    } else if (request->address2!=0) {
        request->type==PACKET_ID_LL_HIGH_LEVEL_CONFIG_ERR;
        #ifdef USB_DEBUG
            DEBUG_SERIAL.printf("ConfigRsp: incorrect address2 %d,%d\n",request->address,request->address2);
        #endif
        sendUplinkMessage(request,sizeof(ll_high_level_config));
        return;        
    }

    if (request->type == PACKET_ID_LL_HIGH_LEVEL_CONFIG_GET) {
        ConfigValue oldValue;
        eepromGet(request->address,request->address2,oldValue);
        request->value = oldValue;
        #ifdef USB_DEBUG
            DEBUG_SERIAL.printf("ConfigRsp: reading address %d,%d=%d\n",request->address,request->address2,request->value.int32Value);
        #endif
        sendUplinkMessage(request,sizeof(ll_high_level_config));
    } else if (request->type == PACKET_ID_LL_HIGH_LEVEL_CONFIG_SET) {
        if (request->address==ConfigAddress::EEPROM_STATUS) {
            request->type==PACKET_ID_LL_HIGH_LEVEL_CONFIG_ERR;
            #ifdef USB_DEBUG
                DEBUG_SERIAL.printf("ConfigRsp: read only address %d,%d\n",request->address,request->address2);
            #endif
            sendUplinkMessage(request,sizeof(ll_high_level_config));
            return;
        } else if (request->address==ConfigAddress::COMMAND) {
            uint8_t command = request->value.int8Value;
            if (command==ConfigCommand::CONFIGURATION_LOAD || command==ConfigCommand::CONFIGURATION_SAVE) {
                char conf_message[17];
                if (loadConfiguration(conf_message)){
                    #ifdef USB_DEBUG
                        DEBUG_SERIAL.printf("ConfigRsp: load success %d,%d\n",request->address,request->address2);
                    #endif
                    if (command == ConfigCommand::CONFIGURATION_SAVE){
                        #ifdef USB_DEBUG
                            DEBUG_SERIAL.printf("ConfigRsp: commit ...\n");
                        #endif
                        //use regular comit to stop other core 
                        if(!EEPROMFixed.commit()){
                            #ifdef USB_DEBUG
                                DEBUG_SERIAL.printf("ConfigRsp: commit failed\n");
                            #endif
                        }else{
                            #ifdef USB_DEBUG
                                DEBUG_SERIAL.printf("ConfigRsp: commit succeded\n");
                            #endif
                        }
                    }
                    sendUplinkMessage(request,sizeof(ll_high_level_config));
                } else {
                    request->type==PACKET_ID_LL_HIGH_LEVEL_CONFIG_ERR;
                    #ifdef USB_DEBUG
                        DEBUG_SERIAL.printf("ConfigRsp: load error %d,%d\n",request->address,request->address2);
                    #endif
                    sendUplinkMessage(request,sizeof(ll_high_level_config));
                    return;
                }
            } else {
                request->type==PACKET_ID_LL_HIGH_LEVEL_CONFIG_ERR;
                #ifdef USB_DEBUG
                    DEBUG_SERIAL.printf("ConfigRsp: unknown command %d,%d=%d\n",request->address,request->address2,request->value.int32Value);
                #endif
                sendUplinkMessage(request,sizeof(ll_high_level_config));
                return;
            }
        } else {
            ConfigValue oldValue;
            eepromGet(request->address,request->address2,oldValue);
            if(oldValue.int32Value == request->value.int32Value) {
                request->type = PACKET_ID_LL_HIGH_LEVEL_CONFIG_GET;
                #ifdef USB_DEBUG
                    DEBUG_SERIAL.printf("ConfigRsp: no changes %d,%d=%d\n",request->address,request->address2,request->value.int32Value);
                #endif
                sendUplinkMessage(request,sizeof(ll_high_level_config));
            } else {
                eepromPut(request->address,request->address2,request->value);
                #ifdef USB_DEBUG
                    DEBUG_SERIAL.printf("ConfigRsp: set %d,%d=%d\n",request->address,request->address2,request->value.int32Value);
                #endif
                sendUplinkMessage(request,sizeof(ll_high_level_config));
            }
        }
    }
    
}

void onUplinkPacketReceived(const uint8_t *buffer, size_t size) {
    // sanity check for CRC to work (1 type, 1 data, 2 CRC)
    if (size < 4)
        return;

    // check the CRC
    uint16_t crc = CRC16.ccitt(buffer, size - 2);

    if (buffer[size - 1] != ((crc >> 8) & 0xFF) ||
        buffer[size - 2] != (crc & 0xFF)) {
        #ifdef USB_DEBUG
            DEBUG_SERIAL.printf("got invalid packet %d size %d\n",(int)buffer[0],(int)size);
        #endif       
        return;
    }

    if (buffer[0] == PACKET_ID_LL_HEARTBEAT && size == sizeof(struct ll_heartbeat)) {
        // CRC and packet is OK, reset watchdog
        struct ll_heartbeat *heartbeat = (struct ll_heartbeat *) buffer;
        last_heartbeat_ms = millis();
        power_request = heartbeat->power_request;
        emergency_high_level = heartbeat->emergency_high_level;
    } else if (buffer[0] == PACKET_ID_LL_HIGH_LEVEL_STATE && size == sizeof(struct ll_high_level_state)) {
        // copy the state
        last_high_level_state = *((struct ll_high_level_state *) buffer);
        last_high_level_ms = millis();
        #ifdef USB_DEBUG
            DEBUG_SERIAL.print("got high level status ");
            DEBUG_SERIAL.print(status_message.status_bitmask, BIN);
            DEBUG_SERIAL.println();
        #endif
    } else if (buffer[0] == PACKET_ID_LL_MOTOR_STATE && size == sizeof(struct ll_motor_state)) {
        // copy the state
        last_motor_state = *((struct ll_motor_state *) buffer);
        last_motor_ms = millis();
    } else if ((buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_SET || buffer[0] == PACKET_ID_LL_HIGH_LEVEL_CONFIG_GET) && size == sizeof(struct ll_high_level_config)) {
        //#ifdef USB_DEBUG
        //    DEBUG_SERIAL.println("got config packet");
        //#endif
        onConfigPacketReceived((struct ll_high_level_config *) buffer);
    } else {
        #ifdef USB_DEBUG
            DEBUG_SERIAL.printf("got valid unknown packet %d size %d\n",(int)buffer[0],(int)size);
        #endif       
    }
}

bool checkBatteryTemperatureOk() {
    return !ant_bms_parser.is_online() || 
        ((conf_charge_min_battery_temperature==0 || status_message.battery_temperature >= conf_charge_min_battery_temperature) &&
         (conf_charge_max_battery_temperature==0 || status_message.battery_temperature <= conf_charge_max_battery_temperature));
}

bool checkBalancerTemperatureOk() {
    return !ant_bms_parser.is_online() 
        || conf_charge_stop_balancer_temperature==0 || status_message.balancer_temperature <= conf_charge_stop_balancer_temperature;
}

bool checkChargerVoltageOk() {
    return (conf_charger_max_voltage==0 || status_message.v_charge <= conf_charger_max_voltage) 
        && (conf_charger_min_voltage==0 || status_message.v_charge >= conf_charger_min_voltage);
}

bool checkChargeCurrentOk() {
     return (conf_charge_max_current==0 || status_message.battery_current <= conf_charge_max_current)
        && (conf_charge_stop_current==0 || status_message.battery_current >= conf_charge_stop_current);
}

bool checkWantCharge() {
    return (ant_bms_parser.is_online() && conf_charge_start_soc!=0
                ? status_message.battery_soc < conf_charge_start_soc 
                : conf_charge_start_voltage!=0 ? status_message.v_battery < conf_charge_start_voltage : false);
}

bool checkShouldStartCharge() {
    return conf_valid 
        && status_message.v_battery < status_message.v_charge
        && checkChargerVoltageOk()
        && checkBatteryTemperatureOk()
        && checkBalancerTemperatureOk()
        && checkWantCharge();
}

bool checkShouldStopCharge() {
    return !conf_valid
        || status_message.v_battery > status_message.v_charge
        || !checkChargerVoltageOk()
        || !checkBalancerTemperatureOk()
        //|| status_message.charging_current < 0.5
        || (ant_bms_parser.is_online() && conf_charge_stop_soc!=0
                ? status_message.battery_soc > conf_charge_stop_soc 
                : conf_charge_stop_voltage!=0 ? status_message.v_battery > conf_charge_stop_voltage : true)
        || !checkChargeCurrentOk();
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

float readChargeCurrent() {
    if(status_message.v_charge > conf_charger_min_voltage) { //charger powers current sensor. no reason to read unpowered sensor
        uint16_t raw = analogRead(PIN_ANALOG_BATTERY_VOLTAGE);
        raw = analogRead(PIN_ANALOG_CHARGE_CURRENT);
        filtLowPass32(raw, CURRENT_FILT_COEF, &chargeCurrentFixdt);
        return (float)(chargeCurrentFixdt >> 16) * CURRENT_CALIB_REAL_CURRENT / (CURRENT_CALIB_ADC * 100.0);
    } else {
        //we don't know real battery discharge current
        return -0.2;
    }
}

float readBatteryVoltage() {
    uint16_t raw = analogRead(PIN_ANALOG_BATTERY_VOLTAGE);
    filtLowPass32(raw, BAT_FILT_COEF, &batVoltageFixdt);
    return (float)(batVoltageFixdt >> 16) * BAT_CALIB_REAL_VOLTAGE / (BAT_CALIB_ADC * 100.0);
}


float readChargerVoltage() {
    uint16_t raw = analogRead(PIN_ANALOG_CHARGE_VOLTAGE);
    filtLowPass32(raw, CHARGE_FILT_COEF, &chargeVoltageFixdt);
    return (float)(chargeVoltageFixdt >> 16) * CHARGE_CALIB_REAL_VOLTAGE / (CHARGE_CALIB_ADC * 100.0);
}


void loop() {
    uplinkPacketSerial.update();
#ifdef BMS_SERIAL
    ant_bms_parser.loop();
#endif
    imu_loop();
    updateChargingEnabled();
    updateEmergency();

    uint32_t now = millis();
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
            if( conf_uss_enabled[i] && (status_message.uss_age_ms[i] > USS_TIMEOUT_MS)) {
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
            status_message.v_charge = readChargerVoltage();
            status_message.v_battery = readBatteryVoltage();
            status_message.battery_current = readChargeCurrent();

            // calculate percent value accu filling
            float delta = conf_battery_full_voltage - conf_battery_empty_voltage;
            float vo = status_message.v_battery - conf_battery_empty_voltage;
            status_message.battery_soc = delta!=0 ? vo / delta * 100 : 0;
            status_message.battery_temperature = 20;//we don't know, assume it is ok
            status_message.balancer_temperature = 20;//we don't know, assume it is ok
#ifdef BMS_SERIAL        
        } else {
            status_message.v_charge = readChargerVoltage();
            status_message.v_battery = readBatteryVoltage();
            //status_message.v_battery = ant_bms_parser.total_voltage_sensor_;
            status_message.battery_current = -ant_bms_parser.current_sensor_;//reverse current sensor
            status_message.battery_soc = ant_bms_parser.soc_sensor_;
            //MOS 
            //status_message.battery_temperature = ant_bms_parser.temperatures_[0].temperature_sensor_;
            //Balance
            //status_message.balancer_temperature = ant_bms_parser.temperatures_[1].temperature_sensor_;
            //T1
            status_message.battery_temperature = ant_bms_parser.temperatures_[2].temperature_sensor_;
            //T2
            status_message.balancer_temperature = ant_bms_parser.temperatures_[3].temperature_sensor_;
        }
#endif //BMS_SERIAL
        if (status_message.battery_soc > 100) {
            status_message.battery_soc = 100;
        }

        // Check battery empty
        status_message.status_bitmask &= ~(1 << STATUS_BATTERY_EMPTY_BIT);
        if (conf_battery_shutdown_voltage!=0 && status_message.v_battery < conf_battery_shutdown_voltage) {
            status_message.status_bitmask |= 1<<STATUS_BATTERY_EMPTY_BIT;
        }
        if (ant_bms_parser.is_online() && conf_battery_shutdown_soc!=0 && status_message.battery_soc < conf_battery_shutdown_soc) {
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

#ifdef BMS_SERIAL
    if (now - last_bms_update_ms > BMS_UPDATE_PERIOD_MS) {
        ant_bms_parser.update();
        last_bms_update_ms = now;
    }
#endif //BMS_SERIAL
    
    updateDisplay(now);

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

void displayConfiguration() {
    snprintf(display_line,17, "%c bV %4.1f %4.1f",conf_valid?'+':'-',conf_battery_empty_voltage,conf_battery_full_voltage);
    display.drawString(0, 0, display_line);

    snprintf(display_line,17, "bT %3.1f %3.1f %3.1f",conf_charge_min_battery_temperature,conf_charge_max_battery_temperature,conf_charge_stop_balancer_temperature);
    display.drawString(0, 1, display_line);

    snprintf(display_line,17, "csSOC %3d %3d",(int)conf_charge_start_soc,(int)conf_charge_stop_soc);
    display.drawString(0, 2, display_line);

    snprintf(display_line,17, "csV %4.1f %4.1f",conf_charge_start_voltage,conf_charge_stop_voltage);
    display.drawString(0, 3, display_line);

    snprintf(display_line,17, "csC %4.1f %4.1f",conf_charge_stop_current,conf_charge_max_current);
    display.drawString(0, 4, display_line);

    snprintf(display_line,17, "cV %4.1f %4.1f",conf_charger_min_voltage,conf_charger_max_voltage);
    display.drawString(0, 5, display_line);

    snprintf(display_line,17, "bsSOCV %3d %4.1f",(int)conf_battery_shutdown_soc,conf_battery_shutdown_voltage);
    display.drawString(0, 6, display_line);
    int uss=0;
    for(int i=0;i<USS_COUNT;i++) {
        uss |= conf_uss_enabled[i]<<i;
    }
    int contact=0;
    for(int i=0;i<CONTACT_COUNT;i++) {
        contact |= conf_contact_mode[i]<<i*2;
    }
    int active_low=0;
    for(int i=0;i<CONTACT_COUNT;i++) {
        active_low |= conf_contact_active_low[i]<<i;
    }

    snprintf(display_line,17, "uss%2d ca%3d cl%2d",uss,contact,active_low);
    display.drawString(0, 7, display_line);
}

void updateDisplayLine1(uint32_t now_ms) {
    uint32_t high_level_age_ms = now_ms - last_high_level_ms;
    
    //ROS connection status
    status_line[0]         = emergency_high_level!=0 ? 'E' : 'T';
    status_line_blink[0]   = status_message.emergency_bitmask & (1<<EMERGENCY_ROS_TIMEOUT) ? DISPLAY_SLOW_BLINK : emergency_high_level!=0 ? DISPLAY_FAST_BLINK : DISPLAY_NO_BLINK;

    //GPS status
    status_line[1]         = high_level_age_ms < HIGH_LEVEL_TIMEOUT_MS ? 'G' : 'g';
    if ( high_level_age_ms >= HIGH_LEVEL_TIMEOUT_MS) { status_line_blink[1] = DISPLAY_SLOW_BLINK; }
    else if (last_high_level_state.gps_quality < 25) { status_line_blink[1] = DISPLAY_FAST_BLINK; }
    else if (last_high_level_state.gps_quality < 50) { status_line_blink[1] = DISPLAY_NORM_BLINK; }
    else if (last_high_level_state.gps_quality < 75) { status_line_blink[1] = DISPLAY_SLOW_BLINK; }
    else { status_line_blink[1] = DISPLAY_NO_BLINK; }

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
                status_line[2]         = 'M';
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
    uint8_t anyEmergencyContact = 0;
    uint8_t nonEmergenyContacts = 0;
    for(int i=0;i<CONTACT_COUNT;i++) {
        int emergenyContact = status_message.emergency_bitmask & (1<<contact_emergency_bits[i]);
        int justContact = status_message.contacts & (1<<i);
        anyEmergencyContact |= emergenyContact;
        if(justContact && !emergenyContact) {
            nonEmergenyContacts |= justContact;
        }
    }
    //Lift/Tilt occurs after timeout (PIN_EMERGENCY_3 PIN_EMERGENCY_4)
    status_line[5]         = nonEmergenyContacts ? 'L' : ' ';
    //Unused at the moment
    status_line_blink[5]   = DISPLAY_FAST_BLINK;
    //Stop button(s) pressed after timeout (PIN_EMERGENCY_1 PIN_EMERGENCY_2)
    status_line[6]         = anyEmergencyContact ? 'S' : ' ';
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
    status_line[13]        = (status_message.status_bitmask & (1<<STATUS_CHARGING_BIT)) || checkWantCharge() ? 'C' : ' ';
    //blink C if want charge but no charging
    status_line_blink[13]  = (status_message.status_bitmask & (1<<STATUS_CHARGING_BIT)) == 0 ? DISPLAY_FAST_BLINK : DISPLAY_NO_BLINK;
    uint8_t batt_percentage = status_message.battery_soc >= 100 ? 99 : status_message.battery_soc < 0 ? 0 : status_message.battery_soc;
    status_line[14]        = '0'+batt_percentage/10;
    status_line[15]        = '0'+batt_percentage%10;
    status_line_blink[14]  = status_message.status_bitmask & (1<<STATUS_BMS_TIMEOUT_BIT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    status_line_blink[15]  = status_message.status_bitmask & (1<<STATUS_BMS_TIMEOUT_BIT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    updateBlink(status_line,status_line_blink,16,displayUpdateCounter);

    snprintf(display_line,17, "%.16s",status_line);
    display.drawString(0, 0, display_line);
}

void updateDisplayLine2(uint32_t now_ms) {
    uint16_t s = last_motor_state.status[1];
    display_motor_status[0]       = s & (1<<MOTOR_STATUS_BATTERY_DEAD | 1<<MOTOR_STATUS_BATTERY_L1  | 1<<MOTOR_STATUS_BATTERY_L2) ? 'V' : ' ';
    display_motor_status_blink[0] = s & (1<<MOTOR_STATUS_BATTERY_DEAD) ? DISPLAY_FAST_BLINK : s & (1<<MOTOR_STATUS_BATTERY_L1) ? DISPLAY_NORM_BLINK : DISPLAY_SLOW_BLINK;
    display_motor_status[1]       = s & (1<<MOTOR_STATUS_GEN_TIMEOUT) ? 'G' : s & (1<<MOTOR_STATUS_ADC_TIMEOUT) ? 'A' : ' ';
    display_motor_status_blink[1] = s & (1<<MOTOR_STATUS_GEN_TIMEOUT | 1<<MOTOR_STATUS_ADC_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[2]       = 'C';//connected
    display_motor_status_blink[2] = s & (1<<MOTOR_STATUS_CONN_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[3]       = s & (1<<MOTOR_STATUS_PCB_TEMP_ERR | 1<<MOTOR_STATUS_PCB_TEMP_WARN) ? 'T' : ' ';
    display_motor_status_blink[3] = s & (1<<MOTOR_STATUS_PCB_TEMP_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[4]       = s & (1<<MOTOR_STATUS_RIGHT_MOTOR_ERR) ? 'R' : s & (1<<MOTOR_STATUS_RIGHT_MOTOR_TEMP_ERR) ? 'r' : ' ';
    display_motor_status_blink[4] = s & (1<<MOTOR_STATUS_RIGHT_MOTOR_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[5]       = s & (1<<MOTOR_STATUS_LEFT_MOTOR_ERR) ? 'L' : s & (1<<MOTOR_STATUS_LEFT_MOTOR_TEMP_ERR) ? 'l' : ' ';
    display_motor_status_blink[5] = s & (1<<MOTOR_STATUS_LEFT_MOTOR_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[6]       = s & (1<<MOTOR_STATUS_BAD_CTRL_MODE) ? 'C' : ' ';
    display_motor_status_blink[6] = s & (1<<MOTOR_STATUS_BAD_CTRL_MODE) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[7]       = 'D';//drive on/disabled blink
    display_motor_status_blink[7] = s & (1<<MOTOR_STATUS_DISABLED) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    if( now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS || last_motor_state.status_age_s[1]*1000 > MOTOR_STATUS_TIMEOUT_MS) {
        if(now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS) {
            memcpy(display_motor_status," NO DATA",8);
        }
        for(int i=0;i<8;i++) display_motor_status_blink[i]=DISPLAY_SLOW_BLINK;
    }
    updateBlink(display_motor_status,display_motor_status_blink,8,displayUpdateCounter);

    snprintf(display_line,17, "Front   %.8s",display_motor_status);
    display.drawString(0, 1, display_line);
}

void updateDisplayLine3(uint32_t now_ms){
    uint16_t s = last_motor_state.status[0];
    display_motor_status[0]       = s & (1<<MOTOR_STATUS_BATTERY_DEAD | 1<<MOTOR_STATUS_BATTERY_L1  | 1<<MOTOR_STATUS_BATTERY_L2) ? 'V' : ' ';
    display_motor_status_blink[0] = s & (1<<MOTOR_STATUS_BATTERY_DEAD) ? DISPLAY_FAST_BLINK : s & (1<<MOTOR_STATUS_BATTERY_L1) ? DISPLAY_NORM_BLINK : DISPLAY_SLOW_BLINK;
    display_motor_status[1]       = s & (1<<MOTOR_STATUS_GEN_TIMEOUT) ? 'G' : s & (1<<MOTOR_STATUS_ADC_TIMEOUT) ? 'A' : ' ';
    display_motor_status_blink[1] = s & (1<<MOTOR_STATUS_GEN_TIMEOUT | 1<<MOTOR_STATUS_ADC_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[2]       = 'C';//connected
    display_motor_status_blink[2] = s & (1<<MOTOR_STATUS_CONN_TIMEOUT) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[3]       = s & (1<<MOTOR_STATUS_PCB_TEMP_ERR | 1<<MOTOR_STATUS_PCB_TEMP_WARN) ? 'T' : ' ';
    display_motor_status_blink[3] = s & (1<<MOTOR_STATUS_PCB_TEMP_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[4]       = s & (1<<MOTOR_STATUS_RIGHT_MOTOR_ERR) ? 'R' :  s & (1<<MOTOR_STATUS_RIGHT_MOTOR_TEMP_ERR) ? 'r' : ' ';
    display_motor_status_blink[4] = s & (1<<MOTOR_STATUS_RIGHT_MOTOR_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[5]       = s & (1<<MOTOR_STATUS_LEFT_MOTOR_ERR) ? 'L' : s & (1<<MOTOR_STATUS_LEFT_MOTOR_TEMP_ERR) ? 'l' : ' ';
    display_motor_status_blink[5] = s & (1<<MOTOR_STATUS_LEFT_MOTOR_ERR) ? DISPLAY_FAST_BLINK : DISPLAY_NORM_BLINK;
    display_motor_status[6]       = s & (1<<MOTOR_STATUS_BAD_CTRL_MODE) ? 'C' : ' ';
    display_motor_status_blink[6] = s & (1<<MOTOR_STATUS_BAD_CTRL_MODE) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    display_motor_status[7]       = 'D';//drive on/disabled blink
    display_motor_status_blink[7] = s & (1<<MOTOR_STATUS_DISABLED) ? DISPLAY_SLOW_BLINK : DISPLAY_NO_BLINK;
    if(now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS || last_motor_state.status_age_s[0]*1000 > MOTOR_STATUS_TIMEOUT_MS) {
        if(now_ms - last_motor_ms > MOTOR_STATUS_TIMEOUT_MS) {
            memcpy(display_motor_status," NO DATA",8);
        }
        for(int i=0;i<8;i++) display_motor_status_blink[i]=DISPLAY_SLOW_BLINK;
    }
    updateBlink(display_motor_status,display_motor_status_blink,8,displayUpdateCounter);

    snprintf(display_line,17, "Rear    %.8s",display_motor_status);
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

void updateDisplayLine7(uint32_t now_ms) {
    snprintf(display_line,17, "USS %3d %3d %3d",
    status_message.uss_age_ms[0]<USS_TIMEOUT_MS ? (int)(status_message.uss_ranges_cm[0]) : -1,
    status_message.uss_age_ms[1]<USS_TIMEOUT_MS ? (int)(status_message.uss_ranges_cm[1]) : -1,
    status_message.uss_age_ms[2]<USS_TIMEOUT_MS ? (int)(status_message.uss_ranges_cm[2]) : -1);
    display.drawString(0, 6, display_line);
}

void updateDisplayLine8(uint32_t now_ms) {
    snprintf(display_line,17, "USS %3d %3d",
    status_message.uss_age_ms[3]<USS_TIMEOUT_MS ? (int)(status_message.uss_ranges_cm[3]) : -1,
    status_message.uss_age_ms[4]<USS_TIMEOUT_MS ? (int)(status_message.uss_ranges_cm[4]) : -1);
    display.drawString(0, 7, display_line);
}

void updateDisplay(uint32_t now_ms) {
    //#ifdef USB_DEBUG
    //    DEBUG_SERIAL.print(".");
    //#endif
    last_display_line++;
    if(last_display_line>7) {
        if (now_ms - last_display_ms < DISPLAY_CYCLETIME_MS) {
            last_display_line=7;
            return;
        }
        last_display_ms = now_ms;
        last_display_line=0;
        displayUpdateCounter++;
        if(displayUpdateCounter & DISPLAY_BLINK_CYCLE) {
            displayUpdateCounter = 0;
        }                                         
    }
    if(!conf_valid) {
        displayConfiguration();
        return;
    }

    switch (last_display_line) {
        case 0:
            //High level group 0-4 (5 symbols)
            //Communication heartbit status
            updateDisplayLine1(now_ms);
            break;
        case 1:
            // Rear motor
            updateDisplayLine2(now_ms);
            break;
        case 2:
            // Front motor
            updateDisplayLine3(now_ms);
            break;
        case 3:
            // Mower motor
            updateDisplayLine4(now_ms);
            break;
        case 4:
            updateDisplayLine5(now_ms);
            break;
        case 5:
            updateDisplayLine6(now_ms);
            break;
        case 6:
            updateDisplayLine7(now_ms);
            break;
        case 7:
            updateDisplayLine8(now_ms);
            break;
    }

    //snprintf(display_line,17, "Main    %.8s",ll_display_emerg);
    //display.drawString(0, 2, display_line);

    //snprintf(display_line,17, "ADC %d %d ",batteryADCRaw,chargerADCRaw);
    //display.drawString(0, 6, display_line);

    //snprintf(display_line,17, "ADC %d       ",currentADCRaw);
    //display.drawString(0, 7, display_line);


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
    //#ifdef USB_DEBUG
    //    DEBUG_SERIAL.printf("Display line %d updated in %dms\n",last_display_line,millis()-now_ms);
    //#endif
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

ConfigValue &eepromGet(const uint8_t address,const uint8_t address2,ConfigValue &val){
    int valueSize = sizeof(union ConfigValue);
    int realAddress = ((int)address + (int)address2*4)*valueSize;
    return EEPROMFixed.get<ConfigValue>(realAddress,val);
}

ConfigValue &eepromGet(const uint8_t address,ConfigValue &val){
    return eepromGet(address,0,val);
}

const ConfigValue &eepromPut(const uint8_t address,const uint8_t address2,const ConfigValue &val){
    int valueSize = sizeof(union ConfigValue);
    int realAddress = ((int)address + (int)address2*4)*valueSize;
    return EEPROMFixed.put<ConfigValue>(realAddress,val);
}

const ConfigValue &eepromPut(const uint8_t address,const ConfigValue &val){
    return eepromPut(address,0,val);
}

void resetConfiguration() {
    ConfigValue val;
    
    val.int32Value=0; val.int8Value=85;
    eepromPut(ConfigAddress::CHARGE_START_SOC,val);
    val.int32Value=0; val.floatValue=3.35*12;
    eepromPut(ConfigAddress::CHARGE_START_VOLTAGE,val);

    val.int32Value=0; val.int8Value=95;
    eepromPut(ConfigAddress::CHARGE_STOP_SOC,val);
    val.int32Value=0; val.floatValue=3.45*12;
    eepromPut(ConfigAddress::CHARGE_STOP_VOLTAGE,val);
    
    val.int32Value=0; val.floatValue=10.0;
    eepromPut(ConfigAddress::CHARGE_MAX_CURRENT,val);
    val.int32Value=0; val.floatValue=0.1;
    eepromPut(ConfigAddress::CHARGE_STOP_CURRENT,val);
    
    val.int32Value=0; val.floatValue=5.0;
    eepromPut(ConfigAddress::CHARGER_MIN_VOLTAGE,val);
    val.int32Value=0; val.floatValue=3.8*12;
    eepromPut(ConfigAddress::CHARGER_MAX_VOLTAGE,val);

    val.int32Value=0; val.floatValue=1.0;
    eepromPut(ConfigAddress::CHARGE_MIN_BATTERY_TEMPERATURE,val);
    
    val.int32Value=0; val.floatValue=35.0;
    eepromPut(ConfigAddress::CHARGE_MAX_BATTERY_TEMPERATURE,val);
    
    val.int32Value=0; val.floatValue=80.0;
    eepromPut(ConfigAddress::CHARGE_STOP_BALANCER_TEMPERATURE,val);

    val.int32Value=0; val.floatValue=2.5*12;
    eepromPut(ConfigAddress::BATTERY_EMPTY_VOLTAGE,val);
    val.int32Value=0; val.floatValue=3.65*12;
    eepromPut(ConfigAddress::BATTERY_FULL_VOLTAGE,val);

    val.int32Value=0; val.int8Value=5;
    eepromPut(ConfigAddress::BATTERY_SHUTDOWN_SOC,val);
    val.int32Value=0; val.floatValue=2.7*12;
    eepromPut(ConfigAddress::BATTERY_SHUTDOWN_VOLTAGE,val);

    for (int uss_num = 0; uss_num < USS_COUNT; uss_num++) {
        val.int32Value=0; val.boolValue=true;
        eepromPut(ConfigAddress::USS_ACTIVE,uss_num,val);
    }

    for (int contact_num = 0; contact_num < CONTACT_COUNT; contact_num++) {
        val.int32Value=0; val.int8Value = contact_num==0 ? ContactMode::EMERGENCY_STOP : ContactMode::MONITOR;
        eepromPut(ConfigAddress::CONTACT_MODE,contact_num,val);
        
        val.int32Value=0; val.boolValue=true;
        eepromPut(ConfigAddress::CONTACT_ACTIVE_LOW,contact_num,val);
        
        val.int32Value=0; val.int32Value=20;
        eepromPut(ConfigAddress::CONTACT_TIMEOUT,contact_num,val);
    }

}

bool loadConfiguration(char *conf_message) {
    ConfigValue val;
    
    conf_valid=true;

    conf_charge_start_soc = eepromGet(ConfigAddress::CHARGE_START_SOC,val).int8Value;
    conf_charge_start_voltage = eepromGet(ConfigAddress::CHARGE_START_VOLTAGE,val).floatValue;

    conf_charge_stop_soc = eepromGet(ConfigAddress::CHARGE_STOP_SOC,val).int8Value;
    conf_charge_stop_voltage = eepromGet(ConfigAddress::CHARGE_STOP_VOLTAGE,val).floatValue;

    conf_charge_max_current = eepromGet(ConfigAddress::CHARGE_MAX_CURRENT,val).floatValue;
    conf_charge_stop_current = eepromGet(ConfigAddress::CHARGE_STOP_CURRENT,val).floatValue;

    conf_charger_min_voltage = eepromGet(ConfigAddress::CHARGER_MIN_VOLTAGE,val).floatValue;
    conf_charger_max_voltage = eepromGet(ConfigAddress::CHARGER_MAX_VOLTAGE,val).floatValue;

    conf_charge_min_battery_temperature = eepromGet(ConfigAddress::CHARGE_MIN_BATTERY_TEMPERATURE,val).floatValue;
    conf_charge_max_battery_temperature = eepromGet(ConfigAddress::CHARGE_MAX_BATTERY_TEMPERATURE,val).floatValue;
    conf_charge_stop_balancer_temperature = eepromGet(ConfigAddress::CHARGE_STOP_BALANCER_TEMPERATURE,val).floatValue;

    conf_battery_empty_voltage = eepromGet(ConfigAddress::BATTERY_EMPTY_VOLTAGE,val).floatValue;
    conf_battery_full_voltage = eepromGet(ConfigAddress::BATTERY_FULL_VOLTAGE,val).floatValue;

    conf_battery_shutdown_soc = eepromGet(ConfigAddress::BATTERY_SHUTDOWN_SOC,val).int8Value;
    conf_battery_shutdown_voltage = eepromGet(ConfigAddress::BATTERY_SHUTDOWN_VOLTAGE,val).floatValue;

    for (int uss_num = 0; uss_num < USS_COUNT; uss_num++) {
        conf_uss_enabled[uss_num] = eepromGet(ConfigAddress::USS_ACTIVE,uss_num,val).boolValue;
    }

    for (int contact_num = 0; contact_num < CONTACT_COUNT; contact_num++) {
        conf_contact_mode[contact_num] = eepromGet(ConfigAddress::CONTACT_MODE,contact_num,val).int8Value;
        testIntInRange(conf_contact_mode[contact_num],ContactMode::OFF,ContactMode::EMERGENCY_STOP,"CM",conf_message);
        conf_contact_active_low[contact_num] = eepromGet(ConfigAddress::CONTACT_ACTIVE_LOW,contact_num,val).boolValue;
        conf_contact_timeout_ms[contact_num] = eepromGet(ConfigAddress::CONTACT_TIMEOUT,contact_num,val).int32Value;
        testIntInRange(conf_contact_timeout_ms[contact_num],1,10000,"CTms",conf_message);
    }

    //basic non zero test
    testFloatInRange(conf_battery_empty_voltage,1,conf_battery_full_voltage,"BEV",conf_message);
    testFloatInRange(conf_battery_full_voltage,conf_battery_empty_voltage,100,"BFV",conf_message);

    testFloatOrZero(conf_charger_min_voltage,1,max(conf_charger_max_voltage,conf_battery_full_voltage),"MinCV",conf_message);
    testFloatOrZero(conf_charger_max_voltage,max(conf_charger_min_voltage,conf_battery_empty_voltage),100,"MaxCV",conf_message);

    testFloatOrZero(conf_charge_min_battery_temperature,1,min(30,conf_charge_max_battery_temperature),"MinBT",conf_message);
    testFloatOrZero(conf_charge_max_battery_temperature,max(1,conf_charge_min_battery_temperature),60,"MaxBT",conf_message);
    testFloatOrZero(conf_charge_stop_balancer_temperature,30,80,"StopBT",conf_message);

    testIntOrZero(conf_charge_start_soc,0,conf_charge_stop_soc,"StaCS",conf_message);
    testIntOrZero(conf_charge_stop_soc,conf_charge_start_soc,100,"StoCS",conf_message);

    testIntOrZero(conf_charge_start_voltage,0,conf_charge_stop_voltage,"StaCV",conf_message);
    testIntOrZero(conf_charge_stop_voltage,conf_charge_start_voltage,100,"StoCV",conf_message);

    testIntOrZero(conf_battery_shutdown_soc,0,100,"BSS",conf_message);
    testFloatOrZero(conf_battery_shutdown_voltage,conf_battery_empty_voltage,conf_battery_full_voltage,"BSV",conf_message);

    testFloatOrZero(conf_charge_stop_current,0,conf_charge_max_current,"CSC",conf_message);
    testFloatOrZero(conf_charge_max_current,conf_charge_stop_current,20,"CMC",conf_message);

    #ifdef USB_DEBUG
        if(conf_valid) {
            DEBUG_SERIAL.println("Valid conf");
        } else {
            DEBUG_SERIAL.printf("Invalid conf: %s\n",conf_message);
        }
    #endif

    return conf_valid;
}
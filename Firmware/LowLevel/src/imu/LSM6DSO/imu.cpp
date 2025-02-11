#include "imu.h"
#include "pins.h"
#include <LSM6DSOSensor.h>

#include <SPI.h>
#define spiBus SPI
// Needs software UART because pins were messed up in 0.12
//#include <PioSPI.h>
//PioSPI spiBus(PIN_IMU_MOSI, PIN_IMU_MISO, PIN_IMU_SCK, PIN_IMU_CS, SPI_MODE3, 1000000);

LSM6DSOSensor IMU(&spiBus, PIN_IMU_CS, 1000000);
int32_t accelerometer[3];
int32_t gyroscope[3];

bool init_imu(Stream* serial) {
  spiBus.setCS(PIN_IMU_CS);
  spiBus.setTX(PIN_IMU_TX);
  spiBus.setRX(PIN_IMU_RX);
  spiBus.setSCK(PIN_IMU_SCK);
  spiBus.begin();
  //software variant
  //spiBus.begin();

    int status = IMU.begin();
    if(status != 0) {
        if(serial) {
            serial->print("IMU begin failed ");
            serial->print(status, BIN);
            serial->print("\n");
        }
        return false;
    } else {
        if(serial) {
            serial->println("IMU begin succeded");
        }
    }

    uint8_t WHOAMI = 0;
    IMU.ReadID(&WHOAMI);
    if(WHOAMI != 0b01101010 && WHOAMI != 0b01101100) {
        if(serial) {
            serial->print("IMU initialization WHOAMI ");
            serial->print(WHOAMI, BIN);
            serial->print("\n");
        }
        return false;
    }

    if (IMU.Enable_G() != 0) {
        if(serial) {
            serial->println("IMU enable G failed");
        }
        return false;
    }

    if (IMU.Enable_X() != 0) {
        if(serial) {
            serial->println("IMU enable X failed");
        }
        return false;
    }
    return true;
}

bool imu_read(float *acceleration_mss, float *gyro_rads, float *mag_uT) {
    bool success = true;
    success &= IMU.Get_X_Axes(accelerometer) == 0;
    success &= IMU.Get_G_Axes(gyroscope) == 0;

    acceleration_mss[0] = accelerometer[0] * 9.81 / 1000.0;
    acceleration_mss[1] = accelerometer[1] * 9.81 / 1000.0;
    acceleration_mss[2] = accelerometer[2] * 9.81 / 1000.0;

    gyro_rads[0] = gyroscope[0] * (PI / 180.0) / 1000.0;
    gyro_rads[1] = gyroscope[1] * (PI / 180.0) / 1000.0;
    gyro_rads[2] = gyroscope[2] * (PI / 180.0) / 1000.0;

    mag_uT[0] = 0;
    mag_uT[1] = 0;
    mag_uT[2] = 0;

    return success;
}

void imu_loop() {}

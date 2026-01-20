#include "EEPROMFixed.h" 
#include <Arduino.h>
#include <hardware/flash.h>
#include <hardware/sync.h>

bool EEPROMClassFixed::commitFixed() {
    if (!_size) {
        return false;
    }
    if (!_dirty) {
        return true;
    }
    if (!_data) {
        return false;
    }

    //if (!__isFreeRTOS) {
        noInterrupts();
    //}
    flash_range_erase((intptr_t)_sector - (intptr_t)XIP_BASE, 4096);
    flash_range_program((intptr_t)_sector - (intptr_t)XIP_BASE, _data, _size);
    //if (!__isFreeRTOS) {
        interrupts();
    //}
    _dirty = false;

    return true;
}

EEPROMClassFixed EEPROMFixed;
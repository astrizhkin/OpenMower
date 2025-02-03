#include "conf.h"
#include <Arduino.h>

extern bool conf_valid;

void testIntOrZero(int val, int minVal, int maxVal,const char *name,char *message) {
    if(val==0){
        return;
    }
    testIntInRange(val, minVal, maxVal, name, message);
}

void testIntInRange(int val, int minVal, int maxVal,const char *name,char *message) {
    if(!conf_valid){
        return;
    }
    if(minVal!=0 && val < minVal) {
        snprintf(message, sizeof(message), "%s<%3d", name,minVal);
        conf_valid=false;
    }
    if(maxVal!=0 && val > maxVal) {
        snprintf(message, sizeof(message), "%s>%3d", name,maxVal);
        conf_valid=false;
    }
}

void testFloatInRange(float val, float minVal, float maxVal,const char *name,char *message) {
    if(!conf_valid){
        return;
    }
    if(minVal!=0 && val < minVal) {
        snprintf(message, sizeof(message), "%s<%4.1f", name,minVal);
        conf_valid=false;
    }
    if(maxVal!=0 && val > maxVal) {
        snprintf(message, sizeof(message), "%s>%4.1f", name,maxVal);
        conf_valid=false;
    }
}

void testFloatOrZero(float val, float minVal, float maxVal,const char *name,char *message) {
    if(val==0){
        return;
    }
    testFloatInRange(val, minVal, maxVal, name, message);
}


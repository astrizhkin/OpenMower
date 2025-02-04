#include <EEPROM.h>

class EEPROMClassFixed : public EEPROMClass {
    public:
        bool commitFixed();
};

extern EEPROMClassFixed EEPROMFixed;
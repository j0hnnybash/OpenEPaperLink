#define __packed
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "asmUtil.h"
#include "comms.h"  // for mLastLqi and mLastRSSI
#include "eeprom.h"
#include "i2c.h"
#include "i2cdevices.h"
#include "powermgt.h"
#include "printf.h"
#include "proto.h"
#include "radio.h"
#include "screen.h"
#include "settings.h"
#include "syncedproto.h"
#include "timer.h"
#include "userinterface.h"
#include "wdt.h"
#include "sleep.h"


// #define DEBUG_MODE

//source https://dmitry.gr/?r=05.Projects&proj=30.%20Reverse%20Engineering%20an%20Unknown%20Microcontroller#_TOC_c97c7180a59c5801fd75894c0ce992ef
// Timing Notes:
// 8051 timers tick with 1/12th of 16MHz XTAL (~1.3MHz)
// 8052 Timer2 apparently is not implemented by the core :/
// Note: timer 1 is always available it is not used as the UART clk (as it is in OG 8051)
// WDT counts up at ~62kHz
// SPI CLK rates: 500KHz, 1MHz, 2MHz, 4MHz controlled by "SPICFG"
// (low power) Sleep timer: 24bit, can be set to tick at 1Hz or 32kHz (!?) -> 30ms to 194days
// Q: Why drawWithSleep?
// A: Eink drawing takes quite long, the CPU can spend this time sleeping!

// note: reboot issues -> short battery terminals before battery insertion to discharge input caps.

// Radio commands are completely undocumented?!

void test_sleep_power_consumption() {

    wdt30s();
    doSleep(15000UL); // with CFGPG save/restore 0.00 mA sleep; w/o save/restore 0.00mA
    powerUp(INIT_EPD); showClockDigital(42, 1, 0); powerDown(INIT_EPD);

    wdt30s();
    sleepForMsec(15000UL); // with CFGPG save/restore 0.00 mA sleep; w/o save/restore 0.00mA
    powerUp(INIT_EPD); showClockDigital(42, 2, 0); powerDown(INIT_EPD);

    wdt60s();
    doSleep(30000UL); // with CFGPG save/restore 0.00 mA; w/o save/restore 0.00mA
    powerUp(INIT_EPD); showClockDigital(42, 3, 0); powerDown(INIT_EPD);

    wdt60s();
    sleepForMsec(30000UL); // with CFGPG save/restore 0.00 mA; w/o save/restore 0.00mA
    powerUp(INIT_EPD); showClockDigital(42, 4, 0); powerDown(INIT_EPD);

    wdt120s();
    doSleep(60e3); // with CFGPG save/restore 0.00 mA
    powerUp(INIT_EPD); showClockDigital(42, 5, 0); powerDown(INIT_EPD);

    wdt120s();
    sleepForMsec(60e3); // with CFGPG save/restore 0.00 mA
    powerUp(INIT_EPD); showClockDigital(42, 6, 0); powerDown(INIT_EPD);

    // 0.00 mA in other sleep

}

void main() {
    // displayLoop();  // remove me
    setupPortsInitial();
    powerUp(INIT_BASE | INIT_UART);

    if (RESET & 0x01) {
        wakeUpReason = WAKEUP_REASON_WDT_RESET;
        pr("WDT reset!\n");
    } else {
        wakeUpReason = WAKEUP_REASON_FIRSTBOOT;
    }

    wdt10s();

    boardGetOwnMac(mSelfMac);

    {
        bool __xdata macSet = false;
        for (uint8_t __xdata c = 0; c < 8; c++) {
            if (mSelfMac[c] != 0xFF) {
                macSet = true;
                break;
            }
        }

        if (!macSet) {
            pr("Mac can't be all FF's.\n");
            powerUp(INIT_EPD);
            showNoMAC();
            powerDown(INIT_EPD | INIT_UART | INIT_EEPROM);
            doSleep(-1);
            wdtDeviceReset();
        }
    }

    pr("BOOTED>  %d.%d.%d%s\n", fwVersion / 100, (fwVersion % 100) / 10, (fwVersion % 10), fwVersionSuffix);

    powerUp(INIT_I2C);

    //i2cBusScan();

    if (i2cCheckDevice(0x55)) {
        powerDown(INIT_I2C);
        capabilities |= CAPABILITY_HAS_NFC;
        if (supportsNFCWake()) {
            pr("This board supports NFC wake!\n");
            capabilities |= CAPABILITY_NFC_WAKE;
        }
    } else {
        powerDown(INIT_I2C);
    }

    pr("MAC>%02X%02X", mSelfMac[0], mSelfMac[1]);
    pr("%02X%02X", mSelfMac[2], mSelfMac[3]);
    pr("%02X%02X", mSelfMac[4], mSelfMac[5]);
    pr("%02X%02X\n", mSelfMac[6], mSelfMac[7]);

    powerUp(INIT_RADIO);  // load down the battery using the radio to get a good voltage reading
    powerUp(INIT_EPD_VOLTREADING | INIT_TEMPREADING);
    powerDown(INIT_RADIO);

    /* powerUp(INIT_EEPROM); */
    /* // get the highest slot number, number of slots */
    /* initializeProto(); */
    /* powerDown(INIT_EEPROM); */

    // show the splashscreen
    powerUp(INIT_EPD);
    showSplashScreen();

// we've now displayed something on the screen; for the SSD1619, we are now aware of the lut-size
#ifdef EPD_SSD1619
    capabilities |= CAPABILITY_SUPPORTS_CUSTOM_LUTS;
    if (dispLutSize != 7) {
        capabilities |= CAPABILITY_ALT_LUT_SIZE;
    }
#endif

    powerUp(INIT_EPD);
    wdt60s();
    showClockDigital(42, 42, 0);
    //powerDown(INIT_EPD); // if we don't power down the EPD it will not display anything next time around.
    //doSleep(5000UL);
    wdt10s();
    //powerUp(INIT_EPD); // doesn't seem to be idempotent, requires preceeding powerDown otherwise nothing will be drawn.
    showClockDigital(41, 41, 0);
    powerDown(INIT_EPD | INIT_UART);
    int8_t hours = 0;
    int8_t minutes = 0;

    test_sleep_power_consumption();

    // sleepFotMsec()?
    //doSleep(5000UL); ?
    // Stopwatch
    while (1) {
        powerUp(INIT_EPD);
        showClockDigital(hours, minutes, 0);
        powerDown(INIT_EPD);
        wdt120s();
        sleepForMsec(60e3);
        minutes += 1;
        if (minutes > 59) {
            minutes -= 60;
            hours += 1;
        }
    }
}

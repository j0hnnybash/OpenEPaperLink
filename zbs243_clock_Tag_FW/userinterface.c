

#include "userinterface.h"

#include <stdbool.h>
#include <string.h>

#include "asmUtil.h"
#include "bitmaps.h"
#include "board.h"
#include "comms.h"
#include "cpu.h"
#include "font.h"
#include "lut.h"
#include "powermgt.h"
#include "printf.h"
#include "proto.h"
#include "screen.h"
#include "settings.h"
#include "sleep.h"
#include "spi.h"
#include "syncedproto.h"  // for APmac / Channel
#include "timer.h"

// extern uint8_t __xdata mSelfMac[8];
// extern uint8_t __xdata currentChannel;
// extern uint8_t __xdata APmac[];
// extern uint16_t __xdata batteryVoltage;

const uint8_t __code fwVersion = FW_VERSION;
const char __code fwVersionSuffix[] = FW_VERSION_SUFFIX;

extern uint8_t __xdata capabilities;

bool __xdata lowBatteryShown = false;
bool __xdata noAPShown = false;

void addCapabilities() {
    if (capabilities) epdpr("Options: ");
    if (capabilities & CAPABILITY_HAS_NFC) {
        epdpr("-NFC");
        if (capabilities & CAPABILITY_NFC_WAKE) {
            epdpr("+WAKE");
        } else {
            epdpr(" ");
        }
    }
    if (capabilities & CAPABILITY_HAS_WAKE_BUTTON) {
        epdpr("-WAKE BUTTON");
    }
}

void addOverlay() {
    if (currentChannel == 0) {
#if (SCREEN_WIDTH == 152)
        loadRawBitmap(ant, SCREEN_WIDTH - 16, 0, EPD_COLOR_BLACK);
        loadRawBitmap(cross, SCREEN_WIDTH - 8, 7, EPD_COLOR_RED);
#elif (SCREEN_WIDTH == 128)
        loadRawBitmap(ant, 0, 0, EPD_COLOR_BLACK);
        loadRawBitmap(cross, 8, 0, EPD_COLOR_RED);
#elif (SCREEN_WIDTH == 400)
        loadRawBitmap(ant, SCREEN_WIDTH - 24, 6, EPD_COLOR_BLACK);
        loadRawBitmap(cross, SCREEN_WIDTH - 16, 13, EPD_COLOR_RED);
#endif
        noAPShown = true;
    } else {
        noAPShown = false;
    }

    if (batteryVoltage != 2600) {
#if (SCREEN_WIDTH == 152)
        loadRawBitmap(battery, SCREEN_WIDTH - 16, SCREEN_HEIGHT - 10, EPD_COLOR_BLACK);
#elif (SCREEN_WIDTH == 400)
        loadRawBitmap(battery, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 16, EPD_COLOR_BLACK);
#elif (SCREEN_WIDTH == 128)
        loadRawBitmap(battery, 112, 0, EPD_COLOR_BLACK);
#endif
        lowBatteryShown = true;
    } else {
        lowBatteryShown = false;
    }
}


void afterFlashScreenSaver() {
    selectLUT(EPD_LUT_DEFAULT);
    clearScreen();
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);

#if (SCREEN_WIDTH == 152)  // 1.54"
    epdPrintBegin(2, 2, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("OpenEPaperLink");
    epdPrintEnd();
#endif

#if (SCREEN_WIDTH == 128)  // 2.9"
    epdPrintBegin(0, 295, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("OpenEPaperLink");
    epdPrintEnd();
#endif
#if (SCREEN_WIDTH == 400)  // 4.2"
    epdPrintBegin(3, 3, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("OpenEPaperLink");
    epdPrintEnd();
#endif
    drawWithSleep();
}


#define VERSION_FMT "zbs_clock %d.%d.%d%s"

#if (SCREEN_WIDTH == 152)  // 1.54"
#define SPLASH_TITLE_X 5
#define SPLASH_TITLE_Y 55
#define SPLASH_MAC_X 5
#define SPLASH_MAC_Y 136
#define SPLASH_CAP_X 2
#define SPLASH_CAP_Y 104
#define SPLASH_VERSION_X 2
#define SPLASH_VERSION_Y 120
#elif (SCREEN_WIDTH == 128)  // 2.9"
#define SPLASH_TITLE_X 0
#define SPLASH_TITLE_Y 295
#define SPLASH_MAC_X 105
#define SPLASH_MAC_Y 270
#define SPLASH_CAP_X 64
#define SPLASH_CAP_Y 295
#define SPLASH_VERSION_X 80
#define SPLASH_VERSION_Y 295
#elif (SCREEN_WIDTH == 400)  // 4.2"
#define SPLASH_TITLE_X 3
#define SPLASH_TITLE_Y 3
#define SPLASH_MAC_X 3
#define SPLASH_MAC_Y 284
#define SPLASH_CAP_X 2
#define SPLASH_CAP_Y 252
#define SPLASH_VERSION_X 3
#define SPLASH_VERSION_Y 268
#endif

void showSplashScreen() {
    selectLUT(EPD_LUT_NO_REPEATS);
    clearScreen();
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);

    epdPrintBegin(SPLASH_TITLE_X, SPLASH_TITLE_Y, EPD_DIRECTION_Y, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("Startgni");
    switch (wakeUpReason) {
    case WAKEUP_REASON_TIMED:
        epdpr("(timed)");
        break;
    case WAKEUP_REASON_GPIO:
        epdpr("(gpio)");
        break;
    case WAKEUP_REASON_NFC:
        epdpr("(nfc)");
        break;
    case WAKEUP_REASON_FIRSTBOOT:
        epdpr("(first)");
        break;
    case WAKEUP_REASON_NETWORK_SCAN:
        epdpr("(scan)");
        break;
    case WAKEUP_REASON_WDT_RESET:
        epdpr("(reset)");
        break;
    default:
        epdpr("(unk)");
        break;
    }
    epdpr("0x%x", RESET);
    epdPrintEnd();

    epdPrintBegin(SPLASH_CAP_X, SPLASH_CAP_Y, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    addCapabilities();
    epdPrintEnd();

    epdPrintBegin(SPLASH_VERSION_X, SPLASH_VERSION_Y, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr(VERSION_FMT, fwVersion / 100, (fwVersion % 100) / 10, (fwVersion % 10), fwVersionSuffix);
    epdPrintEnd();

    epdPrintBegin(SPLASH_MAC_X, SPLASH_MAC_Y, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_RED);
    epdpr("MAC: %02X:%02X", mSelfMac[7], mSelfMac[6]);
    epdpr(":%02X:%02X", mSelfMac[5], mSelfMac[4]);
    epdpr(":%02X:%02X", mSelfMac[3], mSelfMac[2]);
    epdpr(":%02X:%02X", mSelfMac[1], mSelfMac[0]);
    epdPrintEnd();

    uint8_t __xdata buffer[17];
    spr(buffer, "%02X%02X", mSelfMac[7], mSelfMac[6]);
    spr(buffer + 4, "%02X%02X", mSelfMac[5], mSelfMac[4]);
    spr(buffer + 8, "%02X%02X", mSelfMac[3], mSelfMac[2]);
    spr(buffer + 12, "%02X%02X", mSelfMac[1], mSelfMac[0]);

#if (SCREEN_WIDTH == 152)  // 1.54"
    // nothing
#elif (SCREEN_WIDTH == 128)  // 2.9" 
    printBarcode(buffer, 120, 284);
#elif (SCREEN_WIDTH == 400)  // 4.2
    printBarcode(buffer, 392, 264);
    printBarcode(buffer, 384, 264);
#endif

    //loadRawBitmap(oepli, 0, 12, EPD_COLOR_BLACK);
    //loadRawBitmap(cloud, 0, 0, EPD_COLOR_RED);
    // lutTest();
    //  drawLineVertical(EPD_COLOR_RED, 64, 10, 286);
    //  drawLineVertical(EPD_COLOR_BLACK, 65, 10, 286);

    // timerDelay(TIMER_TICKS_PER_SECOND * 4);
    drawWithSleep();
}

void showClockDigital(int8_t hours, int8_t minutes, int8_t seconds) {
    selectLUT(EPD_LUT_NO_REPEATS); // what does this do?
    //selectLUT(EPD_LUT_FAST_NO_REDS); // presumably this means we do not want to paint any red?
    clearScreen();
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);

    // screen center is 148
    uint16_t y = (SCREEN_HEIGHT / 2) + 25; // ca 2.5 * chard width (maybe 2.5 * 8 pixels?)
    epdPrintBegin(48, y, EPD_DIRECTION_Y, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("%02d:%02d:%02d", hours, minutes,seconds);
    epdPrintEnd();
    draw();
    //drawNoWait(); // or with wait?
    //drawWithSleep(); // what does this exactly mean?
}

void showApplyUpdate() {
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);
    selectLUT(1);
    clearScreen();
    setColorMode(EPD_MODE_IGNORE, EPD_MODE_NORMAL);

#if (SCREEN_WIDTH == 152)
    epdPrintBegin(12, 60, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
#endif
#if (SCREEN_WIDTH == 128)
    epdPrintBegin(48, 220, EPD_DIRECTION_Y, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
#endif

#if (SCREEN_WIDTH == 400)
    epdPrintBegin(136, 134, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
#endif

    epdpr("Updating!");
    epdPrintEnd();
    drawNoWait();
}

uint8_t __xdata resultcounter = 0;

void showScanningWindow() {
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);
    selectLUT(EPD_LUT_FAST_NO_REDS);
    clearScreen();
#if (SCREEN_WIDTH == 128)  // 2.9"
    epdPrintBegin(2, 275, EPD_DIRECTION_Y, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("Scanning for APs");
    epdPrintEnd();
    // epdPrintBegin(40, 262, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_RED);
    // epdpr("Channel - Quality");
    // epdPrintEnd();
    loadRawBitmap(receive, 36, 24, EPD_COLOR_BLACK);
#endif
#if (SCREEN_WIDTH == 152)  // 1.54"
    loadRawBitmap(receive, 96, 28, EPD_COLOR_BLACK);
    epdPrintBegin(3, 0, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("Scanning...");
    epdPrintEnd();
#endif
#if (SCREEN_WIDTH == 400)  // 4.2"
    epdPrintBegin(2, 2, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("Scanning for APs");
    epdPrintEnd();

    // epdPrintBegin(2, 40, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_RED);
    // epdpr("Channel - Quality");
    // epdPrintEnd();
    loadRawBitmap(receive, 320, 125, EPD_COLOR_BLACK);
#endif

    draw();
    selectLUT(EPD_LUT_FAST);
    resultcounter = 0;
}

void addScanResult(uint8_t channel, uint8_t lqi) {
    if (channel == 11) resultcounter = 0;
#if (SCREEN_WIDTH == 128)  // 2.9"
    epdPrintBegin(56 + ((resultcounter % 4) * 16), 282 - (47 * (resultcounter / 4)), EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
#endif
#if (SCREEN_WIDTH == 152)  // 1.54"
    epdPrintBegin(4 + (47 * (resultcounter / 8)), 31 + (15 * (resultcounter % 8)), EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
#endif
#if (SCREEN_WIDTH == 400)  // 4.2"
    epdPrintBegin(4 + (47 * (resultcounter / 8)), 58 + (15 * (resultcounter % 8)), EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
#endif
    epdpr("%d-%d", channel, lqi);
    epdPrintEnd();
    resultcounter++;
}

void showAPFound() {
    clearScreen();
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);
    selectLUT(1);
#if (SCREEN_WIDTH == 128)
    epdPrintBegin(0, 285, EPD_DIRECTION_Y, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("Waiting for data...");
    epdPrintEnd();
    epdPrintBegin(48, 278, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Found the following AP:");
    epdPrintEnd();
    epdPrintBegin(64, 293, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("AP MAC: %02X:%02X", APmac[7], APmac[6]);
    epdpr(":%02X:%02X", APmac[5], APmac[4]);
    epdpr(":%02X:%02X", APmac[3], APmac[2]);
    epdpr(":%02X:%02X", APmac[1], APmac[0]);
    epdPrintEnd();
    epdPrintBegin(80, 293, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Ch: %d RSSI: %d LQI: %d", currentChannel, mLastRSSI, mLastLqi);
    epdPrintEnd();

    epdPrintBegin(103, 258, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Tag MAC: %02X:%02X", mSelfMac[7], mSelfMac[6]);
    epdpr(":%02X:%02X", mSelfMac[5], mSelfMac[4]);
    epdpr(":%02X:%02X", mSelfMac[3], mSelfMac[2]);
    epdpr(":%02X:%02X", mSelfMac[1], mSelfMac[0]);
    epdPrintEnd();

    uint8_t __xdata buffer[17];
    spr(buffer, "%02X%02X", mSelfMac[7], mSelfMac[6]);
    spr(buffer + 4, "%02X%02X", mSelfMac[5], mSelfMac[4]);
    spr(buffer + 8, "%02X%02X", mSelfMac[3], mSelfMac[2]);
    spr(buffer + 12, "%02X%02X", mSelfMac[1], mSelfMac[0]);
    printBarcode(buffer, 120, 253);
    loadRawBitmap(receive, 36, 14, EPD_COLOR_BLACK);
#endif
#if (SCREEN_WIDTH == 152)  // 1.54"
    epdPrintBegin(25, 0, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("Waiting");
    epdPrintEnd();
    epdPrintBegin(3, 32, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("for data...");
    epdPrintEnd();

    epdPrintBegin(5, 64, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("AP MAC:");
    epdPrintEnd();
    epdPrintBegin(5, 80, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("%02X%02X", APmac[7], APmac[6]);
    epdpr("%02X%02X", APmac[5], APmac[4]);
    epdpr("%02X%02X", APmac[3], APmac[2]);
    epdpr("%02X%02X", APmac[1], APmac[0]);
    epdPrintEnd();

    epdPrintBegin(5, 96, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Ch:%d rssi:%d lqi:%d", currentChannel, mLastRSSI, mLastLqi);
    epdPrintEnd();

    epdPrintBegin(5, 120, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Tag MAC:");
    epdPrintEnd();

    epdPrintBegin(5, 136, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_RED);
    epdpr("%02X%02X", mSelfMac[7], mSelfMac[6]);
    epdpr("%02X%02X", mSelfMac[5], mSelfMac[4]);
    epdpr("%02X%02X", mSelfMac[3], mSelfMac[2]);
    epdpr("%02X%02X", mSelfMac[1], mSelfMac[0]);
    epdPrintEnd();
#endif
#if (SCREEN_WIDTH == 400)
    epdPrintBegin(10, 10, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("Waiting for data...");
    epdPrintEnd();
    epdPrintBegin(48, 80, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Found the following AP:");
    epdPrintEnd();
    epdPrintBegin(48, 96, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("AP MAC: %02X:%02X", APmac[7], APmac[6]);
    epdpr(":%02X:%02X", APmac[5], APmac[4]);
    epdpr(":%02X:%02X", APmac[3], APmac[2]);
    epdpr(":%02X:%02X", APmac[1], APmac[0]);
    epdPrintEnd();
    epdPrintBegin(48, 112, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Ch: %d RSSI: %d LQI: %d", currentChannel, mLastRSSI, mLastLqi);
    epdPrintEnd();

    epdPrintBegin(366, 258, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Tag MAC: %02X:%02X", mSelfMac[7], mSelfMac[6]);
    epdpr(":%02X:%02X", mSelfMac[5], mSelfMac[4]);
    epdpr(":%02X:%02X", mSelfMac[3], mSelfMac[2]);
    epdpr(":%02X:%02X", mSelfMac[1], mSelfMac[0]);
    epdPrintEnd();

    uint8_t __xdata buffer[17];
    spr(buffer, "%02X%02X", mSelfMac[7], mSelfMac[6]);
    spr(buffer + 4, "%02X%02X", mSelfMac[5], mSelfMac[4]);
    spr(buffer + 8, "%02X%02X", mSelfMac[3], mSelfMac[2]);
    spr(buffer + 12, "%02X%02X", mSelfMac[1], mSelfMac[0]);
    printBarcode(buffer, 392, 253);
    printBarcode(buffer, 384, 253);
    loadRawBitmap(receive, 100, 170, EPD_COLOR_BLACK);
#endif
    addOverlay();
    drawWithSleep();
}

void showNoAP() {
    selectLUT(EPD_LUT_NO_REPEATS);
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);
    clearScreen();
#if (SCREEN_WIDTH == 128)  // 2,9"
    epdPrintBegin(0, 285, EPD_DIRECTION_Y, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("No AP found :(");
    epdPrintEnd();
    epdPrintBegin(48, 285, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("We'll try again in a");
    epdPrintEnd();
    epdPrintBegin(64, 285, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("little while...");
    epdPrintEnd();
    loadRawBitmap(receive, 36, 24, EPD_COLOR_BLACK);
    loadRawBitmap(failed, 42, 26, EPD_COLOR_RED);
#endif
#if (SCREEN_WIDTH == 152)  // 1.54"
    epdPrintBegin(40, 0, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("No AP");
    epdPrintEnd();
    epdPrintBegin(22, 32, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("found :(");
    epdPrintEnd();

    epdPrintBegin(8, 76, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("We'll try again in");
    epdPrintEnd();
    epdPrintBegin(25, 92, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("a little while");
    epdPrintEnd();
#endif
#if (SCREEN_WIDTH == 400)  // 4.2"
    epdPrintBegin(10, 10, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("No AP found :(");
    epdPrintEnd();
    epdPrintBegin(10, 274, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("We'll try again in a little while");
    epdPrintEnd();
    loadRawBitmap(receive, 76, 120, EPD_COLOR_BLACK);
    loadRawBitmap(failed, 82, 122, EPD_COLOR_RED);
#endif
    addOverlay();
    drawWithSleep();
}

void showLongTermSleep() {
    selectLUT(EPD_LUT_NO_REPEATS);
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);
    clearScreen();
#if (SCREEN_WIDTH == 128)  // 2.9"
    epdPrintBegin(0, 295, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("zZ");
    epdPrintEnd();
#else
    epdPrintBegin(2, SCREEN_HEIGHT - 16, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("zZ");
    epdPrintEnd();
#endif
    addOverlay();
    drawWithSleep();
}
void showNoEEPROM() {
    selectLUT(EPD_LUT_NO_REPEATS);
    clearScreen();
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);
#if (SCREEN_WIDTH == 128)  // 2.9"
    epdPrintBegin(0, 285, EPD_DIRECTION_Y, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("EEPROM FAILED :(");
    epdPrintEnd();
    epdPrintBegin(64, 285, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Sleeping forever :'(");
    epdPrintEnd();
    loadRawBitmap(failed, 42, 26, EPD_COLOR_RED);
#endif
#if (SCREEN_WIDTH == 152)  // 1.54"
    epdPrintBegin(26, 0, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("EEPROM ");
    epdPrintEnd();
    epdPrintBegin(8, 32, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("FAILED :(");
    epdPrintEnd();
    loadRawBitmap(failed, 60, 72, EPD_COLOR_RED);

    epdPrintBegin(3, 136, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Sleeping forever :'(");
    epdPrintEnd();
#endif
#if (SCREEN_WIDTH == 400)  // 4.2"
    epdPrintBegin(50, 3, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("EEPROM FAILED :(");
    epdPrintEnd();
    loadRawBitmap(failed, 176, 126, EPD_COLOR_RED);
    epdPrintBegin(100, 284, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Sleeping forever :'(");
    epdPrintEnd();
#endif
    drawWithSleep();
}

void showNoMAC() {
    selectLUT(EPD_LUT_NO_REPEATS);
    clearScreen();
    setColorMode(EPD_MODE_NORMAL, EPD_MODE_INVERT);
#if (SCREEN_WIDTH == 128)  // 2.9"
    epdPrintBegin(0, 285, EPD_DIRECTION_Y, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("NO MAC SET :(");
    epdPrintEnd();
    epdPrintBegin(64, 285, EPD_DIRECTION_Y, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Sleeping forever :'(");
    epdPrintEnd();
    loadRawBitmap(failed, 42, 26, EPD_COLOR_RED);
#endif
#if (SCREEN_WIDTH == 152)  // 1.54"
    epdPrintBegin(20, 0, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("NO MAC");
    epdPrintEnd();
    epdPrintBegin(30, 32, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("SET :(");
    epdPrintEnd();
    loadRawBitmap(failed, 60, 72, EPD_COLOR_RED);
    epdPrintBegin(3, 136, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Sleeping forever :'(");
    epdPrintEnd();
#endif
#if (SCREEN_WIDTH == 400)  // 4.2"
    epdPrintBegin(100, 3, EPD_DIRECTION_X, EPD_SIZE_DOUBLE, EPD_COLOR_BLACK);
    epdpr("NO MAC SET :(");
    epdPrintEnd();
    loadRawBitmap(failed, 176, 126, EPD_COLOR_RED);
    epdPrintBegin(100, 284, EPD_DIRECTION_X, EPD_SIZE_SINGLE, EPD_COLOR_BLACK);
    epdpr("Sleeping forever :'(");
    epdPrintEnd();
#endif
    drawWithSleep();
}
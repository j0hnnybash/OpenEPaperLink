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

void timed_sleep1(uint32_t length)
{
	__bit irqEn = IEN_EA; // save previous IRQ state
	uint8_t prescaler;
	IEN_EA = 0; // IRQs off
	
	if (!length)
		length = 0xfffffffful;

        // is this a cheap variant of 	powerDown(INIT_RADIO); ? it matches except for the radioRxEnable(false,true);
        // since IRQs are off this is 100% equivalent... except a few instructions shorter...
	RADIO_IRQ4_pending = 0;
	UNK_C1 &=~ 0x81;
	TCON &=~ 0x20; // clear T= overflow bit
	RADIO_command = RADIO_CMD_UNK_2;
	RADIO_command = RADIO_CMD_UNK_3;
        //powerDown(INIT_RADIO); // this would be slower, but equivalent and actually less code...

        // sleep timer seems to count down?
	if (length <= 0x00008000ul) {
		length <<= 5;
		prescaler = 0x56;		//0x56 = one tick is 1/32k of sec
	}
	else {
		if (length != 0xfffffffful)
			length += 500;
		length /= 1000;
		prescaler = 0x16;		//0x16 = one tick is 1 second
	}
	if (length > 0x000fffff) {
		RADIO_SleepTimerLo = 0xff;
		RADIO_SleepTimerMid = 0xff;
		RADIO_SleepTimerHi = 0x0f;
	}
	else {
		RADIO_SleepTimerLo = length;
		RADIO_SleepTimerMid = length >> 8;
		RADIO_SleepTimerHi = ((uint8_t)(length >> 16)) & 0x0f;
	}

	__asm__("nop");
	RADIO_SleepTimerSettings = prescaler;
	__asm__("nop\nnop\nnop\nnop\n");
	RADIO_SleepTimerSettings |= 0x80;
	__asm__("nop\nnop\n");
	RADIO_RadioPowerCtl = 0x44;
	__asm__("nop\nnop\n");

        // Why do we need this busy sleep here?!
	//make sure time does not run backwards
	/* TL0 = 0x0; */
	/* TH0 = 0xFF; */
	/* while (TH0 == 0xFF); // so this busily waits for 8bits of count on timer 0?! */

        // this is the reverse of the clearing of these flags above, identical to the powerDown(INIT_RADIO)?
        // but it is not equivalent to powerUp(INIT_RADIO)
	//UNK_C1 |= 0x81;
	//TCON |= TCON_TF0_MASK; /* FIXME: why would we write to it? isnt this a read only value? also why dont we use bit addressing? this is cleared by some ISRs so maybe we want to detect that..?*/
        // maybe Timer 0 does not signal overflow if IRQs are off, and this is needed to emulate the behavior of Timer 0 overflowing

	IEN_EA = irqEn;
}

#define GPIO_IN 1
#define GPIO_OUT 0
#define GPIO_PULLUP_ON 1
#define GPIO_PULLUP_OFF 0
// TODO: pin change interrupts


/* static void clr_bit(uint8_t * p, int n) { */
/*     *p &= ~(1U << n); */
/* } */
/* static void set_bit(uint8_t * p, int n) { */
/*     *p |= (1U << n); */
/* } */
#define clr_bit(p,n) ((p) &= ~(1U << (n)))
#define set_bit(p,n) ((p) |= (1U << (n)))

static void setup_gpio_1(int pin, int dir, int pullup) {
    // 1 - func, 0 - gpio
    clr_bit(P1FUNC, pin);
    // 1 - in, 0 - out
    if (dir == GPIO_IN) {
        set_bit(P1DIR, pin);
    } else {
        clr_bit(P1DIR, pin);
    }
    // 1 - pull up enable, 0 - disable
    if (pullup == GPIO_PULLUP_ON) {
        set_bit(P1PULL, pin);
    } else {
        clr_bit(P1PULL, pin);
    }
    /* // ? */
    /* P1LVLSEL |= (1 << 0); */
    /* // change lvl interrupt enable */
    /* P1INTEN = (1 << 0); */
    /* // pin changed status, 1=changed, (clear _before_ enabling interrupt?!) */
    /* P1CHSTA &= ~(1 << 0); */
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
    powerDown(INIT_EPD); // if we don't power down the EPD it will not display anything next time around.
    //doSleep(5000UL);
    //wdt30s();
    //powerUp(INIT_EPD); // doesn't seem to be idempotent, requires preceeding powerDown otherwise nothing will be drawn.

    // power up GPIO?, not really configures GPIO so EPD can be addressed
    //epdConfigGPIO(true);


    // Pin P1.0 is "testpoint"
    //setup_gpio_1(0, GPIO_OUT, GPIO_PULLUP_OFF);
    P1DIR &= ~((1 << 0));
    P1_0 = 0;
    //timed_sleep1(500); // on
    timerDelay(TIMER_TICKS_PER_SECOND / 2);
    P1_0 = 1;
    //timed_sleep1(500); // off
    timerDelay(TIMER_TICKS_PER_SECOND / 2);
    P1_0 = 0;
    //timed_sleep1(500); // on, crash
    timerDelay(TIMER_TICKS_PER_SECOND / 2);
    P1_0 = 1;
    P1DIR |= (1 << 0);
    //timed_sleep1(500);
    timerDelay(TIMER_TICKS_PER_SECOND / 2);

    // power down GPIO?
    //epdConfigGPIO(false);

    //powerUp(INIT_EPD);
    //powerDown(INIT_EPD | INIT_UART);
    powerDown(INIT_UART);
    
    int8_t hours = 0;
    int8_t minutes = 0;

    // sleepFotMsec()?
    //doSleep(5000UL); ?
    // Stopwatch
    while (1) {
        powerUp(INIT_EPD);
        showClockDigital(hours, minutes, 0);
        powerDown(INIT_EPD);
        wdt120s();
        //sleepForMsec(60e3);
        timed_sleep1(60e3);
        minutes += 1;
        if (minutes > 59) {
            minutes -= 60;
            hours += 1;
        }
    }
}

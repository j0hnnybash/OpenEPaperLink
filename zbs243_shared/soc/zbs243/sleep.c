#include "sleep.h"
#include "cpu.h"
#include "powermgt.h"

void sleepForMsec(uint32_t length)
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
	TCON &=~ 0x20;
	RADIO_command = RADIO_CMD_UNK_2;
	RADIO_command = RADIO_CMD_UNK_3;
        //powerDown(INIT_RADIO); // this would be slower, but equivalent and actually less code...

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

	//make sure time does not run backwards
	TL0 = 0x0;
	TH0 = 0xFF;
	while (TH0 == 0xFF); // so this busily waits for 8bits of count on timer 0?!
	
	UNK_C1 |= 0x81;
	TCON |= TCON_TF0_MASK; /* FIXME: why would we write to it? isnt this a read only value? also why dont we use bit addressing? */
	
	IEN_EA = irqEn;
}

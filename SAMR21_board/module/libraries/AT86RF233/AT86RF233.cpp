/*
 * AT86RF233.c
 *
 *  Created on: May 5, 2018
 *      Author: doru
 */
#include <Arduino.h>
#include "AT86RF233.h"

AT86RF233Class::AT86RF233Class()
{
	sercom = &PERIPH_SPI1;
}

AT86RF233Class::~AT86RF233Class()
{

}

void setPinConf(PortGroup *port, uint32_t pin_mask)
{
	uint32_t pin_conf = 0;
	//enable peripheral flag MUX
	pin_conf |= PORT_WRCONFIG_PMUXEN;
	pin_conf |= 0x5 << PORT_WRCONFIG_PMUX_Pos;
	//enable input buffer flag
	pin_conf |= PORT_WRCONFIG_INEN;
	//enable pull down control flag
	pin_conf |= PORT_WRCONFIG_PULLEN;

	//clear the port dir bits to disable the output buffer
	port->DIRCLR.reg = pin_mask;

	uint32_t lower_pin_mask = (pin_mask & 0xFFFF);
	uint32_t upper_pin_mask = (pin_mask >> 16);

	port->WRCONFIG.reg = (lower_pin_mask << PORT_WRCONFIG_PINMASK_Pos) |
	pin_conf | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_WRPINCFG;

	port->WRCONFIG.reg = (upper_pin_mask << PORT_WRCONFIG_PINMASK_Pos) |
	pin_conf | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_WRPINCFG |
	PORT_WRCONFIG_HWSEL;

	port->OUTSET.reg = pin_mask;
}

void disableClockChannel(uint8_t channel)
{
	*((uint8_t*)&GCLK->CLKCTRL.reg) = channel;
	/* Switch to known-working source so that the channel can be disabled */
	uint32_t prev_id = GCLK->CLKCTRL.bit.GEN;
	GCLK->CLKCTRL.bit.GEN = 0;

	/* Disable the generic clock */
	GCLK->CLKCTRL.reg &= ~GCLK_CLKCTRL_CLKEN;
	while (GCLK->CLKCTRL.reg & GCLK_CLKCTRL_CLKEN) {
		/* Wait for clock to become disabled */
	}
	/* Restore previous configured clock generator */
	GCLK->CLKCTRL.bit.GEN = prev_id;
}

void enableClockChannel(uint8_t channel)
{
	*((uint8_t*)&GCLK->CLKCTRL.reg) = channel;
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN;
}

void AT86RF233Class::setupClock()
{
	disableClockChannel(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val);
	GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_ID_SERCOM4_CORE_Val << GCLK_CLKCTRL_ID_Pos);
	enableClockChannel(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val);
	disableClockChannel(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val);
	GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val << GCLK_CLKCTRL_ID_Pos);
	enableClockChannel(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val);
}
#if IRQ 
void AT86RF233Class::configureISR(void (*isr)(void))
{
	EExt_Interrupts in = g_APinDescription[PIN_ATRF233_IRQ].ulExtInt;
	uint32_t inMask = (1UL << in);
	int current = 0;
	for (current=0; current<nints; current++) {
		if (ISRlist[current] == inMask) {
			break;
		}
	}
	if (current == nints) {
		// Need to make a new entry
		nints++;
	}
	ISRlist[current] = inMask;       // List of interrupt in order of when they were attached
	ISRcallback[current] = isr;
	//enable isr
	EIC->INTENSET.reg = EIC_INTENSET_EXTINT(inMask);
}
#endif
void AT86RF233Class::setupSPI()
{
	/**********************************
	*	Setup SS pin
	***********************************/
	uint32_t ss_pin_conf = 0;
	uint32_t ss_pin_mask = (1UL << 31);

	ss_pin_conf &= ~PORT_WRCONFIG_PULLEN;
	uint32_t lower_pin_mask = (ss_pin_mask & 0xFFFF);
	uint32_t upper_pin_mask = (ss_pin_mask >> 16);

	PORT->Group[PORTB].WRCONFIG.reg
	= (lower_pin_mask << PORT_WRCONFIG_PINMASK_Pos) |
	ss_pin_conf | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_WRPINCFG;

	PORT->Group[PORTB].WRCONFIG.reg
	= (upper_pin_mask << PORT_WRCONFIG_PINMASK_Pos) |
	ss_pin_conf | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_WRPINCFG |
	PORT_WRCONFIG_HWSEL;

	PORT->Group[PORTB].DIRSET.reg = ss_pin_mask;
	PORT->Group[PORTB].OUTSET.reg = ss_pin_mask;

	/**********************************
	*	Enable Sercom4
	***********************************/
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM4;

	/**********************************
	*	Setup GCLK
	***********************************/
	setupClock();

	/**********************************
	*	Setup SPI
	***********************************/
	while(SERCOM4->SPI.SYNCBUSY.bit.ENABLE);
	SERCOM4->SPI.CTRLA.bit.ENABLE = 0; //disable SPI
	SERCOM4->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_MODE_SPI_MASTER;
	setPinConf(&PORT->Group[PORTC], (1UL << 19));
	setPinConf(&PORT->Group[PORTB], (1UL << 30));
	setPinConf(&PORT->Group[PORTC], (1UL << 18));

	uint32_t clock = 4000000;
	uint32_t clockFreq = (clock >= (F_CPU / SPI_MIN_CLOCK_DIVIDER) ? F_CPU / SPI_MIN_CLOCK_DIVIDER : clock);
	uint8_t baud = (clockFreq/2 - 1);
	SERCOM4->SPI.BAUD.reg = (uint8_t)baud;
	SERCOM4->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_DOPO(SPI_PAD_2_SCK_3) |
						  SERCOM_SPI_CTRLA_DIPO(SERCOM_RX_PAD_0) |
						  SERCOM_SPI_CTRLA_RUNSTDBY |
						  MSB_FIRST << SERCOM_SPI_CTRLA_DORD_Pos;

	SERCOM4->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(SPI_CHAR_SIZE_8_BITS) |
							 SERCOM_SPI_CTRLB_RXEN | SERCOM_SPI_CTRLB_SSDE;

	SERCOM4->SPI.CTRLA.bit.ENABLE = 1;

	while(SERCOM4->SPI.SYNCBUSY.bit.ENABLE)
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}
	#if IRQ
	/**********************************
	*	Setup IRQ - PB00
	***********************************/
	uint32_t irq_pin_cfg = 0;
	uint32_t irq_pin_mask = (1UL << 0);
	irq_pin_cfg |= PORT_WRCONFIG_PMUXEN;
	irq_pin_cfg |= (0x0 << PORT_WRCONFIG_PMUX_Pos);
	irq_pin_cfg |= PORT_WRCONFIG_INEN;
	irq_pin_cfg |= PORT_WRCONFIG_PULLEN;
	PORT->Group[PORTB].DIRCLR.reg = irq_pin_mask;

	uint32_t lower_pin_mask = (irq_pin_mask & 0xFFFF);
	uint32_t upper_pin_mask = (irq_pin_mask >> 16);

	PORT->Group[PORTB].WRCONFIG.reg = (lower_pin_mask << PORT_WRCONFIG_PINMASK_Pos) |
									  irq_pin_cfg | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_WRPINCFG;

	PORT->Group[PORTB].WRCONFIG.reg = (upper_pin_mask << PORT_WRCONFIG_PINMASK_Pos) |
									  irq_pin_cfg | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_WRPINCFG |
									  PORT_WRCONFIG_HWSEL;
	PORT->Group[PORTB].OUTCLR.reg = irq_pin_mask;
	uint32_t channel = (4 * (channel % 8));
	uint32_t config_pos = 0;
	uint32_t new_config = 1; //RISING
	/* Clear the existing and set the new channel configuration */
	EIC->CONFIG[channel/8].reg = (EIC->CONFIG[channel/8].reg & ~((EIC_CONFIG_SENSE0_Msk | EIC_CONFIG_FILTEN0) << config_pos)) |
						(new_config << config_pos);

	/* Set the channel's new wake up mode setting */
	EIC->WAKEUP.reg |=  (1UL << 0);
	#endif
}


void AT86RF233Class::begin()
{
	//disable sercom4
	PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM4;
	setupSPI();
	delay(10);
	//*TODO: Modify pinPeripheral(pin, PIO_EXTINT) so that we can use pinMode and attachInterrupt 
}

uint8_t AT86RF233Class::send(uint8_t toSend)
{
	uint8_t c = pRegisterAccess_R | pRegAddr_XOSC_CTRL;
	Serial.println("Command: ");
	Serial.print(c);
	Serial.println("\n");

	digitalWrite(PIN_SPI1_SS, LOW);
	uint8_t res1 = sercom->transferDataSPI(c);
	uint8_t res2 = sercom->transferDataSPI(toSend);
	digitalWrite(PIN_SPI1_SS, HIGH);
	
	char txt[100];
	sprintf(txt, "\nDataReg: %#X", res1);
	Serial.println("Send first Byte ");
	Serial.println(txt);
	sprintf(txt, "\nDataReg: %#X", res2);
	Serial.println("\Send second Byte ");
	Serial.println(txt);
	Serial.println("\n");
	return (uint8_t)res2;
}

uint8_t AT86RF233Class::receive()
{
	digitalWrite(PIN_SPI1_SS, LOW);
	uint8_t c = pRegisterAccess_R | pRegAddr_XOSC_CTRL;
	sercom->transferDataSPI(c);
	uint8_t res = sercom->transferDataSPI(0x00);
	digitalWrite(PIN_SPI1_SS, HIGH);
	return (uint8_t)res;
}

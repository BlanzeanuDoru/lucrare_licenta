/*
 * Copyright (C) 2013 Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_at86rf2xx
 * @{
 *
 * @file
 * @brief       Implementation of public functions for AT86RF2xx drivers
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Mark Solters <msolters@gmail.com>
 *
 * @}
 */

#include <Arduino.h>
#include <SPI.h>

#include "at86rf2xx.h"

/*  Declare radio device as globally scoped struct  */
AT86RF2XX at86rf2xx = AT86RF2XX();

/**
 * @brief   Increments events count by  1.
 */
static void at86rf2xx_irq_handler()
{
    at86rf2xx.events++;
    return;
}

AT86RF2XX::AT86RF2XX() {}

void AT86RF2XX::disableClockChannel(uint8_t channel)
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

void AT86RF2XX::enableClockChannel(uint8_t channel)
{
	*((uint8_t*)&GCLK->CLKCTRL.reg) = channel;
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN;
}

void AT86RF2XX::setupClock()
{
	disableClockChannel(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val);
	GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_ID_SERCOM4_CORE_Val << GCLK_CLKCTRL_ID_Pos);
	enableClockChannel(GCLK_CLKCTRL_ID_SERCOM4_CORE_Val);
	disableClockChannel(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val);
	GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val << GCLK_CLKCTRL_ID_Pos);
	enableClockChannel(GCLK_CLKCTRL_ID_SERCOMX_SLOW_Val);
}

int AT86RF2XX::init()
{
	cs_pin = PIN_ATRF233_SS;
	int_pin = PIN_ATRF233_IRQ;
	sleep_pin = PIN_ATRF233_SLP;
	reset_pin = PIN_ATRF233_RST;
	idle_state = AT86RF2XX_STATE_TRX_OFF;
	state = AT86RF2XX_STATE_SLEEP;

	/**********************************
	*	Disable sercom4
	***********************************/
    PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM4;
	/**********************************
	*	Setup SS pin
	***********************************/
	pinMode(PIN_ATRF233_SS, OUTPUT);
	digitalWrite(PIN_ATRF233_SS, HIGH);
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
	pinMode(PIN_ATRF233_MISO, INPUT_PULLDOWN);
	digitalWrite(PIN_ATRF233_MISO, HIGH);
	pinMode(PIN_ATRF233_MOSI, INPUT_PULLDOWN);
	digitalWrite(PIN_ATRF233_MOSI, HIGH);
	pinMode(PIN_ATRF233_SCK, INPUT_PULLDOWN);
	digitalWrite(PIN_ATRF233_SCK, HIGH);
	
	SPI1.begin();
	SPI1.setBitOrder(MSBFIRST);
	SPI1.setClockDivider(SPI_CLOCK_DIV8);
	SPI1.setDataMode(SPI_MODE0);

	pinMode(PIN_ATRF233_IRQ, INPUT_PULLDOWN);
	digitalWrite(PIN_ATRF233_IRQ, LOW);
	attachInterrupt(PIN_ATRF233_IRQ, at86rf2xx_irq_handler, RISING);
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


	/* make sure device is not sleeping, so we can query part number */
	assert_awake();

	/* test if the SPI is set up correctly and the device is responding */
	byte part_num = reg_read(AT86RF2XX_REG__PART_NUM);
	if (part_num != AT86RF233_PARTNUM) {
		Serial.println("[at86rf2xx] Error: unable to read correct part number.");
		return -1;
	}
	Serial.print("[at86rf2xx] Detected part #: 0x");
	Serial.println(part_num, HEX);
	Serial.print("[at86rf2xx] Version: 0x");
	Serial.println(reg_read(AT86RF2XX_REG__VERSION_NUM), HEX);

	/* reset device to default values and put it into RX state */
    reset();

    return 0;
}

void AT86RF2XX::reset()
{
    hardware_reset();

    /* Reset state machine to ensure a known state */
    reset_state_machine();

    /* reset options and sequence number */
    seq_nr = 0;
    options = 0;

    /* set short and long address */
    set_addr_long(AT86RF2XX_DEFAULT_ADDR_LONG);
    set_addr_short(AT86RF2XX_DEFAULT_ADDR_SHORT);

    /* set default PAN id */
    set_pan(AT86RF2XX_DEFAULT_PANID);

    /* set default channel */
    set_chan(AT86RF2XX_DEFAULT_CHANNEL);

    /* set default TX power */
    set_txpower(AT86RF2XX_DEFAULT_TXPOWER);

    /* set default options */
    set_option(AT86RF2XX_OPT_PROMISCUOUS, true);
    set_option(AT86RF2XX_OPT_AUTOACK, true);
    set_option(AT86RF2XX_OPT_CSMA, true);
    set_option(AT86RF2XX_OPT_TELL_RX_START, true);
    set_option(AT86RF2XX_OPT_TELL_RX_END, true);

    /* enable safe mode (protect RX FIFO until reading data starts) */
    reg_write(AT86RF2XX_REG__TRX_CTRL_2, AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE);

//#ifdef MODULE_AT86RF212B
//    at86rf2xx_set_freq(dev, AT86RF2XX_FREQ_915MHZ);
//#endif

    /* don't populate masked interrupt flags to IRQ_STATUS register */
    /*uint8_t tmp = at86rf2xx_reg_read(AT86RF2XX_REG__TRX_CTRL_1);
    tmp &= ~(AT86RF2XX_TRX_CTRL_1_MASK__IRQ_MASK_MODE);
    at86rf2xx_reg_write(AT86RF2XX_REG__TRX_CTRL_1, tmp);*/

    /* disable clock output to save power */
    byte tmp = reg_read(AT86RF2XX_REG__TRX_CTRL_0);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
    tmp |= (AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF);
    reg_write(AT86RF2XX_REG__TRX_CTRL_0, tmp);

    /* enable interrupts */
    reg_write(AT86RF2XX_REG__IRQ_MASK, AT86RF2XX_IRQ_STATUS_MASK__TRX_END);

    /* clear interrupt flags */
    reg_read(AT86RF2XX_REG__IRQ_STATUS);

    /* go into RX state */
    set_state(AT86RF2XX_STATE_RX_AACK_ON);

    Serial.println("[at86rf2xx] Reset complete.");
}

bool AT86RF2XX::cca()
{
    uint8_t tmp;
    uint8_t status;

    assert_awake();

    /* trigger CCA measurment */
    tmp = reg_read(AT86RF2XX_REG__PHY_CC_CCA);
    tmp &= AT86RF2XX_PHY_CC_CCA_MASK__CCA_REQUEST;
    reg_write(AT86RF2XX_REG__PHY_CC_CCA, tmp);

    /* wait for result to be ready */
    do {
        status = reg_read(AT86RF2XX_REG__TRX_STATUS);
    } while (!(status & AT86RF2XX_TRX_STATUS_MASK__CCA_DONE));

    /* return according to measurement */
    if (status & AT86RF2XX_TRX_STATUS_MASK__CCA_STATUS) {
        return true;
    }
    else {
        return false;
    }
}

size_t AT86RF2XX::send(uint8_t *data, size_t len)
{
    /* check data length */
    if (len > AT86RF2XX_MAX_PKT_LENGTH) {
        Serial.println("[at86rf2xx] Error: Data to send exceeds max packet size.");
        return 0;
    }
    AT86RF2XX::tx_prepare();
    AT86RF2XX::tx_load(data, len, 0);
    AT86RF2XX::tx_exec();
    return len;
}

void AT86RF2XX::tx_prepare()
{
    uint8_t state;

    /* make sure ongoing transmissions are finished */
    do {
        state = get_status();
    }
    while (state == AT86RF2XX_STATE_BUSY_TX_ARET);

    /* if receiving cancel */
    if(state == AT86RF2XX_STATE_BUSY_RX_AACK) {
        force_trx_off();
        idle_state = AT86RF2XX_STATE_RX_AACK_ON;
    } else if (state != AT86RF2XX_STATE_TX_ARET_ON) {
        idle_state = state;
    }
    set_state(AT86RF2XX_STATE_TX_ARET_ON);
    frame_len = IEEE802154_FCS_LEN;
}

size_t AT86RF2XX::tx_load(uint8_t *data,
                         size_t len, size_t offset)
{
    frame_len += (uint8_t)len;
    sram_write(offset + 1, data, len);
    return offset + len;
}

void AT86RF2XX::tx_exec()
{
    /* write frame length field in FIFO */
    sram_write(0, &(frame_len), 1);
    /* trigger sending of pre-loaded frame */
    reg_write(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__TX_START);
    /*if (at86rf2xx.event_cb && (at86rf2xx.options & AT86RF2XX_OPT_TELL_TX_START)) {
        at86rf2xx.event_cb(NETDEV_EVENT_TX_STARTED, NULL);
    }*/
}

size_t AT86RF2XX::rx_len()
{
    uint8_t phr;
    fb_read(&phr, 1);

    /* ignore MSB (refer p.80) and substract length of FCS field */
    return (size_t)((phr & 0x7f) - 2);
}

void AT86RF2XX::rx_read(uint8_t *data, size_t len, size_t offset)
{
    /* when reading from SRAM, the different chips from the AT86RF2xx family
     * behave differently: the AT86F233, the AT86RF232 and the ATRF86212B return
     * frame length field (PHR) at position 0 and the first data byte at
     * position 1.
     * The AT86RF231 does not return the PHR field and return
     * the first data byte at position 0.
     */
#ifndef MODULE_AT86RF231
    sram_read(offset + 1, data, len);
#else
    sram_read(offset, data, len);
#endif
}

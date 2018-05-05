/*
 * AT86RF233.h
 *
 *  Created on: May 5, 2018
 *      Author: doru blanzeanu
 */

#ifndef _AT86RF233_AT86RF233_H_
#define _AT86RF233_AT86RF233_H_

/*INCLUDES*/
#include <Arduino.h>
#include <SPI.h>

/*Table 35-2. SPI Command Byte Definition*/
typedef enum
{
	pRegisterAccess_R = (1 << 7),
	pRegisterAccess_W = (3 << 6),
	pFrameBufferAccess_R = (1 << 5),
	pFrameBufferAccess_W = (3 << 5),
	pSRAMAccess_R = 0,
	pSRAMAccess_W = (1 << 6),
} pRFAccessMode;

typedef enum
{
	pRegAddr_TRX_STATUS     = 0x01,
	pRegAddr_TRX_STATE      = 0x02,
	pRegAddr_TRX_CTRL_0     = 0x03,
	pRegAddr_TRX_CTRL_1     = 0x04,
	pRegAddr_PHY_TX_PWR     = 0x05,
	pRegAddr_PHY_RSSI       = 0x06,
	pRegAddr_PHY_ED_LEVEL   = 0x07,
	pRegAddr_PHY_CC_CCA     = 0x08,
	pRegAddr_CCA_THRES      = 0x09,
	pRegAddr_RX_CTRL        = 0x0A,
	pRegAddr_SFD_VALUE      = 0x0B,
	pRegAddr_TRX_CTRL_2     = 0x0C,
	pRegAddr_ANT_DIV        = 0x0D,
	pRegAddr_IRQ_MASK       = 0x0E,
	pRegAddr_IRQ_STATUS     = 0x0F,
	pRegAddr_VREG_CTRL      = 0x10,
	pRegAddr_BATMON         = 0x11,
	pRegAddr_XOSC_CTRL      = 0x12,
} pRFRegAddr;

class AT86RF233Class
{
public:
	AT86RF233Class(const pRFAccessMode, const pRFAccessMode);
	~AT86RF233Class();
	void begin();
	void send(const char);
	char receive();

private:
	pRFAccessMode sendMode;
	pRFAccessMode receiveMode;
};

#endif /* _AT86RF233_AT86RF233_H_ */

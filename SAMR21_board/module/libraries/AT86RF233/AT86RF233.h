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

#define PACK(access, reg, data) ((short)(((((access) << 6) + (reg)) << 8) + (data)))
#define REG_RFCTRL_FECFG (*(RwReg16*)0x42005400U)
#define RFCTRL_CFG_ANT_DIV 4

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
	pRegAddr_CC_CTRL_0      = 0x13,
	pRegAddr_CC_CTRL_1      = 0x14,
	pRegAddr_RX_SYN         = 0x15,
	pRegAddr_TRX_RPC        = 0x16,
	pRegAddr_XAH_CTRL_1     = 0x17,
	pRegAddr_FTN_CTRL       = 0x18,
	pRegAddr_XAH_CTRL_2     = 0x19,
	pRegAddr_PLL_CF         = 0x1A,
	pRegAddr_PLL_DCU        = 0x1B,
	pRegAddr_PART_NUM       = 0x1C,
	pRegAddr_VERSION_NUM    = 0x1D,
	pRegAddr_MAN_ID_0       = 0x1E,
	pRegAddr_MAN_ID_1       = 0x1F,
	pRegAddr_SHORT_ADDR_0   = 0x20,
	pRegAddr_SHORT_ADDR_1   = 0x21,
	pRegAddr_PAN_ID_0       = 0x22,
	pRegAddr_PAN_ID_1       = 0x23,
	pRegAddr_IEEE_ADDR_0    = 0x24,
	pRegAddr_IEEE_ADDR_1    = 0x25,
	pRegAddr_IEEE_ADDR_2    = 0x26,
	pRegAddr_IEEE_ADDR_3    = 0x27,
	pRegAddr_IEEE_ADDR_4    = 0x28,
	pRegAddr_IEEE_ADDR_5    = 0x29,
	pRegAddr_IEEE_ADDR_6    = 0x2A,
	pRegAddr_IEEE_ADDR_7    = 0x2B,
	pRegAddr_XAH_CTRL_0     = 0x2C,
	pRegAddr_CSMA_SEED_0    = 0x2D,
	pRegAddr_CSMA_SEED_1    = 0x2E,
	pRegAddr_CSMA_BE        = 0x2F,
	pRegAddr_TST_CTRL_DIGI  = 0x36,
	pRegAddr_TST_AGC        = 0x3C,
	pRegAddr_TST_SDM        = 0x3D,
	pRegAddr_PHY_TX_TIME    = 0x3B,
} pRFRegAddr;

#define IRQ 0
class AT86RF233Class
{
public:
	AT86RF233Class();
	~AT86RF233Class();
	void begin();
	uint8_t send(const uint8_t);
	uint8_t receive();
	#if IRQ
	void configureISR(void (*isr)(void));
	#endif
private:
	void setAlternateFunctions();
	void setupClock();
	void setupSPI();
	SERCOM *sercom;
};

#endif /* _AT86RF233_AT86RF233_H_ */

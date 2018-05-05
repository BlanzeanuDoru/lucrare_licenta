/*
 * AT86RF233.c
 *
 *  Created on: May 5, 2018
 *      Author: doru
 */

#include "AT86RF233.h"

AT86RF233Class::AT86RF233Class(const pRFAccessMode sM, const pRFAccessMode rM):
	sendMode(sM), receiveMode(rM)
{
	SPI1.begin();
}

AT86RF233Class::~AT86RF233Class()
{

}

void AT86RF233Class::begin()
{

}

void AT86RF233Class::send(const char toSend)
{
	SPI1.transfer16()
}

char AT86RF233Class::receive()
{

}

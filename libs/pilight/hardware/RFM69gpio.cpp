/*
	Copyright (C) 2015 chrillomat

	This file is part of pilight.

	pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with pilight. If not, see	<http://www.gnu.org/licenses/>
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern "C" {
#include "../core/pilight.h"
#include "../core/common.h"
#include "../core/dso.h"
#include "../core/log.h"
#include "../core/json.h"
#include "../core/irq.h"
#include "../config/hardware.h"
#include "../../wiringx/wiringX.h"
// TODO: RFM69 class is C++ code and causes unwanted mess here -> convert?
#include "../../RFM69/RFM69.h"
#include "../../RFM69/RFM69registers.h"
#include "RFM69gpio.h"
}

static int gpio_RFM69 = 0;
static int spidev_RFM69 = 0;

RFM69 radio;

static unsigned short gpioRFM69HwInit(void) {
	if(wiringXSetup() == -1) {
		return EXIT_FAILURE;
	}
	if(gpio_RFM69 >= 0) {
		if(wiringXValidGPIO(gpio_RFM69) != 0) {
			logprintf(LOG_ERR, "invalid receiver pin: %d", gpio_RFM69);
			return EXIT_FAILURE;
		}
		if(wiringXISR(gpio_RFM69, INT_EDGE_BOTH) < 0) {
			logprintf(LOG_ERR, "unable to register interrupt for pin %d", gpio_RFM69);
			return EXIT_FAILURE;
		}
	}
	// RFM69
	if(!radio.initialize(RF69_433MHZ, spidev_RFM69))
	  {
	  logprintf(LOG_ERR, "error: could not initialize SPI\n");
	  return EXIT_FAILURE;
	    }
	else
	   logprintf(LOG_DEBUG, "SPI init OK\n");
	radio.writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUS | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00);
	radio.setFrequency(433680000);
	radio.writeReg(REG_BITRATEMSB, 0x16);
	radio.writeReg(REG_BITRATELSB, 0xDA);
	radio.writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0);
	radio.writeReg(REG_SYNCCONFIG, RF_SYNC_OFF);
	radio.writeReg(REG_RSSITHRESH, 220);
	radio.writeReg(REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHDEC_000);
	radio.writeReg(REG_OOKFIX,6);
	radio.writeReg(REG_AFCFEI, 0);

	return EXIT_SUCCESS;
}

static unsigned short gpioRFM69HwDeinit(void) {
	radio.setMode(RF69_MODE_SLEEP);
	return EXIT_SUCCESS;
}

static int gpioRFM69Send(int *code, int rawlen, int repeats) {
	int r = 0, x = 0;
	radio.setMode(RF69_MODE_TX);
	pinMode(gpio_RFM69, OUTPUT);
	if(gpio_RFM69 >= 0) {
		for(r=0;r<repeats;r++) {
			for(x=0;x<rawlen;x+=2) {
				digitalWrite(gpio_RFM69, 1);
				usleep((__useconds_t)code[x]);
				digitalWrite(gpio_RFM69, 0);
				if(x+1 < rawlen) {
					usleep((__useconds_t)code[x+1]);
				}
			}
		}
		digitalWrite(gpio_RFM69, 0);
	} else {
		sleep(1);
	}
	return EXIT_SUCCESS;
}

static int gpioRFM69Receive(void) {
	radio.setMode(RF69_MODE_RX);
	pinMode(gpio_RFM69, SYS);
	if(gpio_RFM69 >= 0) {
		return irq_read(gpio_RFM69);
	} else {
		sleep(1);
		return 0;
	}
}

static unsigned short gpioRFM69Settings(JsonNode *json) {
	if(strcmp(json->key, "gpio") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_RFM69 = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	}
	if(strcmp(json->key, "spidev") == 0) {
		if(json->tag == JSON_NUMBER) {
			spidev_RFM69 = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void gpioRFM69Init(void) {
	hardware_register(&gpioRFM69);
	hardware_set_id(gpioRFM69, "RFM69gpio");

	options_add(&gpioRFM69->options, 'g', "gpio", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");
	options_add(&gpioRFM69->options, 's', "spidev", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");

	gpioRFM69->hwtype=RF433;
	gpioRFM69->comtype=COMOOK;
	gpioRFM69->init=&gpioRFM69HwInit;
	gpioRFM69->deinit=&gpioRFM69HwDeinit;
	gpioRFM69->send=&gpioRFM69Send;
	gpioRFM69->receiveOOK=&gpioRFM69Receive;
	gpioRFM69->settings=&gpioRFM69Settings;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "RFM69gpio";
	module->version = "0.1";
	module->reqversion = "5.0";
	module->reqcommit = "86";
}

void init(void) {
	gpioRFM69Init();
}
#endif

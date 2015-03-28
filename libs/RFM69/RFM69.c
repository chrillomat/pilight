// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Based on work by Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************

#include "RFM69.h"
#include "RFM69registers.h"
#include <stdio.h>
#include <errno.h>
#include "../wiringx/wiringX.h"

bool rfm69Initialize(uint8_t freqBand, uint8_t spidev)
{
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, 0xDA }, // NETWORK ID
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };

  // TODO: outside?
  wiringXSetup();
  if(wiringXSPISetup(spidev,1000000) < 0){
    return false;
  }

  do rfm69WriteReg(REG_SYNCVALUE1, 0xAA); while (rfm69ReadReg(REG_SYNCVALUE1) != 0xAA);
  do rfm69WriteReg(REG_SYNCVALUE1, 0x55); while (rfm69ReadReg(REG_SYNCVALUE1) != 0x55);

  uint8_t i;
  for (i = 0; CONFIG[i][0] != 255; i++)
    rfm69WriteReg(CONFIG[i][0], CONFIG[i][1]);

  rfm69SetMode(RF69_MODE_STANDBY);
  while ((rfm69ReadReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  return true;
}

// return the frequency (in Hz)
uint32_t rfm69GetFrequency()
{
  return RF69_FSTEP * (((uint32_t) rfm69ReadReg(REG_FRFMSB) << 16) + ((uint16_t) rfm69ReadReg(REG_FRFMID) << 8) + rfm69ReadReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void rfm69SetFrequency(uint32_t freqHz)
{
  uint8_t oldMode = _mode;
  if (oldMode == RF69_MODE_TX) {
    rfm69SetMode(RF69_MODE_RX);
  }
  freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
  rfm69WriteReg(REG_FRFMSB, freqHz >> 16);
  rfm69WriteReg(REG_FRFMID, freqHz >> 8);
  rfm69WriteReg(REG_FRFLSB, freqHz);
  if (oldMode == RF69_MODE_RX) {
    rfm69SetMode(RF69_MODE_SYNTH);
  }
  rfm69SetMode(oldMode);
}

void rfm69SetMode(uint8_t newMode)
{
  if (newMode == _mode)
    return;

  switch (newMode) {
    case RF69_MODE_TX:
      rfm69WriteReg(REG_OPMODE, (rfm69ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      break;
    case RF69_MODE_RX:
      rfm69WriteReg(REG_OPMODE, (rfm69ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      break;
    case RF69_MODE_SYNTH:
      rfm69WriteReg(REG_OPMODE, (rfm69ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      rfm69WriteReg(REG_OPMODE, (rfm69ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      rfm69WriteReg(REG_OPMODE, (rfm69ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;
  }

  while (_mode == RF69_MODE_SLEEP && (rfm69ReadReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  _mode = newMode;
}

void rfm69Sleep() {
  rfm69SetMode(RF69_MODE_SLEEP);
}

// set output power: 0 = min, 31 = max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
void rfm69SetPowerLevel(uint8_t powerLevel)
{
  rfm69WriteReg(REG_PALEVEL, (rfm69ReadReg(REG_PALEVEL) & 0xE0) | (powerLevel > 31 ? 31 : powerLevel));
}

int16_t rfm69ReadRSSI(bool forceTrigger) {
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    rfm69WriteReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((rfm69ReadReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -rfm69ReadReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t rfm69ReadReg(uint8_t addr)
{
  uint8_t buf[2];
  int status;
  buf[0]=addr & 0x7F;
  buf[1]=0x00;
  status=wiringXSPIDataRW(0, buf, 2);
  if(status<0)
    {
      wiringXLog(LOG_ERR, "rfm69ReadReg error");//: %s\n",strerror(errno));
    }
  return buf[1];
}

void rfm69WriteReg(uint8_t addr, uint8_t value)
{
  uint8_t buf[2];
  int status;
  buf[0]=addr | 0x80;
  buf[1]=value;
  status=wiringXSPIDataRW(0, buf, 2);
  if(status<0)
    {
      wiringXLog(LOG_ERR, "rfm69WriteReg error");//: %s\n",strerror(errno));
    }
}

void rfm69SetHighPower(bool onOff) {
  rfm69WriteReg(REG_OCP, onOff ? RF_OCP_OFF : RF_OCP_ON);
  if (onOff) // turning ON
    rfm69WriteReg(REG_PALEVEL, (rfm69ReadReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    rfm69WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | (rfm69ReadReg(REG_PALEVEL) & 0x1F)); // enable P0 only
}

void rfm69SetHighPowerRegs(bool onOff) {
  rfm69WriteReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  rfm69WriteReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

// for debugging
void rfm69ReadAllRegs()
{
  uint8_t regVal;
  uint8_t regAddr;
  
  for (regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    regVal = rfm69ReadReg(regAddr);

    printf("reg 0x%x: 0x%x\n",regAddr, regVal);
  }
}

uint8_t rfm69ReadTemperature(uint8_t calFactor) // returns centigrade
{
  rfm69SetMode(RF69_MODE_STANDBY);
  rfm69WriteReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((rfm69ReadReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~rfm69ReadReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void rfm69RcCalibration()
{
  rfm69WriteReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((rfm69ReadReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Based on work by Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************
#ifndef RFM69_h
#define RFM69_h

#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
// TODO
#define RF69_IRQ_PIN 0
#define RF69_IRQ_NUM 0

#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

#include <stdint.h>
#include <stdbool.h>

static volatile uint8_t _mode; // should be protected?

bool rfm69Initialize(uint8_t freqBand, uint8_t spidev);
uint32_t rfm69GetFrequency();
void rfm69SetFrequency(uint32_t freqHz);
int16_t rfm69ReadRSSI(bool forceTrigger);
void rfm69SetHighPower(bool onOFF); // has to be called after initialize() for RFM69HW
void rfm69SetPowerLevel(uint8_t level); // reduce/increase transmit power level
void rfm69Sleep();
uint8_t rfm69ReadTemperature(uint8_t calFactor); // get CMOS temperature (8bit)
void rfmRcCalibration(); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

uint8_t rfm69ReadReg(uint8_t addr);
void rfm69WriteReg(uint8_t addr, uint8_t val);
void rfm69ReadAllRegs();
void rfm69SetMode(uint8_t mode);
void rfm69SetHighPowerRegs(bool onOff);

#endif

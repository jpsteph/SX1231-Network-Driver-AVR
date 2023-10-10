

#include <avr/io.h>

#define __AVR_ATMEGA32u4__

#ifndef RFM69_h
#define RFM69_h

// Definitions

#   define SS_DDR                DDRB
#   define SS_PORT              PORTB
#   define SS_PIN                   6

#   define INT_DDR               DDRD
#   define INT_PORT             PORTD
#   define INT_PIN               PIND
#   define INT_pin_num              0
#   define INT_PIN_n              PD0
#   define INTn                  INT0
#   define ISCn0                ISC00
#   define ISCn1                ISC01
#   define INT_VECT         INT0_vect

#   define EICRn               EICRA


#define RF69_MAX_DATA_LEN       61  // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define CSMA_LIMIT             -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0   // XTAL OFF
#define RF69_MODE_STANDBY       1   // XTAL ON
#define RF69_MODE_SYNTH         2   // PLL ON
#define RF69_MODE_RX            3   // RX MODE
#define RF69_MODE_TX            4   // TX MODE
#define RF69_MODE_LISTEN		5  // Listen Mode
#define null                  0
#define COURSE_TEMP_COEF    -90     // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR   0
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP    61.03515625   // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet) / FXOSC = module crystal oscillator frequency 
// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

// Global Variables
volatile uint8_t DATA[RF69_MAX_DATA_LEN+1];  // RX/TX payload buffer, including end of string NULL char
volatile uint8_t DATALEN;

// Function Declerations
void rfm69_init(uint16_t freqBand, uint8_t nodeID, uint8_t networkID);
void setAddress(uint8_t addr);
void setNetwork(uint8_t networkID);
uint8_t canSend();

void receiveBegin();
void receiveEvent(char * rxbuff);
uint8_t receiveDone();
uint32_t getFrequency();
void setFrequency(uint32_t freqHz);
void encrypt(const char* key);
int16_t readRSSI(uint8_t forceTrigger);
void setHighPower(uint8_t onOFF);           // has to be called after initialize() for RFM69HW
void setPowerLevel(uint8_t level);            // reduce/increase transmit power level
void sleep(uint16_t eight_second_periods);
uint8_t readTemperature(uint8_t calFactor); // get CMOS temperature (8bit)
void sleep_delay(unsigned long delay);
void rcCalibration();                        // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]
uint8_t readReg(uint8_t addr);
void writeReg(uint8_t addr, uint8_t val);
void sendFrame(uint8_t toAddress, char * buffer);
void setMode(uint8_t mode);
void setHighPowerRegs(uint8_t onOff);
void maybeInterrupts();
void select();
void unselect();
uint8_t receiveDone();

#endif


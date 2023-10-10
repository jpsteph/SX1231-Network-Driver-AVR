
#include <avr/io.h>
#include <avr/interrupt.h>
#include "spi.h"
#include "RFM69registers.h"
#include "RFM69.h"
#include <util/delay.h>
#include "LED.h"
#include "uart.h"
#include <stdio.h>
#include "Timer1.h"

volatile uint8_t SENDERID;
volatile uint8_t TARGETID;                 // should match _address
volatile uint8_t PAYLOADLEN;
volatile uint8_t ACK_REQUESTED;
volatile uint8_t ACK_RECEIVED;             // should be polled immediately after sending a packet with ACK request
volatile int16_t RSSI;                     // most accurate RSSI during reception (closest to the reception)
volatile uint8_t mode = RF69_MODE_STANDBY; // should be protected?
volatile uint8_t inISR = 0; 
uint8_t isRFM69HW = 1;                     // if RFM69HW model matches high power enable possible
uint8_t address;                           //nodeID
uint8_t powerLevel = 31;
uint8_t promiscuousMode = 0;
unsigned long millis_current;
char rssibuff[30];

#define LED 7


void rfm69_init(uint16_t freqBand, uint8_t nodeID, uint8_t networkID)
{
    spi_init();                // spi initialize
    SS_DDR |= 1<<SS_PIN;       // setting SS as output
    SS_PORT |= 1<<SS_PIN;      // setting slave select high

    //register configs, see datasheet for more info!
    writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
    writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00);
	writeReg(REG_BITRATEMSB, RF_BITRATEMSB_55555);
    writeReg(REG_BITRATELSB, RF_BITRATELSB_55555);
    writeReg(REG_FDEVMSB, RF_FDEVMSB_50000);
    writeReg(REG_FDEVLSB, RF_FDEVLSB_50000);
    writeReg(REG_FRFMSB, RF_FRFMSB_433);
    writeReg(REG_FRFMID, RF_FRFMID_433);
    writeReg(REG_FRFLSB, RF_FRFLSB_433);
    writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2);
    writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
    writeReg(REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF);
    writeReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
    writeReg(REG_RSSITHRESH, 220);
    writeReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
    writeReg(REG_PAYLOADLENGTH, 66);
    writeReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
    writeReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
    writeReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);

    while (readReg(REG_SYNCVALUE1) != 0xaa)
    {
        writeReg(REG_SYNCVALUE1, 0xaa);
    }

    while (readReg(REG_SYNCVALUE1) != 0x55)
    {
        writeReg(REG_SYNCVALUE1, 0x55);
    }

 
    // Encryption is persistent between resets and can trip you up during debugging.
    // Disable it during initialization so we always start from a known state.
    encrypt(0);

    setHighPower(isRFM69HW);        // called regardless if it's a RFM69W or RFM69HW
    setMode(RF69_MODE_STANDBY);
    while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);
    
    EICRn |= (1<<ISCn1)|(1<<ISCn0); // setting INTn rising. details datasheet p91. must change with interrupt pin.
    EIMSK |= 1<<INTn;               // enable INTn
    inISR = 0;
    sei();     
	Timer1_init();  //timer for sleep mode
	Timer1_disable();           
    
    address = nodeID;
    setAddress(address);            // setting this node id
    setNetwork(networkID);
}

// set this node's address
void setAddress(uint8_t addr)
{
    writeReg(REG_NODEADRS, addr);
}

// set network address
void setNetwork(uint8_t networkID)
{
    writeReg(REG_SYNCVALUE2, networkID);
}

uint8_t canSend()
{
    if (mode == RF69_MODE_RX && PAYLOADLEN == 0 && readRSSI(0) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
    {
        setMode(RF69_MODE_STANDBY);
        return 1;
    }
    return 0;
}


void setPowerLevel(uint8_t powerLevel)
{
    uint8_t _powerLevel = powerLevel;
    writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
}


void sleep(uint16_t eight_second_periods) 
{
    setMode(RF69_MODE_SLEEP);
	Timer1_Handle(eight_second_periods);
	setMode(RF69_MODE_RX);
}

uint8_t readTemperature(uint8_t calFactor) // returns centigrade
{
    setMode(RF69_MODE_STANDBY);
    writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
    while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
    return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

// return the frequency (in Hz)
uint32_t getFrequency()
{
    return RF69_FSTEP * (((uint32_t) readReg(REG_FRFMSB) << 16) + ((uint16_t) readReg(REG_FRFMID) << 8) + readReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void setFrequency(uint32_t freqHz)
{
    uint8_t oldMode = mode;
    if (oldMode == RF69_MODE_TX) {
        setMode(RF69_MODE_RX);
    }
    freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
    writeReg(REG_FRFMSB, freqHz >> 16);
    writeReg(REG_FRFMID, freqHz >> 8);
    writeReg(REG_FRFLSB, freqHz);
    if (oldMode == RF69_MODE_RX) {
        setMode(RF69_MODE_SYNTH);
    }
    setMode(oldMode);
}

// Read byte from register
uint8_t readReg(uint8_t addr)
{
    select();
    spi_fast_shift(addr & 0x7F);
    uint8_t regval = spi_fast_shift(0);
    unselect();
    return regval;
}

// Write byte to register
void writeReg(uint8_t addr, uint8_t value)
{
    select();
    spi_fast_shift(addr | 0x80);
    spi_fast_shift(value);
    unselect();
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP"); 16 chars
// To disable encryption: encrypt(null) or encrypt(0)
void encrypt(const char* key) 
{
    setMode(RF69_MODE_STANDBY);
    if (key != 0)
    {
        select();
        spi_fast_shift(REG_AESKEY1 | 0x80);
        for (uint8_t i = 0; i < 16; i++)
            spi_fast_shift(key[i]);
        unselect();
    }
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1:0));
}

void setMode(uint8_t newMode)
{
    if (newMode == mode)
    return;

	if(mode == RF69_MODE_LISTEN) //need to set ListenOn=0, ListenAbort=0 before picking new mode in Listen mode
		writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0x9f));  //0b10011111

    switch (newMode)
    {
        case RF69_MODE_TX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
            if (isRFM69HW) setHighPowerRegs(1);
            break;
        case RF69_MODE_RX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
            if (isRFM69HW) setHighPowerRegs(0);
            break;
        case RF69_MODE_SYNTH:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        case RF69_MODE_STANDBY:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        case RF69_MODE_SLEEP:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
		case RF69_MODE_LISTEN:
			writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
			if (isRFM69HW) setHighPowerRegs(0);
			while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);
			writeReg(REG_OPMODE, (readReg(REG_OPMODE) | 0x40));
			break;	
        default:
        return;
    }
    // we are using packet mode, so this check is not really needed
    // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
    while (mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
    mode = newMode;
}
    
// internal function
void setHighPowerRegs(uint8_t onOff)
{
    if(onOff==1)
    {
    writeReg(REG_TESTPA1, 0x5D);
    writeReg(REG_TESTPA2, 0x7C);
    }
    else
    {
        writeReg(REG_TESTPA1, 0x55);
        writeReg(REG_TESTPA2, 0x70);
    }
}
    
// for RFM69HW only: you must call setHighPower(1) after rfm69_init() or else transmission won't work
void setHighPower(uint8_t onOff) 
{
    isRFM69HW = onOff;
    writeReg(REG_OCP, isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);

    if (isRFM69HW == 1) // turning ON
        writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
    else
        writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | powerLevel); // enable P0 only
}

// get the received signal strength indicator (RSSI)
int16_t readRSSI(uint8_t forceTrigger)
{
    int16_t rssi = 0;
    if (forceTrigger==1)
    {
        // RSSI trigger not needed if DAGC is in continuous mode
        writeReg(REG_RSSICONFIG, RF_RSSI_START);
        while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
    }
    rssi = -readReg(REG_RSSIVALUE);
    rssi >>= 1;
    return rssi;
}


// function for transmitting
void sendFrame(uint8_t toAddress, char * buffer)
{
	uint8_t bufferSize = 0;
	while(*buffer){
		bufferSize++;
		buffer++;
	}
	
	for(uint8_t i=0; i < bufferSize; i++)
		buffer--;
	

	writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
	
	cli();
	_delay_ms(1000);
	sei();
	
    setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
    while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
    //writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
    if (bufferSize > RF69_MAX_DATA_LEN)
        bufferSize = RF69_MAX_DATA_LEN;

    select(); 
    spi_fast_shift(REG_FIFO | 0x80);
    spi_fast_shift(bufferSize + 3);
    spi_fast_shift(toAddress);
    spi_fast_shift(address);

    for (uint8_t i = 0; i < bufferSize; i++)
        spi_fast_shift(((uint8_t*) buffer)[i]);
    
    unselect();

    // no need to wait for transmit mode to be ready since its handled by the radio
    setMode(RF69_MODE_TX);
    
	while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00); // wait for PacketSent
	setMode(RF69_MODE_RX);
	while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_RXREADY ) == 0x00);
}


void sleep_delay(unsigned long delay) 
{
	
	setMode(RF69_MODE_SLEEP);
	
}

// Calibrate RC
void rcCalibration()
{
    writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
    while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}


// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
uint8_t receiveDone()
{
    cli();

    if (mode == RF69_MODE_RX && PAYLOADLEN > 0)
    {
        setMode(RF69_MODE_STANDBY); // enables interrupts
        return 1;
    }
    else if (mode == RF69_MODE_RX) // already in RX no payload yet
    {
        sei(); // explicitly re-enable interrupts
        return 0;
    }
    receiveBegin();
    sei();
    return 0;
}

void receiveEvent(char * rxbuff)
{
	if(receiveDone())
	{
		_delay_ms(10);
		
		for(uint8_t i=0;i<DATALEN;i++)
		{
			*rxbuff = DATA[i];
			rxbuff++;
		}
	}
}

// internal function
void receiveBegin()
{
    DATALEN = 0;
    SENDERID = 0;
    TARGETID = 0;
    PAYLOADLEN = 0;
    ACK_REQUESTED = 0;
    ACK_RECEIVED = 0;
    RSSI = 0;
    if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
    setMode(RF69_MODE_RX);
}


// Only reenable interrupts if we're not being called from the ISR
void maybeInterrupts()
{
    if (!inISR) sei();
}

// Enable SPI transfer
void select()
{
    SS_PORT &= ~(1<<SS_PIN);
    cli();
}

// Disable SPI transfer
void unselect()
{
    SS_PORT |= 1<<SS_PIN;
    maybeInterrupts();
}

// Interrupt Service Routine
ISR(INT_VECT)
{
    inISR = 1;
    if (mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
    {
		LEDBLINKSHORT();
		
        setMode(RF69_MODE_STANDBY);
        select();
        spi_fast_shift(REG_FIFO & 0x7F);
        PAYLOADLEN = spi_fast_shift(0);
        if(PAYLOADLEN>66) PAYLOADLEN=66;
        TARGETID = spi_fast_shift(0);
        if(!(TARGETID == address || TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in promiscuous mode
        || PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
        {
            PAYLOADLEN = 0;
            unselect();
            receiveBegin();
            return;
        }

        DATALEN = PAYLOADLEN - 3;
        SENDERID = spi_fast_shift(0);
        
        for (uint8_t i = 0; i < DATALEN; i++)
        {
            DATA[i] = spi_fast_shift(0);
        }
        if (DATALEN < RF69_MAX_DATA_LEN) DATA[DATALEN] = 0; // add null at end of string
        unselect();
		
        setMode(RF69_MODE_RX);
		
    }
    //RSSI = readRSSI(0);
	sprintf(rssibuff, "RSSI of Signal: %d dBm", readRSSI(0));
	USART_Transmit_String(rssibuff);
	
    inISR = 0;
}

#include <ASK.h>

// Interrupt handler uses this to find the most recently initialised instance of this driver
static ASK* thisASKDriver;

// 4 bit to 6 bit symbol converter table
// Used to convert the high and low nybbles of the transmitted data
// into 6 bit symbols for transmission. Each 6-bit symbol has 3 1s and 3 0s 
// with at most 3 consecutive identical bits
static uint8_t symbols[] =
{
    0xd,  0xe,  0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 
    0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};


// This is the value of the start symbol after 6-bit conversion and nybble swapping
#define RH_ASK_START_SYMBOL 0xb38

RH_ASK::RH_ASK(uint16_t speed, uint8_t rxPin, uint8_t txPin, uint8_t pttPin, bool pttInverted)
    :
    _speed(speed),
    _rxPin(rxPin),
    _txPin(txPin),
    _pttPin(pttPin),
    _rxInverted(false),
    _pttInverted(pttInverted)
{
    // Initialise the first 8 nibbles of the tx buffer to be the standard
    // preamble. We will append messages after that. 0x38, 0x2c is the start symbol before
    // 6-bit conversion to RH_ASK_START_SYMBOL
    uint8_t preamble[RH_ASK_PREAMBLE_LEN] = {0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c};
    memcpy(_txBuf, preamble, sizeof(preamble));
}


bool RH_ASK::init()
{
	
	if (!Driver::init())
	return false;
    thisASKDriver = this;
	
	 // Set up digital IO pins for arduino
    pinMode(_txPin, OUTPUT);
    pinMode(_rxPin, INPUT);
    pinMode(_pttPin, OUTPUT);
	
	
	setModeIdle();
    timerSetup();
	
	return true;
}

// Common function for setting timer ticks @ prescaler values for speed
// Returns prescaler index into {0, 1, 8, 64, 256, 1024} array
// and sets nticks to compare-match value if lower than max_ticks
// returns 0 & nticks = 0 on fault

uint8_t RH_ASK::timerCalc(uint16_t speed, uint16_t max_ticks, uint16_t *nticks)
{
	// Clock divider (prescaler) values - 0/3333: error flag
    uint8_t prescaler;     // index into array & return bit value
    unsigned long ulticks; // calculate by ntick overflow

    // Div-by-zero protection
    if (speed == 0)
    {
        // signal fault
        *nticks = 0;
        return 0;
    }

    // test increasing prescaler (divisor), decreasing ulticks until no overflow
    // 1/Fraction of second needed to xmit one bit
    unsigned long inv_bit_time = ((unsigned long)speed) * 8;
    for (prescaler=1; prescaler < NUM_PRESCALERS; prescaler += 1)
    {
	// Integer arithmetic courtesy Jim Remington
	// 1/Amount of time per CPU clock tick (in seconds)
	uint16_t prescalerValue;
	memcpy_P(&prescalerValue, &prescalers[prescaler], sizeof(uint16_t));
        unsigned long inv_clock_time = F_CPU / ((unsigned long)prescalerValue);
        // number of prescaled ticks needed to handle bit time @ speed
        ulticks = inv_clock_time / inv_bit_time;

        // Test if ulticks fits in nticks bitwidth (with 1-tick safety margin)
        if ((ulticks > 1) && (ulticks < max_ticks))
            break; // found prescaler

        // Won't fit, check with next prescaler value
    }


    // Check for error
    if ((prescaler == 6) || (ulticks < 2) || (ulticks > max_ticks))
    {
        // signal fault
        *nticks = 0;
        return 0;
    }

    *nticks = ulticks;
    return prescaler;
}

// The idea here is to get 8 timer interrupts per bit period
void RH_ASK::timerSetup()
{
	 uint16_t nticks; // number of prescaled ticks needed
    uint8_t prescaler; // Bit values for CS0[2:0]

	
	 // Use timer 1
    prescaler = timerCalc(_speed, (uint16_t)-1, &nticks);    
    if (!prescaler)
        return; // fault
    TCCR1A = 0; // Output Compare pins disconnected
    TCCR1B = _BV(WGM12); // Turn on CTC mode

    // convert prescaler index to TCCRnB prescaler bits CS10, CS11, CS12
    TCCR1B |= prescaler;

    // Caution: special procedures for setting 16 bit regs
    // is handled by the compiler
    OCR1A = nticks;
    // Enable interrupt
   #ifdef TIMSK1
    // atmega168
    TIMSK1 |= _BV(OCIE1A);
   #else
    // others
    TIMSK |= _BV(OCIE1A);
	
	
}

void INTERRUPT_ATTR RH_ASK::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	// Disable the transmitter hardware
	writePtt(LOW);
	writeTx(LOW);
	_mode = RHModeIdle;
    }
}

void RH_ASK::setModeRx()
{
    if (_mode != RHModeRx)
    {
	// Disable the transmitter hardware
	writePtt(LOW);
	writeTx(LOW);
	_mode = RHModeRx;
    }
}


void RH_ASK::setModeTx()
{
    if (_mode != RHModeTx)
    {
	// PRepare state varibles for a new transmission
	_txIndex = 0;
	_txBit = 0;
	_txSample = 0;

	// Enable the transmitter hardware
	writePtt(HIGH);

	_mode = RHModeTx;
    }
}

// Call this often
bool RH_ASK::available()
{
    if (_mode == RHModeTx)
	return false;
    setModeRx();
    if (_rxBufFull)
    {
	validateRxBuf();
	_rxBufFull= false;
    }
    return _rxBufValid;
}

bool RH_ASK::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	// Skip the length and 4 headers that are at the beginning of the rxBuf
	// and drop the trailing 2 bytes of FCS
	uint8_t message_len = _rxBufLen-RH_ASK_HEADER_LEN - 3;
	if (*len > message_len)
	    *len = message_len;
	memcpy(buf, _rxBuf+RH_ASK_HEADER_LEN+1, *len);
    }
    _rxBufValid = false; // Got the most recent message, delete it
//    printBuffer("recv:", buf, *len);
    return true;
}

// Caution: this may block
bool RH_ASK::send(const uint8_t* data, uint8_t len)
{
    uint8_t i;
    uint16_t index = 0;
    uint16_t crc = 0xffff;
    uint8_t *p = _txBuf + RH_ASK_PREAMBLE_LEN; // start of the message area
    uint8_t count = len + 3 + RH_ASK_HEADER_LEN; // Added byte count and FCS and headers to get total number of bytes

    if (len > RH_ASK_MAX_MESSAGE_LEN)
	return false;

    // Wait for transmitter to become available
    waitPacketSent();

    if (!waitCAD()) 
	return false;  // Check channel activity

    // Encode the message length
    crc = RHcrc_ccitt_update(crc, count);
    p[index++] = symbols[count >> 4];
    p[index++] = symbols[count & 0xf];

    // Encode the headers
    crc = RHcrc_ccitt_update(crc, _txHeaderTo);
    p[index++] = symbols[_txHeaderTo >> 4];
    p[index++] = symbols[_txHeaderTo & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFrom);
    p[index++] = symbols[_txHeaderFrom >> 4];
    p[index++] = symbols[_txHeaderFrom & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderId);
    p[index++] = symbols[_txHeaderId >> 4];
    p[index++] = symbols[_txHeaderId & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFlags);
    p[index++] = symbols[_txHeaderFlags >> 4];
    p[index++] = symbols[_txHeaderFlags & 0xf];

    // Encode the message into 6 bit symbols. Each byte is converted into 
    // 2 6-bit symbols, high nybble first, low nybble second
    for (i = 0; i < len; i++)
    {
	crc = RHcrc_ccitt_update(crc, data[i]);
	p[index++] = symbols[data[i] >> 4];
	p[index++] = symbols[data[i] & 0xf];
    }

    // Append the fcs, 16 bits before encoding (4 6-bit symbols after encoding)
    // Caution: VW expects the _ones_complement_ of the CCITT CRC-16 as the FCS
    // VW sends FCS as low byte then hi byte
    crc = ~crc;
    p[index++] = symbols[(crc >> 4)  & 0xf];
    p[index++] = symbols[crc & 0xf];
    p[index++] = symbols[(crc >> 12) & 0xf];
    p[index++] = symbols[(crc >> 8)  & 0xf];

    // Total number of 6-bit symbols to send
    _txBufLen = index + RH_ASK_PREAMBLE_LEN;

    // Start the low level interrupt handler sending symbols
    setModeTx();

    return true;
}





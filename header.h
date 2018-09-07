#ifndef RH_ASK_h
#define RH_ASK_h

#include <Driver.h>

// Maximum message length (including the headers, byte count and FCS) we are willing to support
// This is pretty arbitrary
#define RH_ASK_MAX_PAYLOAD_LEN 67

// The length of the headers we add (To, From, Id, Flags)
// The headers are inside the payload and are therefore protected by the FCS
#define RH_ASK_HEADER_LEN 4

// This is the maximum message length that can be supported by this library. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RH_ASK_MAX_MESSAGE_LEN
 #define RH_ASK_MAX_MESSAGE_LEN (RH_ASK_MAX_PAYLOAD_LEN - RH_ASK_HEADER_LEN - 3)
#endif

#if !defined(RH_ASK_RX_SAMPLES_PER_BIT)
/// Number of samples per bit
 #define RH_ASK_RX_SAMPLES_PER_BIT 8
#endif //RH_ASK_RX_SAMPLES_PER_BIT  

/// The size of the receiver ramp. Ramp wraps modulo this number
#define RH_ASK_RX_RAMP_LEN 160

// Ramp adjustment parameters
// Standard is if a transition occurs before RH_ASK_RAMP_TRANSITION (80) in the ramp,
// the ramp is retarded by adding RH_ASK_RAMP_INC_RETARD (11)
// else by adding RH_ASK_RAMP_INC_ADVANCE (29)
// If there is no transition it is adjusted by RH_ASK_RAMP_INC (20)
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_INC (RH_ASK_RX_RAMP_LEN/RH_ASK_RX_SAMPLES_PER_BIT)
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_TRANSITION RH_ASK_RX_RAMP_LEN/2
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_ADJUST 9
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_INC_RETARD (RH_ASK_RAMP_INC-RH_ASK_RAMP_ADJUST)
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_INC_ADVANCE (RH_ASK_RAMP_INC+RH_ASK_RAMP_ADJUST)

/// Outgoing message bits grouped as 6-bit words
/// 36 alternating 1/0 bits, followed by 12 bits of start symbol (together called the preamble)
/// Followed immediately by the 4-6 bit encoded byte count, 
/// message buffer and 2 byte FCS
/// Each byte from the byte count on is translated into 2x6-bit words
/// Caution, each symbol is transmitted LSBit first, 
/// but each byte is transmitted high nybble first
/// This is the number of 6 bit nibbles in the preamble
#define RH_ASK_PREAMBLE_LEN 8


class RH_ASK : public Driver
{
public:
    /// Constructor.
    /// At present only one instance of RH_ASK per sketch is supported.
    /// \param[in] speed The desired bit rate in bits per second
    /// \param[in] rxPin The pin that is used to get data from the receiver
    /// \param[in] txPin The pin that is used to send data to the transmitter
    /// \param[in] pttPin The pin that is connected to the transmitter controller. It will be set HIGH to enable the transmitter (unless pttInverted is true).
    /// \param[in] pttInverted true if you desire the pttin to be inverted so that LOW wil enable the transmitter.
    RH_ASK(uint16_t speed = 2000, uint8_t rxPin = 11, uint8_t txPin = 12, uint8_t pttPin = 10, bool pttInverted = false);

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool    init();

    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received bythe transport, when it wil be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool    available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool    recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
    virtual bool    send(const uint8_t* data, uint8_t len);

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF69.
    void           setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF69.
    void           setModeTx();

    /// dont call this it used by the interrupt handler
    void            handleTimerInterrupt();

    /// Returns the current speed in bits per second
    /// \return The current speed in bits per second
    uint16_t        speed() { return _speed;}

#if (RH_PLATFORM == RH_PLATFORM_ESP8266)
    /// ESP8266 timer0 increment value
    uint32_t _timerIncrement;
#endif

protected:
    /// Helper function for calculating timer ticks
    uint8_t         timerCalc(uint16_t speed, uint16_t max_ticks, uint16_t *nticks);

    /// Set up the timer and its interrutps so the interrupt handler is called at the right frequency
    void            timerSetup();

    /// Read the rxPin in a platform dependent way, taking into account whether it is inverted or not
    bool            readRx();

    /// Write the txPin in a platform dependent way
    void            writeTx(bool value);

    /// Write the txPin in a platform dependent way, taking into account whether it is inverted or not
    void            writePtt(bool value);

    /// Translates a 6 bit symbol to its 4 bit plaintext equivalent
    uint8_t         symbol_6to4(uint8_t symbol);

    /// The receiver handler function, called a 8 times the bit rate
    void            receiveTimer();

    /// The transmitter handler function, called a 8 times the bit rate 
    void            transmitTimer();

    /// Check whether the latest received message is complete and uncorrupted
    /// We should always check the FCS at user level, not interrupt level
    /// since it is slow
    void            validateRxBuf();

    /// Configure bit rate in bits per second
    uint16_t        _speed;

    /// The configure receiver pin
    uint8_t         _rxPin;

    /// The configure transmitter pin
    uint8_t         _txPin;

    /// The configured transmitter enable pin
    uint8_t         _pttPin;

    /// True of the sense of the rxPin is to be inverted
    bool            _rxInverted;

    /// True of the sense of the pttPin is to be inverted
    bool            _pttInverted;

    // Used in the interrupt handlers
    /// Buf is filled but not validated
    volatile bool   _rxBufFull;

    /// Buf is full and valid
    volatile bool   _rxBufValid;

    /// Last digital input from the rx data pin
    volatile bool   _rxLastSample;

    /// This is the integrate and dump integral. If there are <5 0 samples in the PLL cycle
    /// the bit is declared a 0, else a 1
    volatile uint8_t _rxIntegrator;

    /// PLL ramp, varies between 0 and RH_ASK_RX_RAMP_LEN-1 (159) over 
    /// RH_ASK_RX_SAMPLES_PER_BIT (8) samples per nominal bit time. 
    /// When the PLL is synchronised, bit transitions happen at about the
    /// 0 mark. 
    volatile uint8_t _rxPllRamp;

    /// Flag indicates if we have seen the start symbol of a new message and are
    /// in the processes of reading and decoding it
    volatile uint8_t _rxActive;

    /// Last 12 bits received, so we can look for the start symbol
    volatile uint16_t _rxBits;

    /// How many bits of message we have received. Ranges from 0 to 12
    volatile uint8_t _rxBitCount;
    
    /// The incoming message buffer
    uint8_t _rxBuf[RH_ASK_MAX_PAYLOAD_LEN];
    
    /// The incoming message expected length
    volatile uint8_t _rxCount;
    
    /// The incoming message buffer length received so far
    volatile uint8_t _rxBufLen;

    /// Index of the next symbol to send. Ranges from 0 to vw_tx_len
    uint8_t _txIndex;

    /// Bit number of next bit to send
    uint8_t _txBit;

    /// Sample number for the transmitter. Runs 0 to 7 during one bit interval
    uint8_t _txSample;

    /// The transmitter buffer in _symbols_ not data octets
    uint8_t _txBuf[(RH_ASK_MAX_PAYLOAD_LEN * 2) + RH_ASK_PREAMBLE_LEN];

    /// Number of symbols in _txBuf to be sent;
    uint8_t _txBufLen;

};

/// @example ask_reliable_datagram_client.pde
/// @example ask_reliable_datagram_server.pde
/// @example ask_transmitter.pde
/// @example ask_receiver.pde
#endif
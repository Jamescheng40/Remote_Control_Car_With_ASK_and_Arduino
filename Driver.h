#define RHGenericDriver_h


// Defines bits of the FLAGS header reserved for use by the RadioHead library and 
// the flags available for use by applications
#define RH_FLAGS_RESERVED                 0xf0
#define RH_FLAGS_APPLICATION_SPECIFIC     0x0f
#define RH_FLAGS_NONE                     0

// Default timeout for waitCAD() in ms
#define RH_CAD_DEFAULT_TIMEOUT            10000


class RHGenericDriver
{
public:
    /// \brief Defines different operating modes for the transport hardware
    ///
    /// These are the different values that can be adopted by the _mode variable and 
    /// returned by the mode() member function,
    typedef enum
    {
	RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
	RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
	RHModeIdle,             ///< Transport is idle.
	RHModeTx,               ///< Transport is in the process of transmitting a message.
	RHModeRx,               ///< Transport is in the process of receiving a message.
	RHModeCad               ///< Transport is in the process of detecting channel activity (if supported)
    } RHMode;

    /// Constructor
    RHGenericDriver();

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool init();

    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, if there is an uncollected received message, and there is no message
    /// currently bing transmitted, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it will be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop.
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv().
    virtual bool available() = 0;

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool recv(uint8_t* buf, uint8_t* len) = 0;

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then optionally waits for Channel Activity Detection (CAD) 
    /// to show the channnel is clear (if the radio supports CAD) by calling waitCAD().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. If the message is too long for the underlying radio technology, send() will
    /// return false and will not send the message.
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// specify the maximum time in ms to wait. If 0 (the default) do not wait for CAD before transmitting.
    /// \return true if the message length was valid and it was correctly queued for transmit. Return false
    /// if CAD was requested and the CAD timeout timed out before clear channel was detected.
    virtual bool send(const uint8_t* data, uint8_t len) = 0;

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength() = 0;

    /// Blocks until the transmitter 
    /// is no longer transmitting.
    virtual bool            waitPacketSent();

    /// Blocks until the transmitter is no longer transmitting.
    /// or until the timeout occuers, whichever happens first
    /// \param[in] timeout Maximum time to wait in milliseconds.
    /// \return true if the radio completed transmission within the timeout period. False if it timed out.
    virtual bool            waitPacketSent(uint16_t timeout);


    // Bent G Christensen (bentor@gmail.com), 08/15/2016
    /// Channel Activity Detection (CAD).
    /// Blocks until channel activity is finished or CAD timeout occurs.
    /// Uses the radio's CAD function (if supported) to detect channel activity.
    /// Implements random delays of 100 to 1000ms while activity is detected and until timeout.
    /// Caution: the random() function is not seeded. If you want non-deterministic behaviour, consider
    /// using something like randomSeed(analogRead(A0)); in your sketch.
    /// Permits the implementation of listen-before-talk mechanism (Collision Avoidance).
    /// Calls the isChannelActive() member function for the radio (if supported) 
    /// to determine if the channel is active. If the radio does not support isChannelActive(),
    /// always returns true immediately
    /// \return true if the radio-specific CAD (as returned by isChannelActive())
    /// shows the channel is clear within the timeout period (or the timeout period is 0), else returns false.
    virtual bool            waitCAD();

 

    /// Determine if the currently selected radio channel is active.
    /// This is expected to be subclassed by specific radios to implement their Channel Activity Detection
    /// if supported. If the radio does not support CAD, returns true immediately. If a RadioHead radio 
    /// supports isChannelActive() it will be documented in the radio specific documentation.
    /// This is called automatically by waitCAD().
    /// \return true if the radio-specific CAD (as returned by override of isChannelActive()) shows the
    /// current radio channel as active, else false. If there is no radio-specific CAD, returns false.
    virtual bool            isChannelActive();



    /// Returns the operating mode of the library.
    /// \return the current mode, one of RF69_MODE_*
    RHMode          mode();

    /// Sets the operating mode of the transport.
    void            setMode(RHMode mode);

    /// Sets the transport hardware into low-power sleep mode
    /// (if supported). May be overridden by specific drivers to initialte sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// \return true if sleep mode is supported by transport hardware and the RadioHead driver, and if sleep mode
    ///         was successfully entered. If sleep mode is not suported, return false.
    virtual bool    sleep();

  

   

   
protected:

    /// The current transport operating mode
    volatile RHMode     _mode;

    /// This node id
    uint8_t             _thisAddress;
    
    /// Whether the transport is in promiscuous mode
    bool                _promiscuous;

    /// TO header in the last received mesasge
    volatile uint8_t    _rxHeaderTo;

    /// FROM header in the last received mesasge
    volatile uint8_t    _rxHeaderFrom;

    /// ID header in the last received mesasge
    volatile uint8_t    _rxHeaderId;

    /// FLAGS header in the last received mesasge
    volatile uint8_t    _rxHeaderFlags;

    /// TO header to send in all messages
    uint8_t             _txHeaderTo;

    /// FROM header to send in all messages
    uint8_t             _txHeaderFrom;

    /// ID header to send in all messages
    uint8_t             _txHeaderId;

    /// FLAGS header to send in all messages
    uint8_t             _txHeaderFlags;

    /// The value of the last received RSSI value, in some transport specific units
    volatile int16_t     _lastRssi;

    /// Count of the number of bad messages (eg bad checksum etc) received
    volatile uint16_t   _rxBad;

    /// Count of the number of successfully transmitted messaged
    volatile uint16_t   _rxGood;

    /// Count of the number of bad messages (correct checksum etc) received
    volatile uint16_t   _txGood;
    
    /// Channel activity detected
    volatile bool       _cad;

    /// Channel activity timeout in ms
    unsigned int        _cad_timeout;

private:

};
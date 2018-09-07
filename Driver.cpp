


#include <Driver.h>

RHGenericDriver::RHGenericDriver()
    :
    _mode(RHModeInitialising),
    _thisAddress(RH_BROADCAST_ADDRESS),
    _txHeaderTo(RH_BROADCAST_ADDRESS),
    _txHeaderFrom(RH_BROADCAST_ADDRESS),
    _txHeaderId(0),
    _txHeaderFlags(0),
    _rxBad(0),
    _rxGood(0),
    _txGood(0),
    _cad_timeout(0)
{
}
//initial RHGenericDriver fucntion 
bool RHGenericDriver::init()
{
    return true;
}

bool RHGenericDriver::waitPacketSent()
{
    while (_mode == RHModeTx)
	YIELD; // Wait for any previous transmit to finish
    return true;
}


//checking to see if previous bits are done transmitting
bool RHGenericDriver::waitPacketSent(uint16_t timeout)
{
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout)
    {
        if (_mode != RHModeTx) // Any previous transmit finished?
           return true;
	YIELD;
    }
    return false;
}

// Wait until no channel activity detected or timeout
bool RHGenericDriver::waitCAD()
{
    if (!_cad_timeout)
	return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
    unsigned long t = millis();
    while (isChannelActive())
    {
         if (millis() - t > _cad_timeout) 
	     return false;
#if (RH_PLATFORM == RH_PLATFORM_STM32) // stdlib on STMF103 gets confused if random is redefined
	 delay(_random(1, 10) * 100);
#else
         delay(random(1, 10) * 100); // Should these values be configurable? Macros?
#endif
    }

    return true;
}

// subclasses are expected to override if CAD is available for that radio
bool RHGenericDriver::isChannelActive()
{
    return false;
}






RHGenericDriver::RHMode  RHGenericDriver::mode()
{
    return _mode;
}

void  RHGenericDriver::setMode(RHMode mode)
{
    _mode = mode;
}

bool  RHGenericDriver::sleep()
{
    return false;
}



#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined(RH_PLATFORM_ATTINY)
// Tinycore does not have __cxa_pure_virtual, so without this we
// get linking complaints from the default code generated for pure virtual functions
extern "C" void __cxa_pure_virtual()
{
    while (1);
}
#endif
bool RH_ASK::init()
{
	
	 // Set up digital IO pins for arduino
    pinMode(_txPin, OUTPUT);
    pinMode(_rxPin, INPUT);
    pinMode(_pttPin, OUTPUT);
	
	
	setModeIdle();
    timerSetup();
	
	 return true;
}
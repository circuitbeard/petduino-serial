#include "PetduinoSerial.h"

// Store reference to petduino instance, there can be only one
PetduinoSerial *_petduinoInstance;

// CmdMessenger requires that callback functions need to
// be global, so create global functions that just ping
// back to the current petduino instance
void _onSetState(){       _petduinoInstance->onSetState();       }
void _onGetState(){       _petduinoInstance->onGetState();       }
void _onSetLed(){         _petduinoInstance->onSetLed();         }
void _onToggleLed(){      _petduinoInstance->onToggleLed();      }
void _onGetLed(){         _petduinoInstance->onGetLed();         }
void _onGetTemperature(){ _petduinoInstance->onGetTemperature(); }
void _onGetLighLevel(){   _petduinoInstance->onGetLightLevel();  }
void _onSetData(){        _petduinoInstance->onSetData();        }

// Constructor
PetduinoSerial::PetduinoSerial() {
}

// Setup inputs, outputs, etc. Call this from main arduino setup() method
void PetduinoSerial::begin(){
  PetduinoSerial::begin(9600);
}

void PetduinoSerial::begin(long baudRate){

  // Store reference to instance
  _petduinoInstance = this;

  // Setup Petduino
  Petduino::begin();

  // Start serial
  Serial.begin(baudRate);

  // Setup cmd messenger
  cmdMessenger.printLfCr();
  cmdMessenger.attach(SET_STATE_ACTION,       _onSetState);
  cmdMessenger.attach(GET_STATE_ACTION,       _onGetState);
  cmdMessenger.attach(SET_LED_ACTION,         _onSetLed);
  cmdMessenger.attach(TOGGLE_LED_ACTION,      _onToggleLed);
  cmdMessenger.attach(GET_LED_ACTION,         _onGetLed);
  cmdMessenger.attach(GET_TEMPERATURE_ACTION, _onGetTemperature);
  cmdMessenger.attach(GET_LIGHT_LEVEL_ACTION, _onGetLighLevel);
  cmdMessenger.attach(SET_DATA,               _onSetData);

}

// Update function to be called from main loop() method
void PetduinoSerial::update() {

  // Update Petduino
  Petduino::update();

  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();

  // Check for btn1 state change
  bool btn1Reading = isBtn1Held();
  if(btn1Reading != lastBtn1Reading) {
    lastBtn1Reading = btn1Reading;
    cmdMessenger.sendCmd(BTN1_EVENT, btn1Reading);
  }

  // Check for btn2 state change
  bool btn2Reading = isBtn2Held();
  if(btn2Reading != lastBtn2Reading) {
    lastBtn2Reading = btn2Reading;
    cmdMessenger.sendCmd(BTN2_EVENT, btn2Reading);
  }

  // Check for temperature state changed
  //TODO: Need debounce or something because individual readings can be eratic
  if(tempBroadcastThreshold > 0){
    float reading = getTemperature();
    int readingInt = (int)floor(reading + 0.5);
    if((readingInt%tempBroadcastThreshold) == 0 && abs(lastTempReading - reading) >= tempBroadcastThreshold) {
      lastTempReading = reading;
      cmdMessenger.sendCmd(TEMPERATURE_EVENT, reading);
    }
  }

  // Check for light level state changes
  if(ldrBroadcastThreshold > 0) {
    int reading = getLightLevel();
    if((reading%ldrBroadcastThreshold) == 0 && abs(lastLdrReading - reading) >= ldrBroadcastThreshold) {
      lastLdrReading = reading;
      cmdMessenger.sendCmd(LIGHT_LEVEL_EVENT, reading);
    }
  }

}

// Protected ===========================================

// Public ===========================================

// Set the value of the Petduino LED
void PetduinoSerial::setLed(bool value) {

  // Set the led
  Petduino::setLed(value);

  // Send event
  cmdMessenger.sendCmd(LED_EVENT, getLed());

}

// Toggle the Petduino LED
void PetduinoSerial::toggleLed() {

  // Toggle the led
  Petduino::toggleLed();

  // Send event
  cmdMessenger.sendCmd(LED_EVENT, getLed());

}

// Lines up the next state to move to after the given interval
void PetduinoSerial::setNextState(unsigned int state, unsigned long interval) {

  // Set next state
  Petduino::setNextState(state, interval);

  // Send event
  if(interval == 0 && state != WAIT_INTERVAL_STATE && state != WAIT_STATE) {
    cmdMessenger.sendCmd(STATE_EVENT, state);
  }

}

void PetduinoSerial::setTemperatureBroadcastThreshold(unsigned int threshold) {
  tempBroadcastThreshold = threshold;
}

void PetduinoSerial::setLightLevelBroadcastThreshold(unsigned int threshold) {
  ldrBroadcastThreshold = threshold;
}

// Set state command handler
void PetduinoSerial::onSetState()
{
  // Read state argument, interpret string as int
  int state = cmdMessenger.readInt32Arg();

  // Set state
  setState(state);
}

// Get state command handler
void PetduinoSerial::onGetState()
{
  // Send status that describes the state manually
  cmdMessenger.sendCmd(STATE_EVENT, getState());
}

// Set led command handler
void PetduinoSerial::onSetLed()
{
  // Read led state argument, interpret string as boolean
  bool ledState = cmdMessenger.readBoolArg();

  // Set led
  setLed(ledState ? HIGH : LOW);
}

// Toggle led command handler
void PetduinoSerial::onToggleLed()
{
  // Set led
  toggleLed();
}

// Get led command handler
void PetduinoSerial::onGetLed()
{
  // Send status that describes the led state manually
  cmdMessenger.sendCmd(LED_EVENT, getLed());
}

// Get temperature command handler
void PetduinoSerial::onGetTemperature()
{
  // Send status that describes the led state manually
  cmdMessenger.sendCmd(TEMPERATURE_EVENT, getTemperature());
}

// Get temperature command handler
void PetduinoSerial::onGetLightLevel()
{
  // Send status that describes the led state manually
  cmdMessenger.sendCmd(LIGHT_LEVEL_EVENT, getLightLevel());
}

// Set data command handler
void PetduinoSerial::onSetData()
{
  //TODO: Create user definable callback
}

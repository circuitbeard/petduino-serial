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
  onDataCallBack = NULL;
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
  if(tempBroadcastThreshold > 0){
    float reading = getTemperature();
    int readingInt = (int)floor(reading + 0.5);
    int readingDir = (lastTempReading - reading)/abs(lastTempReading - reading);
    if(abs(lastTempReading - reading) >= tempBroadcastThreshold) {
      if(tempReadingDir == 0 || tempReadingDir == readingDir) {
        if(tempReadingCount >= tempConsecutiveReadings){
          tempReadingCount = 0;
          tempReadingDir = 0;
          lastTempReading = reading;
          cmdMessenger.sendCmd(TEMPERATURE_EVENT, reading);
        } else  {
          tempReadingCount++;
          tempReadingDir = readingDir;
        }
      } else {
        tempReadingCount = 0;
        tempReadingDir = readingDir;
      }
    } else {
      tempReadingCount = 0;
      tempReadingDir = 0;
    }
  }

  // Check for light level state changes
  if(ldrBroadcastThreshold > 0) {
    int reading = getLightLevel();
    int readingDir = (lastLdrReading - reading)/abs(lastLdrReading - reading);
    if(abs(lastLdrReading - reading) >= ldrBroadcastThreshold) {
      if(ldrReadingDir == 0 || ldrReadingDir == readingDir) {
        if(ldrReadingCount >= ldrConsecutiveReadings) {
          ldrReadingCount = 0;
          ldrReadingDir = 0;
          lastLdrReading = reading;
          cmdMessenger.sendCmd(LIGHT_LEVEL_EVENT, reading);
        } else {
          ldrReadingCount++;
          ldrReadingDir = readingDir;
        }
      } else {
        ldrReadingCount = 0;
        ldrReadingDir = readingDir;
      }
    } else {
      ldrReadingCount = 0;
      ldrReadingDir = 0;
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

// Sets the threshold  by which a temperature reading must change, and for how
// many readings untill it should be auto broadcast
void PetduinoSerial::setTemperatureBroadcastThreshold(unsigned int threshold, unsigned int consecutiveReadings) {
  tempBroadcastThreshold = threshold;
  tempConsecutiveReadings = consecutiveReadings;
}

// Sets the threshold  by which a light level reading must change, and for how
// many readings untill it should be auto broadcast
void PetduinoSerial::setLightLevelBroadcastThreshold(unsigned int threshold, unsigned int consecutiveReadings) {
  ldrBroadcastThreshold = threshold;
  ldrConsecutiveReadings = consecutiveReadings;
}

// Sets the on data callback function
void PetduinoSerial::setOnDataCallback(onDataCallbackFunc callback) {
  onDataCallBack = callback;
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
  // Make sure we have a callback function registered
  if (onDataCallBack != NULL) {

    // Read the string param
    char *data = cmdMessenger.readStringArg();

    // Call the callback function
    (*onDataCallBack)(data);
  }
}

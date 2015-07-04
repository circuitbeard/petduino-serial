#ifndef PetduinoSerial_h
#define PetduinoSerial_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <LedControl.h>
#include <Petduino.h>
#include <CmdMessenger.h>

extern "C"
{
  typedef void(*onDataCallbackFunc) (char *data);
}

class PetduinoSerial : public Petduino {

    protected:

      // Variables
      CmdMessenger cmdMessenger = CmdMessenger(Serial);

      // Define actions
      enum {
        SET_STATE_ACTION,
        GET_STATE_ACTION,
        SET_LED_ACTION,
        TOGGLE_LED_ACTION,
        GET_LED_ACTION,
        GET_TEMPERATURE_ACTION,
        GET_LIGHT_LEVEL_ACTION,
        SET_DATA
      };

      // Define events
      enum {
        STATE_EVENT,
        LED_EVENT,
        TEMPERATURE_EVENT,
        LIGHT_LEVEL_EVENT,
        BTN1_EVENT,
        BTN2_EVENT
      };

      // Variables
      unsigned int tempBroadcastThreshold = 0;
      unsigned int tempConsecutiveReadings = 0;
      unsigned int tempReadingCount = 0;
      int tempReadingDir = 0;

      unsigned int ldrBroadcastThreshold = 0;
      unsigned int ldrConsecutiveReadings = 0;
      unsigned int ldrReadingCount = 0;
      int ldrReadingDir = 0;

      float lastTempReading = 0;
      int lastLdrReading = 0;
      bool lastBtn1Reading = false;
      bool lastBtn2Reading = false;

      onDataCallbackFunc onDataCallBack;

    public:

      PetduinoSerial();

      // Methods
      virtual void begin();
      virtual void begin(long baudRate);
      virtual void update();

      virtual void setLed(bool value);
      virtual void toggleLed();

      virtual void setNextState(unsigned int state, unsigned long interval);

      virtual void setTemperatureBroadcastThreshold(unsigned int threshold, unsigned int consecutiveReadings = 3);
      virtual void setLightLevelBroadcastThreshold(unsigned int threshold, unsigned int consecutiveReadings = 3);
      virtual void setOnDataCallback(onDataCallbackFunc callback);

      // Handlers
      void onSetState();
      void onGetState();
      void onSetLed();
      void onToggleLed();
      void onGetLed();
      void onGetTemperature();
      void onGetLightLevel();
      void onSetData();

};

#endif  //PetduinoSerial.h

# PetduinoSerial
PetduinoSerial is a wrapper around the original [Petduino library](https://github.com/circuitbeard/petduino) adding a serial interface to all the Petduino actions, allowing you to both control and listen to your Petduino over serial.

## Installation
1. Follow the standard [Petduino instalation instructions](https://github.com/circuitbeard/petduino).
3. Download and install the latest [CmdMessenger arduino library](https://github.com/thijse/Arduino-CmdMessenger#downloading).
4. Download this library from the [Releases page](https://github.com/circuitbeard/petduino-serial/releases).
5. Install this library using [the standard Arduino library install procedure](http://www.arduino.cc/en/Guide/Libraries#.UwxndHX5PtY).

## Getting Started
Every PetduinoSerial project starts with the same basic code structure.

    #include <LedControl.h>
    #include <Petduino.h>

    #include <CmdMessenger.h>
    #include <PetduinoSerial.h>

    PetduinoSerial pet = PetduinoSerial();

    void setup() {

      // Setup Petduino
      pet.begin(9600); // Serial baud rate

    }

    void loop() {

      // Update pet
      pet.update();

      // Do your thing...

    }

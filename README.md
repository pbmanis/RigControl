RigControl
==========

This is a little program that controls various helper hardware on a brain slice rig through
an Arduino. It provides a little window for user interaction, and a USB connection to the Arduino,
as well as some external power and other hardware.

Provides the following functions:

1. Temperature controller (Luke Campagnola, uses PWM to control a resistive pad)
2. Valve switching for solution changes.
3. LEDs for rig illumination.
4. LEDS for fluorescence.
5. Recirculation bath stepping motor to drive a "washing machine" peristaltic pump.


Hardware
--------
Arduino Uno or Mega2560. 
Sparkfun Power driver board (to switch solution valves)
Adafruit Stepping motor driver breakout.

External
--------
LEDs (Thorlabs with Mightex drivers)
Valves (Cole-Parmer)
LEDS (misc. for rig illumination)

Software
--------
Arduino IDE (c)
Python 3 (+ pyqtgraph)

2024, pbm


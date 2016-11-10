# Arduino Controlled Fan Tester
Arduino based circuit for testing PC fans.  General purpose circuitry to demonstrate programming and engineering concepts

This sketch controls circuitry to test PC fans in a learning environment.

Fans can be powered on, speed varied using fan PWM (if present), or by PWM the fan power if no PWM input is on the fan. The fan TACH output is read and the fan RPM is automatically calculated and displayed. A separate FAN_TACH_LAMP pulse of variable pulse width is generated and is used to strobe the fan using LEDs for demonstration purposes. The user may manually control the FAN_TACH_LAMP pulse also. The current used by the fan is read and displayed.

Both the 9 VDC battery voltage and the 12VDC external voltage (if applied and selected) are monitored.

The program makes extensive use of the KY-040 Encoder library for the user interface. One encoder is used to control menu selection and modifying programmed and displayed values including:

- Fan RPM - the RPM duty cycle can be adjusted from 30% to 100%
- Fan RPM display - the RPM display can be toggled between RPM (0 to 9999) and mSec (PRI)
- Fan PWM destination - PWM can be applied to either the PWM line of the fan, or to the FET driver Q2 if the fan has no PWM line.
- Lamp sync can be toggled between FAN (if a TACH signal is present on the fan), MANually, where the user may program the lamp RPM and pulse width, or OFF.
- Lamp RPM - the user may set the Lamp RPM between 100 and 9999.
- Lamp PW - the user may program the lamp pulse width (ON time) between 0.1 and 50 msec.
- Display brightness - the user may program the LCD display between 0 (OFF) and 255 (fully ON).
- Backlight - the user may toggle the backlight ON and OFF

The library may be found at https://github.com/Billwilliams1952/KY-040-Encoder-Library---Arduino.

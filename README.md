# Arduino-Tachometer-Rangefinder
Arduino based circuit for testing PC fans.  General purpose circuitry to demonstrate programming and engineering concepts

This project controls circuitry to test PC fans in a learning environment.

Fans can be powered on, speed varied using fan PWM (if present), or by PWM the fan power if no PWM input is on the fan. The fan TACH output is read and the fan RPM is automatically calculated and displayed. A separate TACH_SYNC pulse of 1 msec is generated and is used to strobe the fan using LEDs for demonstration purposes. The user may manually control the TACH_SYNC pulse also. The current used by the fan is read and displayed.

Both the battery voltage and the 12VDC external voltage (if applied and selected) are monitored.

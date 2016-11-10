# Arduino Controlled Fan Tester
Arduino based circuit for testing PC fans.  General purpose circuitry to demonstrate programming and engineering concepts

This sketch controls circuitry to test PC fans in a learning environment.

Fans can be powered on, speed varied using fan PWM (if present), or by PWM the fan power if no PWM input is on the fan. The fan TACH output is read and the fan RPM is automatically calculated and displayed. A separate FAN_TACH_LAMP pulse of variable pulse width is generated and is used to strobe the fan using LEDs for demonstration purposes. The user may manually control the FAN_TACH_LAMP pulse also. The current used by the fan is read and displayed.
 
A battery monitor checks if the battery volktage is below 7V, if so, everything halts.

Both the battery voltage and the 12VDC external voltage (if applied and selected) are monitored.

The program makes extensive use of the KY-040 Encoder library.


/*
 * FanTesting.ino
 * 2016 WLWilliams
 * 
 * This sketch does TBS
 * 
 * This program is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version. 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of
 * the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG_SERIAL

#define TACH_INTERRUPT_PIN  3
#define TACH_INTERRUPT_PIN_FOR_INT0  0
#define BOARD_LED           13

volatile byte tachPulseCount = 0;
volatile boolean tachReadingReady = false;
volatile unsigned long deltaTime;

void setup() {                
    pinMode(TACH_INTERRUPT_PIN,INPUT_PULLUP);
    // digitalPinToInterrupt(TACH_INTERRUPT_PIN) doesn't compile on Linux Mint 18
    attachInterrupt(TACH_INTERRUPT_PIN_FOR_INT0, FanTachInterrupt, RISING);	
    
    Serial1.begin(19200);
    Serial1.write(0xFE); Serial1.write(0x58); 
  
#if defined ( DEBUG_SERIAL )
    Serial.begin(19200); 
    Serial.print("Ready...");
#endif
}

void loop() {   
    digitalWrite(BOARD_LED,millis() % 1000 > 500);
    
    if ( tachReadingReady ) {
      int rpm = (int)((float)tachPulseCount / (float)deltaTime  * 60.0e6);
      tachPulseCount = 0;
      tachReadingReady = false;
      Serial1.write(0xFE); Serial1.write(0x58); 
      Serial1.print(RPM_STRING); Serial1.print(rpm);
#if defined ( DEBUG_SERIAL )
      Serial.print(RPM_STRING); Serial.println(rpm);
#endif
    }
}

/*
  Count interrupt occurances over a 1 second interval
*/
void FanTachInterrupt ( void ) {
    static unsigned long currentTime, lastTime = micros();
    
    currentTime = micros();
    if ( tachReadingReady ) {
        lastTime = currentTime;
    }
    else {
        deltaTime = currentTime - lastTime;
        tachPulseCount ++;
        if ( deltaTime >= 1000000UL ) {
          lastTime = currentTime;
          tachReadingReady = true; 
        }
    }
}


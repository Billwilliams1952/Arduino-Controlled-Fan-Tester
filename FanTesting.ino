
/*
 * FanTesting.ino
 * 2016 WLWilliams
 * 
 * This sketch controls circuitry to test PC fans in a learning environment.
 * Fans can be powered on, speed varied using fan PWM (if present), or by PWM the fan
 * power if no PWM input is on the fan. The fan TACH output is read and the fan RPM is 
 * automatically calculated and displayed. A separate TACH_SYNC pulse of 1 msec is
 * generated and is used to strobe the fan using LEDs for demonstration purposes. The user may
 * manually control the TACH_SYNC pulse also. The current used by the fan is read and displayed, 
 * along along with power dissipation.
 *
 * Safety features include battery monitor, AND WHAT ELSE??
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
//#define CHANGE_STARTUP_SCREEN

#define FAN_PWM_CONTROL     A0
#define BATTERY             A1

#define FAN_PWM_OUT         5
#define TACH_INTERRUPT_PIN  3
#define TACH_1MSEC_OUTPUT   4
#define TACH_INTERRUPT_PIN_FOR_INT0  0
#define BOARD_LED           13

#define PULSES_FOR_1MSEC    120

volatile byte tachPulseCount = 0;
volatile boolean tachReadingReady = false;
volatile unsigned long deltaTime;

void ClearScreen ( void ) {
  Serial1.write(0xFE); Serial1.write(0x58);
}

void setup() {
    pinMode(FAN_PWM_OUT,OUTPUT);
    digitalWrite(FAN_PWM_OUT,LOW);  
  
    pinMode(TACH_1MSEC_OUTPUT,OUTPUT);  
    digitalWrite(TACH_1MSEC_OUTPUT,LOW);
    
    pinMode(TACH_INTERRUPT_PIN,INPUT_PULLUP); 
    // digitalPinToInterrupt(TACH_INTERRUPT_PIN) doesn't compile on Linux Mint 18
    attachInterrupt(TACH_INTERRUPT_PIN_FOR_INT0, FanTachInterrupt, RISING);	
    
    Serial1.begin(19200);

    ClearScreen();
    
#if defined ( CHANGE_STARTUP_SCREEN )
    Serial1.write(0xFE); Serial1.write(0x40);
#endif
    Serial1.write("  FAN TESTING     VERSION 0.1   ");
    delay(3000);
    
    ClearScreen(); 
    
    /* 
       Programming TIMER1 as an interrupt. We will want to start counting when the TACH signal is
       received. Assuming 16usec per count, this will take ~ 63 counts 
    */
    TIMSK1  = 0x03;                 // Enable Comparator A and Overflow interrupts
    TCCR1A  = 0x00;                 // Normal operation, timer disabled
    TCCR1B  = 0x00;
  
#if defined ( DEBUG_SERIAL )
    Serial.begin(19200); 
    Serial.print("Ready...");
#endif
}

void loop() {   
    digitalWrite(BOARD_LED,millis() % 1000 > 500);
    
    // NEED TO FIX DISPLAY ISSUES
    //ClearScreen();
    
    ChangeFanSpeed();
    PrintTachReading();
    CheckBattery();
    
    delay(200);    // Update 5 times a second (about)
}

void ChangeFanSpeed ( void ) {
    int reading = analogRead(FAN_PWM_CONTROL); 
    if ( reading <= 306 )
      reading = 307;        // Clamp to 30%, this is the minimum DC spec'd for PC fans
    /* If the fan supports direct PWM, make sure we switch it in */
    analogWrite(FAN_PWM_OUT,map(reading, 0, 1023, 0, 255));
    Serial1.write(0xFE); Serial1.write(0x47); Serial1.write(1); Serial1.write(2);
    Serial1.print("PWM "); Serial1.print((int)((float)reading/1023.0 * 100.0)); Serial1.print("% "); 
}

void PrintTachReading ( void ) {
   static unsigned long rpmTimeout = millis();
   static boolean firstTime = true;
    
   if ( tachReadingReady ) {
      // Divide by 2 since there are two pulses per revolution
      int rpm = (int)((float)tachPulseCount / (float)deltaTime / 2.0 * 60.0e6);
      tachPulseCount = 0;
      tachReadingReady = false;
      ClearScreen();  
      Serial1.print("RPM: "); Serial1.print(rpm);
#if defined ( DEBUG_SERIAL )
      Serial.print("RPM: "); Serial.println(rpm);
#endif
      rpmTimeout = millis();
      firstTime = true;
    }
    
    if ( millis() - rpmTimeout > 2000 ) {
      if ( firstTime ) {
        ClearScreen();
        Serial1.print("RPM NOT DETECTED");
        firstTime = false;
      }
    }
}

void CheckBattery ( void ) {
    /* 100K and 82K resistor divider. Assuming a 9VDC battery, then the 
       max voltage seen by the A/D is 4.95V, close enough to 5V. So we can just
       use the ratio of reading versus full scale of the A/D.
    */
    float voltage;
    static char buff[6];
    do {
        voltage = (float)analogRead(BATTERY) / 1023.0 * 9.0;
        if ( voltage <= 7.0 ) {
            ClearScreen();  
            Serial1.print("   LOW VOLTAGE  ");
            dtostrf(voltage,6,2,buff);
            Serial1.print("    "); Serial1.print(buff); Serial1.write("V");
            Serial.println((int)voltage);
            delay ( 500 );
        }
    } while ( voltage <= 7.0 );
}

/*
  Count interrupt occurances over a 1 second interval
*/
void FanTachInterrupt ( void ) {
    static unsigned long currentTime, lastTime = micros();
    
    digitalWrite(TACH_1MSEC_OUTPUT,HIGH);    // Turn ON TACH output pulse
    OCR1A = PULSES_FOR_1MSEC;                // How much to count up to
    TCCR1B = 0x04;                           // start timer with divide by 256 prescaler
    TCNT1 = 0;                               // Count from 0 up to value in OCR1A register
    
    /* Set TACH output High, setup Timer1 to expire in 1 msec */
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

/* 
   We get here after ~1000 usec has elapsed from the FanTachInterrupt. Now its time to turn
   the TACH output OFF, and return to normal (non-timer) interrupts.
*/
ISR ( TIMER1_COMPA_vect ) {
    // 1 msec has passed. Turn OFF TACH output
    digitalWrite(TACH_1MSEC_OUTPUT,LOW);
    TCCR1B = 0x00;    // Normal operation again
}


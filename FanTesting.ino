                                            
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
 * along along with power dissipation???
 *
 * A battery monitor checks if the battery volktage is below 7V, if so, everything halts.
 * 
 * This program is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version. 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of
 * the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//#define CHANGE_STARTUP_SCREEN

#define FAN_PWM_CONTROL     A0
#define BATTERY             A1
#define CURRENT_SENSE_1     A2
#define FAN_VOLTAGE         A3

#define BOARD_LED          13
#define ENCODER_DT         12
#define ENCODER_SW         11
#define SYNC_TO_TACH        6
#define FAN_PWM_OUT         5
#define TACH_1MSEC_OUTPUT   4
#define TACH_INTERRUPT      3
#define ENCODER_CLK         2

#define ENCODER_INTERRUPT_1  1
#define TACH_INTERRUPT_0     0

#define PULSES_FOR_1MSEC    120
#define ONE_SECOND          1000000UL
#define NO_RPM_DETECTED     -1

volatile byte tachPulseCount = 0, fanPWMDC = 100;
volatile boolean tachReadingReady = false, adjustingRPM = true;
volatile unsigned long deltaTime;

void ClearScreen ( void ) {
  Serial1.write(0xFE); Serial1.write(0x58);
}

void setup() {  
    pinMode(BOARD_LED,OUTPUT);  // I'm alive pulser
    digitalWrite(BOARD_LED,HIGH);
    
    pinMode(SYNC_TO_TACH,INPUT_PULLUP);
    
    pinMode(FAN_PWM_OUT,OUTPUT);
    digitalWrite(FAN_PWM_OUT,LOW);  
  
    pinMode(TACH_1MSEC_OUTPUT,OUTPUT);  
    digitalWrite(TACH_1MSEC_OUTPUT,LOW);
    
    pinMode(TACH_INTERRUPT,INPUT_PULLUP); 
    // digitalPinToInterrupt(TACH_INTERRUPT_PIN) doesn't compile on Linux Mint 18
    attachInterrupt(TACH_INTERRUPT_0, FanTachInterrupt, RISING);	
    
    pinMode(ENCODER_SW,INPUT_PULLUP);
    pinMode(ENCODER_DT,INPUT_PULLUP);
    pinMode(ENCODER_CLK,INPUT_PULLUP); 
    // digitalPinToInterrupt(TACH_INTERRUPT_PIN) doesn't compile on Linux Mint 18
    attachInterrupt(ENCODER_INTERRUPT_1, RotaryEncoderClkInterrupt, RISING);
    
    Serial1.begin(19200);
    
#if defined ( CHANGE_STARTUP_SCREEN )
    Serial1.write(0xFE); Serial1.write(0x40);
#endif
    Serial1.write("  FAN TESTING     VERSION 0.1   ");
    delay(3000);
    
    /* 
       Programming TIMER1 as an interrupt. We will want to start counting when the TACH signal is
       received. Assuming 16usec per count, this will take ~ 63 counts 
    */
    TIMSK1  = 0x03;                 // Enable Comparator A and Overflow interrupts
    TCCR1A  = 0x00;                 // Normal operation, timer disabled
    TCCR1B  = 0x00;                 // Ditto
}

typedef enum { IN_INIT, IN_MAIN, IN_BATTERY_1, IN_BATTERY_2, IN_LOW_BATTERY } States;

void loop() {   
    static unsigned long stateTime = millis();
    static States state = IN_INIT;
    
    // Change blink rate if voltage is below 8VDC
    digitalWrite(BOARD_LED,millis() % 1000 > 500);
    
    // Check if SYNC_TO_TACH switch S1 is closed, if so, then new screen
    // showing Measured RPM versus LAMP flicker rate 
    
    switch ( state ) {
        case IN_INIT:
            state = IN_MAIN;
            break;
        case IN_MAIN:
            if ( millis() - stateTime >= 200 ) {
              UpdateMainScreen();
              stateTime = millis();
            }
            break;
    }
    
    delay(50);    // Update 20 times a second (about)
}

void UpdateMainScreen ( void ) {
    static unsigned long time = millis();
    int dc, rpm;
    float volts, current;
    char buff[7];
    
    // Go to 1,1 on LCD
    Serial1.write(0xFE); Serial1.write(0x48); 
    
    volts = GetBatteryVoltage();
    if ( volts <= 7.0 ) {
      // print message and halt here
    }
    else if ( millis() - time >= 10000 ) {
      // display the battery voltage
      ClearScreen();
      Serial1.print("  Battery: "); dtostrf(volts,4,1,buff); Serial1.print(buff); Serial1.print("V");
      delay(2000);
      time = millis();
      return;
    }
    
    dc = GetFanPWMDutyCycle();
    rpm = GetRPM();
    volts = GetFanVoltage();
    current = GetCurrent();
     
    Serial1.print(" "); dtostrf(volts,4,1,buff); Serial1.print(buff); 
    Serial1.print("V  RPM "); 
    if ( rpm == NO_RPM_DETECTED )
      Serial1.print("----");
    else {
      sprintf(buff,"%04d",rpm);
      Serial1.print(buff); 
    }
    dtostrf(current,5,1,buff); Serial1.print(buff); 
    Serial1.print("mA  DC ");
    sprintf(buff,"%03d",dc);   Serial1.print(buff);
    Serial1.print('%');
    
}

int GetFanPWMDutyCycle( void ) {
    int reading = analogRead(FAN_PWM_CONTROL); 
    if ( reading <= 306 )
      reading = 307;        // Clamp to 30%, this is the minimum DC spec'd for PC fans
    
    analogWrite(FAN_PWM_OUT,map(reading, 0, 1023, 0, 255));; 
    return (int)((float)reading/1023.0 * 100.0);
}

int GetRPM ( void ) {
   static unsigned long rpmTimeout = millis();
   static int lastRpm = NO_RPM_DETECTED;
   if ( tachReadingReady ) {
      noInterrupts();    // Should never get a TACH interrupt, but just in case
      // Divide by 2 since there are two pulses per revolution
      lastRpm = (int)((float)tachPulseCount / (float)deltaTime / 2.0 * 60.0e6);
      tachPulseCount = 0;
      tachReadingReady = false;
      interrupts();
      if ( lastRpm > 9999 )    // something funny happened
        lastRpm = NO_RPM_DETECTED;
      rpmTimeout = millis();
      return lastRpm;
    }
    else {
      if ( millis() - rpmTimeout > 2000 ) {
        return NO_RPM_DETECTED;
      }
      else {
        return lastRpm;
      }
    }
}

float GetFanVoltage ( void ) {
    /* 100K and 82K resistor divider. Assuming a 9VDC battery, then the 
       max voltage seen by the A/D is 4.95V, close enough to 5V. So we can just
       use the ratio of reading versus full scale of the A/D.
    */
    // CHANGE WHEN IMPLEMENTED IN TEST 
    return (float)analogRead(BATTERY) / 1023.0 * 12.0;
}

float GetBatteryVoltage ( void ) {
    /* 100K and 82K resistor divider. Assuming a 9VDC battery, then the 
       max voltage seen by the A/D is 4.95V, close enough to 5V. So we can just
       use the ratio of reading versus full scale of the A/D.
    */
    return (float)analogRead(BATTERY) / 1023.0 * 12.0;
}

float GetCurrent ( void ) {
    unsigned long val = 0;
    #define MAX_AVERAGES 200
    
    // Set to internal 2.5V reference, delay a bit, then make several inital readings
    // to stabilize the input. 
    analogReference(INTERNAL);    // ~2.5V
    delay(50);
    for ( int i = 0; i < 5; i++ )
      analogRead(CURRENT_SENSE_1);

    // Average a bunch of readings
    for ( int i = 0; i < MAX_AVERAGES; i++ ) {
       val += analogRead(CURRENT_SENSE_1);
    }  
    val = val / MAX_AVERAGES;   
    // 2.5V nominal internal ref / 1023 or 2.4437928 mA per bit assuming 1ohm resistor
    float current = (float)val * 2.4437928;
    if ( current < 0.0 ) current = 0.0;  // means an overrange??
    
    // Same as above, reset back
    analogReference(DEFAULT);
    delay(50);
    for ( int i = 0; i < 5; i++ )
      analogRead(CURRENT_SENSE_1);

    return current; // basically 2.44 mA per bit
}

/* Read encoder DT value, this determines direction */
void RotaryEncoderClkInterrupt ( void ) {
    if ( digitalRead(ENCODER_DT) )
        iValue += 1;
    else
        iValue -= 1;
}

/*
  Count interrupt occurances over a 1 second interval
*/
void FanTachInterrupt ( void ) {
    static unsigned long currentTime, lastTime = micros();
    
    /* Set TACH output High, setup Timer1 to expire in 1 msec */   
    digitalWrite(TACH_1MSEC_OUTPUT,HIGH);    // Turn ON TACH output pulse
    OCR1A = PULSES_FOR_1MSEC;                // How much to count up to
    TCCR1B = 0x04;                           // start timer with divide by 256 prescaler
    TCNT1 = 0;                               // Count from 0 up to value in OCR1A register
    
    currentTime = micros();
    if ( tachReadingReady ) {
        lastTime = currentTime;
    }
    else {
        deltaTime = currentTime - lastTime;
        tachPulseCount ++;
        if ( deltaTime >= ONE_SECOND ) {
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


                                            
/*
 * FanTesting.ino
 * 2016 WLWilliams
 * 
 * This sketch controls circuitry to test PC fans in a learning environment.
 * Fans can be powered on, speed varied using fan PWM (if present), or by PWM the fan
 * power if no PWM input is on the fan. The fan TACH output is read and the fan RPM is 
 * automatically calculated and displayed. A separate FAN_TACH_LAMP pulse of variable pulse width is
 * generated and is used to strobe the fan using LEDs for demonstration purposes. The user may
 * manually control the FAN_TACH_LAMP pulse also. The current used by the fan is read and displayed.
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

#include <Arduino.h>
#include <avr/interrupt.h> 
#include <avr/io.h>
#include <ky-040.h>                     // My library for using the KY-040 encoder

#define FREE_MEM_TEST                 // Define to show max/min memory used under Debug
//#define CHANGE_STARTUP_SCREEN         // If defined, text is written to LCD memory to define
                                        // LCD startup text

#define BATTERY_SENSE       A1          // 9VDC divided to give 0 to 5V max
#define CURRENT_SENSE       A2          // Reference set to 2.5V giving ~ 2.44mA per bit
#define FAN_VOLTAGE         A3          // 12VDC divided to give 0 to 5 VDC

#define BOARD_LED           13          // Generic I'm Alive LED
#define ENCODER_DT          12          // Rotary DT line. Direct input
#define ENCODER_SW          11          // Rotary switch, active LOW. 470nF on pin
#define FAN_DRIVER_PWM_OUT   6          // PWM controlled by Timer0
#define FAN_PWM_OUT          5          // PWM controlled by Timer0
#define FAN_TACH_LAMP        4          // Variable PRI and Pulsewidth by Timer1
#define TACH_INPUT           3          // Fan TACH. Interrupt 0, Active HIGH
#define ENCODER_CLK          2          // Rotary switch. Interrupt 1, Active LOW. 470nF on pin

#define PULSES_FOR_1MSEC     250        // 4 usec per count assuming divide by 64 prescaler on 16 MHz clock
#define PULSES_FOR_1200RPM   12500      // 4 usec per count assuming divide by 64 prescaler on 16 MHz clock  
#define ONE_SECOND           1000000UL  // Time interval to count pulses to determine RPM
#define NO_RPM_DETECTED      -1         // Return from TACH interrupt procedure if no RPM detected
#define MIN_DUTY_CYCLE       30         // Fan min PWM
#define MAX_DUTY_CYCLE       100        // Fan max PWM
#define MIN_BRIGHTNESS       0          // LCD display
#define MAX_BRIGHTNESS       255        // LCD display

#define PULSE_LED            digitalWrite(BOARD_LED,millis() % 1000 > 500);

#define ROLLOVER             true
#define NO_ROLLOVER          false

typedef enum { MAIN_MENU, PWM_DC, BRIGHTNESS, LAMP_SOURCE, HUNDREDS, DIGITS, LAMP_PW_MSEC, GENERIC_MENU } RotaryNames;
typedef enum { FANTEST_SCREEN, LAMP_SCREEN, BATTERY_SCREEN, DISPLAY_SCREEN, DEBUG_SCREEN, ABOUT_SCREEN } MainMenuItems;
typedef enum { CURSOR_OFF, BLINKING_BLOCK, UNDERLINE } CursorType;
typedef enum { FANTEST_MENU_RETURN, FANTEST_MENU_TOGGLE_DISPLAY, FANTEST_MENU_DC, FANTEST_MENU_SETUP } FanTestMenu;
typedef enum { FANTEST_SETUP_RETURN, FANTEST_SETUP_PWM_DEST } FanTestSetupMenu;
typedef enum { LAMP_MENU_RETURN, LAMP_MENU_DC, LAMP_MENU_HUNDREDS, LAMP_MENU_DIGITS, LAMP_MENU_SETUP } LampMenu;
typedef enum { LAMP_SETUP_RETURN, LAMP_SETUP_SOURCE, LAMP_SETUP_TOGGLE_DISPLAY, LAMP_SETUP_PW } LampSetupMenu;
typedef enum { LAMP_OFF, LAMP_SYNC_TO_FAN, LAMP_SET_MANUALY } LampSource;
typedef enum { DISPLAY_MENU_RETURN, DISPLAY_MENU_BRIGHTNESS, DISPLAY_MENU_BACKLIGHT } DisplayMenu;
typedef enum { MENU_RETURN } GenericMenuItems;

/* Interrupt variables */
volatile uint8_t        tachPulseCount = 0;
volatile boolean        tachReadingReady = false;
volatile uint32_t       deltaTime;
volatile uint16_t       pulsesForLampRPM = PULSES_FOR_1200RPM, pulsesForLampPW = PULSES_FOR_1MSEC;
volatile uint8_t        clockPrescaler = 0x03;    // Four microseconds per timer tick

/* Common variables */
uint8_t dc, loopCounter;
int16_t rpm;
float   volts, current;
char    buff[7];
boolean backlight = true, displayAsRPM = true, pwmToFan = true;

#ifdef FREE_MEM_TEST
    // Debug memory
    int     minMemory = 10000, maxMemory = 0;
    
    /* Free memory calculation
     * http://www.arduino.cc/playground/Code/AvailableMemory.
     */ 

    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;
    
    void freeMemory ( void ) {
      int free_memory;
    
      if((int)__brkval == 0)
         free_memory = ((int)&free_memory) - ((int)&__bss_end);
      else
        free_memory = ((int)&free_memory) - ((int)__brkval);
    
      if ( free_memory > maxMemory )
          maxMemory = free_memory;
      else if ( free_memory < minMemory )
          minMemory = free_memory; 
    }
    
    #define FREE_MEM  freeMemory();
#else
    #define FREE_MEM
#endif

// Use the KY-040 class to handle the state of the rotary for all of the various test cases....
ky040 rot1(ENCODER_CLK, ENCODER_DT, ENCODER_SW, 8 );

/*
 *          LCD Screen Functions
 */
void ClearScreen ( void ) {
    Serial1.write(0xFE); Serial1.write(0x58);
}

void SetCursorType ( CursorType cursorType ) {
    // There is NO blinking underline
    
    Serial1.write(0xFE);  Serial1.write(0x4B); 
    Serial1.write(0xFE);  Serial1.write(0x54);
    
    switch ( cursorType ) {
        case BLINKING_BLOCK:
            Serial1.write(0xFE);  Serial1.write(0x53);
            break;
        case UNDERLINE:
            Serial1.write(0xFE);  Serial1.write(0x4A);
            break;        
    }
}

void GoToXY ( uint8_t x, uint8_t y ) {
    Serial1.write(0xFE); Serial1.write(0x47); 
    Serial1.write(x); Serial1.write(y);
}

void GoHome ( void ) {
    Serial1.write(0xFE); Serial1.write(0x48);
}

/* ------------------------------------------------------*/

void setup() {
    FREE_MEM
  
    Serial.begin(9600);
  
    pinMode(BOARD_LED,OUTPUT);                // I'm alive pulser
    digitalWrite(BOARD_LED,HIGH);             // Initially OFF
    
    pinMode(FAN_DRIVER_PWM_OUT,OUTPUT);       // PWM to FET Q2 or FAN
    pinMode(FAN_PWM_OUT,OUTPUT);              
  
    pinMode(FAN_TACH_LAMP,OUTPUT);            // TACH Lamp not ON
    digitalWrite(FAN_TACH_LAMP,LOW);
    
    pinMode(TACH_INPUT,INPUT_PULLUP);         // TACH interrupt
    attachInterrupt(digitalPinToInterrupt(TACH_INPUT), FanTachInterrupt, RISING);	
    
    Serial1.begin(19200);                     // LCD communication
    delay(200);

    rot1.AddRotaryCounter(MAIN_MENU, FANTEST_SCREEN, FANTEST_SCREEN, ABOUT_SCREEN, 1, ROLLOVER );
    rot1.AddRotaryCounter(PWM_DC, 80, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE, 5, NO_ROLLOVER );
    rot1.AddRotaryCounter(BRIGHTNESS, MAX_BRIGHTNESS, MIN_BRIGHTNESS, MAX_BRIGHTNESS, 5, NO_ROLLOVER );
    rot1.AddRotaryCounter(LAMP_SOURCE, LAMP_SYNC_TO_FAN, LAMP_OFF, LAMP_SET_MANUALY, 1, ROLLOVER );
    rot1.AddRotaryCounter(HUNDREDS, 12, 2, 99, 1, ROLLOVER );          // Setup for 1200 RPM
    rot1.AddRotaryCounter(DIGITS, 0, 0, 99, 1, ROLLOVER );             // Ditto
    rot1.AddRotaryCounter(LAMP_PW_MSEC, 20, 1, 500, 1, NO_ROLLOVER );  // Will divide by 10 - 0.1 to 50.0 msec
    rot1.AddRotaryCounter(GENERIC_MENU, 0, 0, 1, 1, ROLLOVER );        // Generic used all over

    GetFanPWMDutyCycle();                     // Start up the fan
    
#if defined ( CHANGE_STARTUP_SCREEN )
    Serial1.write(0xFE); Serial1.write(0x40);
    Serial1.print(F("  FAN TESTING     VERSION 1.0   "));   // Change version number as required
#endif

    delay(3000);
}

// Loop is not really used.  Note, RS232 over Serial is affected without some calls
// to Serial via: if (serialEventRun) serialEventRun();
void loop() { 
    FREE_MEM
      
    switch ( MainMenu() ) {
        case FANTEST_SCREEN:
            FanTestScreen();
            break;
        case LAMP_SCREEN:
            LampScreen();
            break;
        case BATTERY_SCREEN:
            SupplyScreen();
            break;
        case DISPLAY_SCREEN:
            DisplayScreen();
            break; 
        case DEBUG_SCREEN:
            DebugScreen();
            break;
        case ABOUT_SCREEN:
            AboutScreen();
            break;  
    }
}

MainMenuItems MainMenu ( void ) {
    MainMenuItems menuPos = FANTEST_SCREEN;   
    uint8_t mainMenuPosX[] = {1, 10, 1, 10, 1, 10}; 
    uint8_t mainMenuPosY[] = {1, 1, 2, 2, 1, 1};

    FREE_MEM
    
    rot1.SetRotary(MAIN_MENU);
    rot1.SetChanged();

    SetCursorType(BLINKING_BLOCK);

    while ( ! rot1.SwitchPressed() ) {
        PULSE_LED
        if ( rot1.HasRotaryValueChanged() ) {
            menuPos = (MainMenuItems)rot1.GetRotaryValue();
            GoHome();
            if ( menuPos <= DISPLAY_SCREEN ) {
                Serial1.print(F("FANTEST  LAMP   "));
                Serial1.print(F("BATTERY  DISPLAY"));
            }
            else {
                Serial1.print(F("DEBUG    ABOUT  "));
                Serial1.print(F("                "));
            }
            GoToXY(mainMenuPosX[menuPos],mainMenuPosY[menuPos]);
        }
    }
    
    return menuPos;
}

void FanTestScreen ( void ) {
    uint8_t x, y;
    FanTestMenu menuPos = FANTEST_MENU_RETURN;   
    uint8_t mainMenuPosX[] = {1, 10, 9, 16 }; 
    uint8_t mainMenuPosY[] = {1, 1, 2, 2 };

HackGoToFanTest:    // Yes yes, bad programming - sue me. Trying to keep indents under control

    FREE_MEM
    
    rot1.SetRotary(GENERIC_MENU);
    rot1.SetMaxValueOnRotary(3,GENERIC_MENU);
    rot1.SetChanged(GENERIC_MENU);

    GoHome();

    Serial1.print(F("^")); GoToXY(6,1); Serial1.print(F("V  "));
    GoToXY(6,2); Serial1.print(F("mA ")); GoToXY(12,2); Serial1.print(F("% DC^"));

    while ( true ) {
        PULSE_LED
        
        if ( rot1.HasRotaryValueChanged(GENERIC_MENU) ) {
            menuPos = (FanTestMenu)rot1.GetRotaryValue(GENERIC_MENU);
            x = mainMenuPosX[menuPos]; y = mainMenuPosY[menuPos];
            GoToXY(x,y);
        }
        
        GetFanVoltage(); dtostrf(volts,4,1,buff); 
        
        /*  All of the GotoXY()'s; Serial1.print()'s; GotoXY()'s are there to minimize 'flickering'
         *  of the block cursor. We move the cursor just long enough to print, then
         *  immediately move it back to its blinking position.
         */
        GoToXY(2,1); Serial1.print(buff); GoToXY(x,y); 

        rpm = GetRPM();
        if ( rpm == NO_RPM_DETECTED ) 
            sprintf(buff,"----");
        else if ( displayAsRPM )
            sprintf(buff,"%04d",rpm);
        else {
            float val = 60.0 / rpm * 1000.0;  // now in mSec 
            dtostrf(val,5,1,buff); 
        }
        GoToXY(9,1); Serial1.print(buff); GoToXY(x,y); 
        if ( displayAsRPM ) {
            GoToXY(13,1); Serial1.print(F(" RPM"));
        }
        else {
            GoToXY(14,1); Serial1.print(F(" ms"));
        }
        GoToXY(x,y); 

        GetCurrent(); dtostrf(current,5,1,buff); 
        GoToXY(1,2); Serial1.print(buff); GoToXY(x,y); 

        GetFanPWMDutyCycle(); sprintf(buff,"%03d",dc);  
        GoToXY(9,2); Serial1.print(buff); GoToXY(x,y); 

        if ( rot1.SwitchPressed() ) {
            switch ( menuPos ) {
                case FANTEST_MENU_RETURN:
                    rot1.SetMaxValueOnRotary(1,GENERIC_MENU);
                    return;
                case FANTEST_MENU_TOGGLE_DISPLAY:
                    displayAsRPM = ! displayAsRPM;
                    break;
                case FANTEST_MENU_DC:
                    ToggleRotaryWithGENERIC_MENU ( PWM_DC );
                    break;
                case FANTEST_MENU_SETUP:
                    FanTestSetup();
                    goto HackGoToFanTest;
            }
        }
        delay(100);
    } 
}

void FanTestSetup ( void ) {
    uint8_t x;
    FanTestSetupMenu menuPos = FANTEST_SETUP_RETURN;   
    static const char * fanPWMDest[] = {"FET Q2", "FAN   " };

    FREE_MEM

    rot1.SetRotary(GENERIC_MENU);
    rot1.SetMaxValueOnRotary(1,GENERIC_MENU);

//    "^  PWM to: FAN/FET Q2"     // FAN or FET Q2                 

    ClearScreen();
    GoHome();
    
    Serial1.print(F("^ PWM to: "));  

    while ( true ) {
        PULSE_LED
        
        if ( rot1.HasRotaryValueChanged(GENERIC_MENU) ) {
            menuPos = (FanTestSetupMenu)rot1.GetRotaryValue(GENERIC_MENU);
            x = menuPos == FANTEST_SETUP_RETURN ? 1 : 11;
            GoToXY(x,1);
        }

        GoToXY(11,1); Serial1.print(fanPWMDest[pwmToFan]); GoToXY(x,1);
          
        if ( rot1.SwitchPressed() ) {
            if ( menuPos == FANTEST_SETUP_RETURN )  return;
            pwmToFan = ! pwmToFan;
            GetFanPWMDutyCycle();     // Update PWM output pins
        }
        delay ( 100 );
    }

}

void LampScreen ( void ) {
    uint8_t x, y;
    LampMenu menuPos = LAMP_MENU_RETURN;   
    uint8_t mainMenuPosX[] = {1, 13, 8, 10, 13 }; 
    uint8_t mainMenuPosY[] = {1, 1, 2, 2, 2 };
    float period, clockPeriod, pw;
    uint16_t lastVal = rot1.GetRotaryValue(HUNDREDS) * 100 + rot1.GetRotaryValue(DIGITS);

    FREE_MEM
    
HackGoToLamp:     // Yes yes, bad programming - sue me. Trying to keep indents under control

    rot1.SetRotary(GENERIC_MENU);
    rot1.SetMaxValueOnRotary(4,GENERIC_MENU);
    rot1.SetChanged(GENERIC_MENU);
    rot1.SetChanged(HUNDREDS);
    GoHome();

    Serial1.print(F("^ Fan: "));
    GoToXY(12,1); Serial1.print(F(" "));
    GoToXY(16,1); Serial1.print(F("% "));
    GoToXY(1,2);  Serial1.print(F(" Lamp: "));
    GoToXY(12,2); Serial1.print(F(" Set "));

    while ( true ) {
        PULSE_LED

        if ( rot1.HasRotaryValueChanged(GENERIC_MENU) ) {
            menuPos = (LampMenu)rot1.GetRotaryValue(GENERIC_MENU);
            x = mainMenuPosX[menuPos]; y = mainMenuPosY[menuPos];
            GoToXY(x,y);
        }
       
        rpm = GetRPM();
        GoToXY(8,1);
        if ( rpm == NO_RPM_DETECTED )
          strcpy(buff,"----");
        else
          sprintf(buff,"%04d",rpm);
        Serial1.print(buff); GoToXY(x,y);

        switch ( (LampSource)rot1.GetRotaryValue(LAMP_SOURCE) ) {
            case LAMP_OFF:
                strcpy(buff,"    ");
                TCCR1B = 0x00;      // No interrupts
                break;
            case LAMP_SET_MANUALY:
                pw = (float)rot1.GetRotaryValue(LAMP_PW_MSEC) / 10000.0;
                if ( rot1.HasRotaryValueChanged(HUNDREDS) || rot1.HasRotaryValueChanged(DIGITS) ||
                     rot1.HasRotaryValueChanged(LAMP_SOURCE) ) {
                    lastVal = rot1.GetRotaryValue(HUNDREDS) * 100 + rot1.GetRotaryValue(DIGITS);
                    period = 60.0 / (float) lastVal;
                    noInterrupts();
                    if ( lastVal >= 1832 ) {
                        clockPeriod = 0.0000005;
                        clockPrescaler = 0x02;
                    }
                    else if ( lastVal >= 229 ) {
                        clockPeriod = 0.000004;
                        clockPrescaler = 0x03;
                    }
                    else {    // 100 RPM to 228 RPM
                        clockPeriod = 0.000016;
                        clockPrescaler = 0x04;
                    }
                    pulsesForLampRPM = (uint16_t)(period / clockPeriod);
                    pulsesForLampPW = (uint16_t)(pw / clockPeriod);
                    pulsesForLampRPM -= pulsesForLampPW;    // subtract off the pulsewidth
                    TCNT1 = 0;
                    OCR1A = pulsesForLampRPM;
                    TCCR1A = 0;
                    TCCR1B  = (1 << WGM12);       // CTC mode (compare match reset)
                    TIMSK1 |= (1 << OCIE1A);      // enable timer compare interrupt
                    TCCR1B |= clockPrescaler;     // enable counting at clockPrescaler rate 
                    interrupts();
                }
                sprintf(buff,"%04d",lastVal);
                break;
            case LAMP_SYNC_TO_FAN: 
                TCCR1B = 0x00;    // Normal operation again
                break;
        }
        
        GoToXY(8,2); Serial1.print(buff); GoToXY(x,y);
       
        GetFanPWMDutyCycle(); 
        sprintf(buff,"%03d",dc); 
        GoToXY(13,1); Serial1.print(buff); GoToXY(x,y);

        if ( rot1.SwitchPressed() ) {
            switch ( menuPos ) {
                case LAMP_MENU_RETURN:
                    rot1.SetMaxValueOnRotary(1,GENERIC_MENU);
                    return;
                case LAMP_MENU_DC:
                    ToggleRotaryWithGENERIC_MENU ( PWM_DC );
                    break;
                case LAMP_MENU_HUNDREDS:
                    if ( rot1.GetRotaryValue(LAMP_SOURCE) == LAMP_SET_MANUALY ) {
                        ToggleRotaryWithGENERIC_MENU ( HUNDREDS ); 
                        rot1.SetChanged(HUNDREDS);
                    }
                    break;   
                case LAMP_MENU_DIGITS:
                    if ( rot1.GetRotaryValue(LAMP_SOURCE) == LAMP_SET_MANUALY ) {
                        ToggleRotaryWithGENERIC_MENU ( DIGITS ); 
                        rot1.SetChanged(HUNDREDS);
                    }
                    break;                  
                case LAMP_MENU_SETUP:
                    LampSetupScreen();
                    goto HackGoToLamp; 
            }
        }
        delay(100);
    }
}

void LampSetupScreen ( void )  {
    uint8_t x, y;
    LampSetupMenu menuPos = LAMP_SETUP_RETURN;   
    uint8_t mainMenuPosX[] = {1, 7, 14, 9 }; 
    uint8_t mainMenuPosY[] = {1, 1, 1, 2 };
    float   pw;
    static const char * lampSource[] = {"OFF", "FAN", "MAN"};

    FREE_MEM
    
    rot1.SetRotary(GENERIC_MENU);
    rot1.SetMaxValueOnRotary(3,GENERIC_MENU);

//    "^Src: MAN as RPM"    // SRC FAN MAN OFF
                            // Display as RPM PRF PRI
//    "  PW: XX.X mS   "    // 0.1 to (max) 50 msec depends on current PRI                  

    GoHome();

    Serial1.print(F("^Src: "));
    GoToXY(10,1); Serial1.print(F(" as "));
    GoToXY(1,2);  Serial1.print(F("  PW: "));
    GoToXY(11,2); Serial1.print(F(" mS   "));

    while ( true ) {
        PULSE_LED

        if ( rot1.HasRotaryValueChanged(GENERIC_MENU) ) {
            menuPos = (LampSetupMenu)rot1.GetRotaryValue(GENERIC_MENU);
            x = mainMenuPosX[menuPos]; y = mainMenuPosY[menuPos];
            GoToXY(x,y);
        }
        
        GoToXY(7,1); Serial1.print(lampSource[rot1.GetRotaryValue(LAMP_SOURCE)]); GoToXY(x,y); 

        pw = (float)rot1.GetRotaryValue(LAMP_PW_MSEC) / 10.0;  
        dtostrf(pw,4,1,buff);
        GoToXY(7,2); Serial1.print(buff); GoToXY(x,y);    

        if ( rot1.SwitchPressed() ) {
            switch ( menuPos ) {
                case LAMP_SETUP_RETURN:
                    return;
                case LAMP_SETUP_SOURCE:
                    ToggleRotaryWithGENERIC_MENU ( LAMP_SOURCE ); 
                    break;
                case LAMP_SETUP_TOGGLE_DISPLAY:
                    break;
                case LAMP_SETUP_PW:
                    ToggleRotaryWithGENERIC_MENU ( LAMP_PW_MSEC ); 
                    break;
            }
        }     
        delay(100);
    }  
}

void SupplyScreen ( void ) {
    FREE_MEM
  
    rot1.SetRotary(GENERIC_MENU);
    rot1.SetMaxValueOnRotary(0,GENERIC_MENU);

    GoHome();
    Serial1.print(F("Battery: ")); GoToXY(14,1); Serial1.print(F("V  "));
    Serial1.print(F("Fan Pwr: ")); GoToXY(14,2); Serial1.print(F("V ^"));

    while ( ! rot1.SwitchPressed() ) {
        PULSE_LED
        /* 100K and 82K resistor divider. Assuming a 9VDC battery, then the 
           max voltage seen by the A/D is 4.95V, close enough to 5V. So we can just
           use the ratio of reading versus full scale of the A/D.
        */
        volts = (float)analogRead(BATTERY_SENSE) / 1023.0 * 9.0;
        dtostrf(volts,4,1,buff); 
        GoToXY(10,1); Serial1.print(buff); GoToXY(16,2);
        GetFanVoltage(); 
        dtostrf(volts,4,1,buff); 
        GoToXY(10,2); Serial1.print(buff); GoToXY(16,2);
        delay(100);
    }
}

void DisplayScreen ( void ) {
    uint8_t x, y, val;
    DisplayMenu menuPos = DISPLAY_MENU_RETURN;   
    uint8_t mainMenuPosX[] = {1, 14, 14}; 
    uint8_t mainMenuPosY[] = {1, 1, 2};

    FREE_MEM
    
    rot1.SetRotary(GENERIC_MENU);
    rot1.SetMaxValueOnRotary(2,GENERIC_MENU);
    rot1.SetChanged();
    
    GoHome();  
    Serial1.print(F("^  Display:  "));
    GoToXY(1,2); Serial1.print(F("   Backlite: "));
      
    while ( true ) {
        PULSE_LED
        
        if ( rot1.HasRotaryValueChanged(GENERIC_MENU) ) {
            menuPos = (DisplayMenu)rot1.GetRotaryValue(GENERIC_MENU);
            x = mainMenuPosX[menuPos];  y = mainMenuPosY[menuPos];
            GoToXY(x,y);
        }

        val = rot1.GetRotaryValue(BRIGHTNESS);
        if ( backlight ) {
            Serial1.write(0xFE); Serial1.write(0x99); Serial1.write(val);
        }
        
        sprintf(buff,"%3d",val);
        GoToXY(14,1); Serial1.print(buff); GoToXY(x,y);
        
        sprintf(buff,"%s",backlight ? " ON" : "OFF");
        GoToXY(14,2); Serial1.print(buff); GoToXY(x,y);

        if ( rot1.SwitchPressed() ) {
            switch ( menuPos )  {
                case DISPLAY_MENU_RETURN:
                  rot1.SetMaxValueOnRotary(1,GENERIC_MENU);
                  return;
                case DISPLAY_MENU_BRIGHTNESS:
                  ToggleRotaryWithGENERIC_MENU ( BRIGHTNESS );
                  break;
                case DISPLAY_MENU_BACKLIGHT:
                  backlight = ! backlight;
                  Serial1.write(0xFE);
                  if ( backlight ) {
                    Serial1.write(0x42); Serial1.write(0);
                  }
                  else Serial1.write(0x46);
                  break;
            }
        }
        delay(100);
    }   
}


void DebugScreen ( void ) {
    FREE_MEM
    
    ClearScreen();
    GoHome();
    
#ifdef FREE_MEM_TEST
    Serial1.print(F("Max: XXXX Bytes ")); 
    Serial1.print(F("Min: XXXX Bytes "));
    while ( ! rot1.SwitchPressed() ) {
        sprintf(buff,"%04d",maxMemory);
        GoToXY(6,1); Serial1.print(buff);
        sprintf(buff,"%04d",minMemory);
        GoToXY(6,2); Serial1.print(buff);
        delay(100);
    }
#else
    Serial1.print(F("DEBUG NOT AVAIL")); 
    while ( ! rot1.SwitchPressed() ) {
        delay(100);
    }  
#endif
    return;
}

void AboutScreen ( void ) {
    FREE_MEM
    
    GenericMenuItems menuPos = MENU_RETURN;
    
    rot1.SetRotary(GENERIC_MENU);
    rot1.SetMaxValueOnRotary(1,GENERIC_MENU);

    while ( true ) {
        PULSE_LED
        
        GoHome();    
        Serial1.print(F("^ FANTEST v1.0 >"));
        Serial1.print(F("  Bill Williams "));  
        GoHome();
        
        while ( ! rot1.SwitchPressed() ) {
            PULSE_LED
            if ( rot1.HasRotaryValueChanged() ) {
                menuPos = (GenericMenuItems)rot1.GetRotaryValue();
                GoToXY(menuPos == MENU_RETURN ? 1 : 16,1);
            }
        }
        if ( menuPos == MENU_RETURN )
            return;
            
        // Show second screen and loop until keypressed
        ClearScreen();
    
        Serial1.print(F("< DEV: LK-162-12"));
        GoToXY(1,2);
        // This code does not appear to be working
        Serial1.write(0xFE); Serial1.write(0x36);
        uint8_t ver = Serial1.read();
        Serial1.print(F("   FW: ")); Serial1.print(ver);  
        GoHome();
        
        while ( ! rot1.SwitchPressed() )
            PULSE_LED

        rot1.SetChanged();    // make sure menu is updated
    }
}

void ToggleRotaryWithGENERIC_MENU ( uint8_t rotaryID ) {
    FREE_MEM
    
    if ( rot1.IsActive(rotaryID) ) {
        rot1.SetRotary(GENERIC_MENU);
        SetCursorType(BLINKING_BLOCK);
    }
    else {
        rot1.SetRotary(rotaryID);
        SetCursorType(UNDERLINE);
    }   
}

int16_t GetRPM ( void ) {
   static uint32_t rpmTimeout = millis();
   static int16_t lastRpm = NO_RPM_DETECTED;

   FREE_MEM
   
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
    }
    else if ( millis() - rpmTimeout > 2000 )    // Two seconds for no RPM
        return NO_RPM_DETECTED;

    return lastRpm;
}

void GetCurrent ( void ) {
    uint32_t val = 0;
    #define MAX_AVERAGES 200

    FREE_MEM
    
    SetReference(INTERNAL);     // Set to internal 2.5V reference

    // Average a bunch of readings
    for ( loopCounter = 0; loopCounter < MAX_AVERAGES; loopCounter++ ) {
       val += analogRead(CURRENT_SENSE);
    }  
    val = val / MAX_AVERAGES; 
      
    // 2.5V nominal internal ref / 1023 or 2.4437928 mA per bit assuming 1ohm resistor
    current = (float)val * 2.4437928;
    if ( current < 0.0 )    current = 0.0;     // should NEVER happen
    if ( current > 999.0 )  current = 999.0;   // floating current line?
    
    SetReference(DEFAULT);     // Reset back to 5V
}

void SetReference ( uint8_t ReferenceSource ) {
    FREE_MEM
    
    // Set reference, delay a bit, then make several initial readings
    // to stabilize the input.
    analogReference(ReferenceSource);
    delay(50);
    for ( loopCounter = 0; loopCounter < 5; loopCounter++ )
        analogRead(CURRENT_SENSE);       
}

void GetFanVoltage ( void ) {
    FREE_MEM
    
    /* 100K and 82K resistor divider. Assuming a 9VDC battery, then the 
       max voltage seen by the A/D is 4.95V, close enough to 5V. So we can just
       use the ratio of reading versus full scale of the A/D.
    */
    // CHANGE WHEN IMPLEMENTED IN TEST 
    volts = (float)analogRead(BATTERY_SENSE) / 1023.0 * 12.0;
}

void GetFanPWMDutyCycle ( void ) {
    FREE_MEM
    
    dc = (uint8_t)rot1.GetRotaryValue(PWM_DC);
    /*  Depending on the state, either adjust PWM to the fan and keep FET ON or
     *  adjust PWM to the FET and keep the PWM to the fan ON
     *  This change removes the need for an external switch.
     */
    uint8_t val = map(dc, 0, 100, 0, 255);
    if ( pwmToFan ) {
        digitalWrite(FAN_DRIVER_PWM_OUT,HIGH);
        analogWrite(FAN_PWM_OUT,val);
    }
    else {
        digitalWrite(FAN_PWM_OUT,HIGH);
        analogWrite(FAN_DRIVER_PWM_OUT,val);        
    }
}

/*
  Count FAN TACH interrupt occurances over a 1 second interval
*/
void FanTachInterrupt ( void ) {
    static uint32_t currentTime, lastTime = micros();

    /* Set TACH output High, setup Timer1 to overflow in pulsesForLampPW msec */ 
    /* This is enabled only if the LAMP source is FAN */  
    if ( rot1.GetRotaryValue(LAMP_SOURCE) == LAMP_SYNC_TO_FAN )  {
        digitalWrite(FAN_TACH_LAMP,HIGH);        // Turn ON TACH output pulse
        OCR1A = 0;
        TIMSK1 = 1;                              // Only OVERFLOW
        TCNT1 = 65536 - pulsesForLampPW;         // Count till overflow
        TCCR1B = clockPrescaler;
    }
    
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

ISR ( TIMER1_OVF_vect ) {         
    // Overflowed pulse width count, turn OFF lamp
    digitalWrite(FAN_TACH_LAMP,LOW);
    TCCR1B = 0x00;            // Interrrupts OFF, wait for next Fan TACH pulse
}

/* 
 *  Manually generate LAMP pulse of variable RPM and PW
*/
ISR ( TIMER1_COMPA_vect ) {
    uint8_t state = digitalRead(FAN_TACH_LAMP);
    OCR1A =  state ? pulsesForLampRPM : pulsesForLampPW;
    digitalWrite(FAN_TACH_LAMP,!state);   // toggle state
}



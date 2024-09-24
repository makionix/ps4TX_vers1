/*
// Simple Arduino trasmisster
// Arduino Nano
// ELRS 2.4G TX moduel
*/
// mismatch code by Onix, done by trying to combine 2 different bases, doesnt work in current implementation:
//https://github.com/kkbin505/Arduino-Transmitter-for-ELRS/blob/main/SimpleTX/SimpleTX.ino
//https://github.com/felis/USB_Host_Shield_2.0/blob/master/examples/PS4USB/PS4USB.ino

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!
//#define GIMBAL_CALIBRATION // if not commented out, Serial.print() is active! For debugging only!!
#include <Arduino.h>
#include "config.h"
#include "crsf.h"

#include "EEPROM.h"

#include <PS4USB.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>


int Aileron_value = 0; 
int Elevator_value = 0;
int Throttle_value = 0;
int Rudder_value = 0;
int previous_throttle = 191;

USB Usb;
PS4USB PS4(&Usb); // Using PS4 controller library

bool printAngle;
uint8_t state = 0;


int loopCount = 0; // for ELRS seeting

float batteryVoltage;

int currentPktRate = 3;
int currentPower = 3;
int currentDynamic = 1;
int currentSetting = 1;


uint8_t crsfPacket[CRSF_PACKET_SIZE];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

CRSF crsfClass;



void selectSetting() {
    // startup stick commands (rate/power selection / initiate bind / turn on tx module wifi)
    // Right stick:
    // Up Left - Rate/Power setting 1 (250hz / 100mw / Dynamic)
    // Up Right - Rate/Power setting 2 (50hz / 100mw)
    // Down Left - Start TX bind (for 3.4.2 it is now possible to bind to RX easily). Power cycle after binding
    // Down Right - Start TX module wifi (for firmware update etc)

    if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) { // Elevator up + aileron left
        currentPktRate = SETTING_1_PktRate;
        currentPower = SETTING_1_Power;
        currentDynamic = SETTING_1_Dynamic;
        currentSetting = 1;
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND) { // Elevator up + aileron right
        currentPktRate = SETTING_2_PktRate;
        currentPower = SETTING_2_Power;
        currentDynamic = SETTING_2_Dynamic;
        currentSetting = 2;
    } else if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron left
        currentSetting = 3;  // Bind
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) { // Elevator down + aileron right
        currentSetting = 4;  // TX Wifi
    } else {
        currentSetting = 0;
    }
}

/*
void selectSetting() {
    // startup stick commands (rate/power selection / initiate bind / turn on tx module wifi)
    // Right stick:
    // Up Left - Rate/Power setting 1 (250hz / 100mw / Dynamic)
    // Up Right - Rate/Power setting 2 (50hz / 100mw)
    // Down Left - Start TX bind (for 3.4.2 it is now possible to bind to RX easily). Power cycle after binding
    // Down Right - Start TX module wifi (for firmware update etc)

    currentSetting = 1;

    // Update rate/pkt power dynamic settings
    currentPktRate = SETTING_1_PktRate;
    currentPower = SETTING_1_Power;
    currentDynamic = SETTING_1_Dynamic;
}
*/

void setup()
{
  
  
    // inialize rc data
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
    }
    analogReference(EXTERNAL);
    #ifdef PPMOUTPUT
    pinMode(ppmPin, OUTPUT);
    digitalWrite(ppmPin, !onState); // set the PPM signal pin to the default state (off)
    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;
    OCR1A = 100;             // compare match register, change this    //what does this mean??
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
    #endif
    #ifdef GIMBAL_CALIBRATION
        Serial.begin(115200);
        //calibrationReset();
        calStatus = true;
        Serial.println("Start Calibration"); 
    #endif


    delay(1000); // Give enough time for uplodading firmware 1 second
    
    #ifdef DEBUG
        Serial.begin(115200);

    #else
        crsfClass.begin();
    #endif
    //This needs to be here for the ps4 controller to get values
    if (Usb.Init() == -1) {
    //Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt

  rcChannels[AILERON] = CRSF_DIGITAL_CHANNEL_MIN;
  rcChannels[ELEVATOR] = CRSF_DIGITAL_CHANNEL_MIN;
  rcChannels[RUDDER] = CRSF_DIGITAL_CHANNEL_MIN;
  rcChannels[THROTTLE] = CRSF_DIGITAL_CHANNEL_MIN;
  rcChannels[AUX1] = CRSF_DIGITAL_CHANNEL_MIN;
  rcChannels[AUX2] = CRSF_DIGITAL_CHANNEL_MIN;
  rcChannels[AUX3] = CRSF_DIGITAL_CHANNEL_MIN;
  }
  //Serial.print(F("\r\nPS4 USB Library Started"));

  
    
  }
  //Serial.print(F("\r\nPS4 USB Library Started"));

  




void loop() {
  uint32_t currentMicros = micros();
  Usb.Task();

/*
  Aileron_value = map(PS4.getAnalogHat(RightHatX), 0 , 255, ADC_MIN, ADC_MAX);
  Aileron_value = constrain(Aileron_value,  ADC_MIN, ADC_MAX); 
  rcChannels[AILERON]   = map(Aileron_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 

*/







//_____________________________________________________________________________________________
  if (PS4.connected()) {
    // Roll Axis is on PS4 Right Joystick X Axis
    //rcChannels[AILERON] = map(Aileron_value, 0 , 255, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    Aileron_value = map(PS4.getAnalogHat(RightHatX), 0 , 255, ADC_MIN, ADC_MAX);
    Aileron_value = constrain(Aileron_value,  ADC_MIN, ADC_MAX); 
    rcChannels[AILERON]   = map(Aileron_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 

    // Pitch Axis on PS4 Right Joystick Y Axis 
    //rcChannels[ELEVATOR] = map(PS4.getAnalogHat(RightHatY), 0 , 255, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    Elevator_value = map(PS4.getAnalogHat(RightHatY), 0 , 255, ADC_MIN, ADC_MAX);
    Elevator_value = constrain(Elevator_value,  ADC_MIN, ADC_MAX); 
    rcChannels[ELEVATOR]   = map(Elevator_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 

    // Yaw Axis is on PS4 Left Joystick X Axis
    //rcChannels[RUDDER] = map(PS4.getAnalogHat(LeftHatX), 0 , 255, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    Rudder_value = map(PS4.getAnalogHat(LeftHatX), 0 , 255, ADC_MIN, ADC_MAX);
    Rudder_value = constrain(Rudder_value,  ADC_MIN, ADC_MAX); 
    rcChannels[RUDDER]   = map(Rudder_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 


    // Throttle is on R2 Button 
    //rcChannels[THROTTLE] = map(PS4.getAnalogButton(R2), 0 , 255, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    Throttle_value = map(PS4.getAnalogButton(R2), 0 , 255, ADC_MIN, ADC_MAX);
    Throttle_value = constrain(Throttle_value,  ADC_MIN, ADC_MAX); 
    rcChannels[THROTTLE]   = map(Throttle_value,  ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 


    

    if (PS4.getButtonClick(SQUARE)){
      // AUX3 Upper 
      rcChannels[AUX3] = CRSF_DIGITAL_CHANNEL_MAX;
    }
    if (PS4.getButtonClick(CIRCLE)){
      // AUX3 lower
      rcChannels[AUX3] = CRSF_DIGITAL_CHANNEL_MIN;
    }

    if (PS4.getButtonClick(CROSS)){
      // AUX2 Lower Bound 
      rcChannels[AUX2] = CRSF_DIGITAL_CHANNEL_MIN;
    }
    if (PS4.getButtonClick(TRIANGLE)){
      // AUX2 Upper Bound 
      rcChannels[AUX2] = CRSF_DIGITAL_CHANNEL_MAX;
    }

    if (PS4.getButtonClick(L1)){
      // L1 to Disarm 
      rcChannels[AUX1] = CRSF_DIGITAL_CHANNEL_MIN;
    }
      
    if (PS4.getButtonClick(R1)){
      // R1 is to Arm 
      rcChannels[AUX1] = CRSF_DIGITAL_CHANNEL_MAX;
    }
  }

  if (!PS4.connected()){
    rcChannels[AUX1] = CRSF_DIGITAL_CHANNEL_MIN;  // Disarm if PS4 is disconnected
  }

//_____________________________________________________________________________________________

    if (loopCount == 0) {
        // Check if sticks are held in specific position on startup (bind/wifi/packet rate select)
        selectSetting();
    }

    // Handle CRSF packets
    if (currentMicros > crsfTime) {
        #ifdef DEBUG
            Serial.print(PS4.getAnalogHat(RightHatX));
            Serial.print("AILERON: "); Serial.print(rcChannels[AILERON]);
            Serial.print(" ELEVATOR: "); Serial.print(rcChannels[ELEVATOR]);
            Serial.print(" THROTTLE: "); Serial.print(rcChannels[THROTTLE]);
            Serial.print(" RUDDER: "); Serial.print(rcChannels[RUDDER]);
            Serial.print(" AUX1: "); Serial.print(rcChannels[AUX1]);
            Serial.print(" AUX2: "); Serial.print(rcChannels[AUX2]);
            Serial.print(" AUX3: "); Serial.print(rcChannels[AUX3]);
            Serial.print(" AUX4: "); Serial.println(rcChannels[AUX4]);
        #else
            if (loopCount <= 500) { // repeat 500 packets to build connection to TX module
                // Build commond packet
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
                loopCount++;
            }

            if (loopCount > 500 && loopCount <= 505) { // repeat 5 packets to avoid bad packet, change rate setting
                // Build commond packet
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, currentPktRate);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 3) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_BIND_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 4) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_WIFI_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 505 && loopCount <= 510) { // repeat 5 packets to avoid bad packet, change TX power level
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, currentPower);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 510 && loopCount <= 515) { // repeat 5 packets to avoid bad packet, change TX dynamic power setting
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_DYNAMIC_POWER_COMMAND, currentDynamic);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else {
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
            }

        #endif
        crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
    }
}



ISR(TIMER1_COMPA_vect) { 
    static boolean state = true;

    TCNT1 = 0;

    if (state) { // start pulse
        digitalWrite(ppmPin, onState);
        OCR1A = PULSE_LENGTH * 2;
        state = false;
    } else { // end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(ppmPin, !onState);
        state = true;

        if (cur_chan_numb >= CHANNEL_NUMBER) {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PULSE_LENGTH; //
            OCR1A = (FRAME_LENGTH - calc_rest) * 2;
            calc_rest = 0;
        } else {
            OCR1A = (map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000) - PULSE_LENGTH) * 2;
            calc_rest = calc_rest + map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000);
            cur_chan_numb++;
        }
    }
}
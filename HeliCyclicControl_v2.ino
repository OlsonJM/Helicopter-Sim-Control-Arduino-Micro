/*
# Helicopter cyclic Control 
# by James Olson
# Date: 12/26/2024
#
# Arduino IDE version 2.3.4
# Arduino Micro 
# 
# Joystick library by Giuseppe version 2.1.1
# Stepper is default Arduino stepper library with IDE
#
*/

#include <Stepper.h>
#include <Joystick.h>

#define DEBUG

//MICRO I/O PINS
#define TRIM_DISABLE_OUT 0

#define TRIM_DISABLE_OUT 0
#define TRIM_RELEASE 1
#define TRIM_DISABLE 7
#define HAT_1_UP 12
#define HAT_1_DOWN 13
#define HAT_1_LEFT 14
#define HAT_1_RIGHT 15
#define RADIO 20
#define ICS 21
#define LH_BUTTON 22
#define CARGO_RELEASE 23
#define PINKY_BTN 2



//DEFINE WINDOWS JOYSTICK ObJECT
#define JOYSTICK_ID 0x03
Joystick_ HeliControls(JOYSTICK_ID,JOYSTICK_TYPE_JOYSTICK,
  6, 1,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering

//Define Joystick button numbers
#define JOY_RADIO_BTN 0
#define JOY_ICS_BTN 1
#define JOY_LH_BTN 2
#define JOY_CARGO_RELEASE_BTN 3
#define JOY_TRIM_RELEASE_BTN 4
#define JOY_PINKY_BTN 5

int lastButtonState[10] = {0,0,0,0,0,0};
int buttonMap[10] = {JOY_RADIO_BTN, JOY_ICS_BTN, JOY_LH_BTN, JOY_CARGO_RELEASE_BTN, JOY_TRIM_RELEASE_BTN, JOY_PINKY_BTN};
int joyXAXISvalue = 0;
int joyYAXISvalue = 0;
int sendJoystickStateUpdate = 0;
int joyHat1InputState = -1;
int joyHat1State = -1;
int joyHat1PreviousState = -1;
int tempInputRead = -1;

#define ANALOG_AVERAGE 10
#define ANALOG_SCAN_DELAY_MS 1
#define ANALOG_FILTER 5
#define ROLL_ANALOG_IN A0
#define PITCH_ANALOG_IN A1
#define POLL_DELAY 10     //Delay in ms between I/O scanns. Keep in mind 1ms delay between each analog poll

#define TRIM_MOTOR_SPEED 3
#define TRIM_MOTOR_STEP_SIZE 75
int trimState = -1;

int rollAnalogValue = 0;
int pitchAnalogValue = 0;

int rollPreviousAnalogValue = 0;
int pitchPreviousAnalogValue = 0;

int rollDiff = 0;
int pitchDiff = 0;

//Stepper Motors For Trim
const int stepsPerRevolution = 1600*4;  
Stepper pitchTrimMotor(stepsPerRevolution, 3, 4, 5, 6);
Stepper rollTrimMotor(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  rollTrimMotor.setSpeed(TRIM_MOTOR_SPEED);
  pitchTrimMotor.setSpeed(TRIM_MOTOR_SPEED);
  
  pinMode(TRIM_DISABLE_OUT, OUTPUT);
  pinMode(TRIM_RELEASE, INPUT_PULLUP);
  pinMode(TRIM_DISABLE, INPUT_PULLUP);
  pinMode(HAT_1_UP, INPUT_PULLUP);
  pinMode(HAT_1_DOWN, INPUT_PULLUP);
  pinMode(HAT_1_LEFT, INPUT_PULLUP);
  pinMode(HAT_1_RIGHT, INPUT_PULLUP);
  pinMode(RADIO, INPUT_PULLUP);
  pinMode(ICS, INPUT_PULLUP);
  pinMode(LH_BUTTON, INPUT_PULLUP);
  pinMode(CARGO_RELEASE, INPUT_PULLUP);
  pinMode(PINKY_BTN, INPUT_PULLUP);

  // Initialize Joystick Library
  HeliControls.begin();
  sendJoystickStateUpdate = 0;
  
}

void loop() {
  // Check Digital Inputs & analogs

    //Check if trim release button or switch is on.
    tempInputRead = !digitalRead(TRIM_DISABLE);
    if(tempInputRead == 1){
      if (trimState != 1)
      {
        digitalWrite(TRIM_DISABLE_OUT, LOW);
        trimState = 1;
        #ifdef DEBUG
          Serial.print("TRIM DISABLE OUTPUT: ");
          Serial.println(trimState);
        #endif        
      }
    }else
    {
      if (trimState != 0)
      {
        digitalWrite(TRIM_DISABLE_OUT, HIGH);
        trimState = 0;
        #ifdef DEBUG
          Serial.print("TRIM DISABLE OUTPUT: ");
          Serial.println(trimState);
        #endif
      }
    }



    //Radio Button Status
    tempInputRead = !digitalRead(TRIM_RELEASE);
    if(lastButtonState[JOY_TRIM_RELEASE_BTN] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_TRIM_RELEASE_BTN] = tempInputRead;
      HeliControls.setButton(JOY_TRIM_RELEASE_BTN, lastButtonState[JOY_TRIM_RELEASE_BTN]);
      #ifdef DEBUG
        Serial.print("TRIM RELEASE BTN ");
        Serial.println(lastButtonState[JOY_TRIM_RELEASE_BTN]);
      #endif
    }

    //Radio Button Status
    tempInputRead = !digitalRead(RADIO);
    if(lastButtonState[JOY_RADIO_BTN] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_RADIO_BTN] = tempInputRead;
      HeliControls.setButton(JOY_RADIO_BTN, lastButtonState[JOY_RADIO_BTN]);
      #ifdef DEBUG
        Serial.print("RADIO BTN ");
        Serial.println(lastButtonState[JOY_RADIO_BTN]);
      #endif
    }

    //ICS Button Status
    tempInputRead = !digitalRead(ICS);
    if(lastButtonState[JOY_ICS_BTN] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_ICS_BTN] = tempInputRead;
      HeliControls.setButton(JOY_ICS_BTN, lastButtonState[JOY_ICS_BTN]);
      #ifdef DEBUG
        Serial.print("ICS BTN ");
        Serial.println(lastButtonState[JOY_ICS_BTN]);
      #endif
    }

    //LH Button Status
    tempInputRead = !digitalRead(LH_BUTTON);
    if(lastButtonState[JOY_LH_BTN] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_LH_BTN] = tempInputRead;
      HeliControls.setButton(JOY_LH_BTN, lastButtonState[JOY_LH_BTN]);
      #ifdef DEBUG
        Serial.print("LH BTN ");
        Serial.println(lastButtonState[JOY_LH_BTN]);
      #endif
    }

    //Cargo Release Button Status
    tempInputRead = !digitalRead(CARGO_RELEASE);
    if(lastButtonState[JOY_CARGO_RELEASE_BTN] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_CARGO_RELEASE_BTN] = tempInputRead;
      HeliControls.setButton(JOY_CARGO_RELEASE_BTN, lastButtonState[JOY_CARGO_RELEASE_BTN]);
      #ifdef DEBUG
        Serial.print("CARGO RELEASE BTN ");
        Serial.println(lastButtonState[JOY_CARGO_RELEASE_BTN]);
      #endif
    }
    
    //Pinky Button Status
    tempInputRead = !digitalRead(PINKY_BTN);
    if(lastButtonState[JOY_PINKY_BTN] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_PINKY_BTN] = tempInputRead;
      HeliControls.setButton(JOY_PINKY_BTN, lastButtonState[JOY_PINKY_BTN]);
      #ifdef DEBUG
        Serial.print("PINKY BTN ");
        Serial.println(lastButtonState[JOY_PINKY_BTN]);
      #endif
    }
    
    //Check HAT switch #0 on cyclic
    if (!digitalRead(HAT_1_UP) == 1){
      joyHat1InputState = 1;
      joyHat1State = 0;
      if(trimState == 1)
        pitchTrimMotor.step(TRIM_MOTOR_STEP_SIZE);
    }else if (!digitalRead(HAT_1_RIGHT) == 1){
      joyHat1InputState = 1;
      joyHat1State = 90;
      if(trimState == 1)
        rollTrimMotor.step(-TRIM_MOTOR_STEP_SIZE);
    }else if (!digitalRead(HAT_1_DOWN) == 1){
      joyHat1InputState = 1;
      joyHat1State = 180;
      if(trimState == 1)
        pitchTrimMotor.step(-TRIM_MOTOR_STEP_SIZE);
    }else if (!digitalRead(HAT_1_LEFT) == 1){
      joyHat1InputState = 1;
      joyHat1State = 270;
      if(trimState == 1)
        rollTrimMotor.step(TRIM_MOTOR_STEP_SIZE);
    }else{
      joyHat1State = -1;
    }
    //Update Hat Switch Position if a change was identified
    if(joyHat1InputState == 1 && (joyHat1State != joyHat1PreviousState))
    {
      joyHat1PreviousState = joyHat1State;
      HeliControls.setHatSwitch(0,joyHat1State);
      joyHat1InputState = 0;
      #ifdef DEBUG
        Serial.print("HAT SWITCH: ");
        Serial.println(joyHat1State);
      #endif
    }


    //sample analog input and get average
    rollAnalogValue = 0;
    int rollValues[ANALOG_AVERAGE];
    int temp;  //Temporary hold for analog values
    for (int i=0; i < ANALOG_AVERAGE; i++)
    {
      temp = analogRead(ROLL_ANALOG_IN);
      rollValues[i] = temp;
      rollAnalogValue += temp;
      delay(ANALOG_SCAN_DELAY_MS);
    }
    rollAnalogValue /= ANALOG_AVERAGE;

    pitchAnalogValue = 0;
    int pitchValues[ANALOG_AVERAGE];
    for (int i=0; i < ANALOG_AVERAGE; i++)
    {
      temp = analogRead(PITCH_ANALOG_IN);
      pitchValues[i] = temp;
      pitchAnalogValue += temp;
      delay(ANALOG_SCAN_DELAY_MS);
    }
    pitchAnalogValue /= ANALOG_AVERAGE;


    rollDiff = rollPreviousAnalogValue - rollAnalogValue;
    pitchDiff = pitchPreviousAnalogValue - pitchAnalogValue;

    if (rollDiff > ANALOG_FILTER || rollDiff < -ANALOG_FILTER ){
       rollPreviousAnalogValue = rollAnalogValue;
       if(joyXAXISvalue != rollAnalogValue)
      {
        sendJoystickStateUpdate = 1;
        joyXAXISvalue = rollAnalogValue;
        HeliControls.setXAxis(joyXAXISvalue);
        #ifdef DEBUG
          Serial.print("ROLL AIN VALUES: ");
          for (int i=0; i < ANALOG_AVERAGE; i++)
          {
            Serial.print(rollValues[i]);
            Serial.print(" ");
          }
          Serial.print("AVERAGE: ");
          Serial.println(rollAnalogValue);
          Serial.print("JOY ROLL ANALOG: ");
          Serial.println(joyXAXISvalue);
        #endif
      }
    }

    
    if (pitchDiff > ANALOG_FILTER || pitchDiff < -ANALOG_FILTER ){
      pitchPreviousAnalogValue = pitchAnalogValue;
      if(joyYAXISvalue != pitchAnalogValue)
      {
        sendJoystickStateUpdate = 1;
        joyYAXISvalue = pitchAnalogValue;
        HeliControls.setYAxis(joyYAXISvalue);
        #ifdef DEBUG
          Serial.print("PITCH AIN VALUES: ");
          for (int i=0; i < ANALOG_AVERAGE; i++)
          {
            Serial.print(pitchValues[i]);
            Serial.print(" ");
          }
          Serial.print("AVERAGE: ");
          Serial.println(pitchAnalogValue);

          Serial.print("JOY PITCH ANALOG: ");
          Serial.println(joyYAXISvalue);
        #endif
      }
    }
    

  //update joysitck state to Windows if any changes detected
  if (sendJoystickStateUpdate == 1){
    HeliControls.sendState();
    //Reset Joystick update tag
    sendJoystickStateUpdate = 0;
  }

  delay(POLL_DELAY);

}

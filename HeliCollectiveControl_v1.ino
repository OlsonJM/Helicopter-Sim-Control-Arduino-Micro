/*
# Helicopter Collective Control 
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
//#define TRIM_RELEASE 1
#define TRIM_DISABLE 7
#define HAT_1_UP 12
#define HAT_1_DOWN 13
#define HAT_1_LEFT 15
#define HAT_1_RIGHT 14
#define HAT_2_UP 8
#define HAT_2_DOWN 9
#define HAT_2_LEFT 11
#define HAT_2_RIGHT 10
#define SPOT_ON 16
#define SPOT_STOW 1
#define ENG_1_INC 20
#define ENG_1_DEC 21
#define ENG_2_INC 22
#define ENG_2_DEC 23
#define RAD_ALT_BTN 2



//DEFINE WINDOWS JOYSTICK ObJECT
#define JOYSTICK_ID 0x04
Joystick_ HeliControls(JOYSTICK_ID,JOYSTICK_TYPE_JOYSTICK,
  7, 2,                  // Button Count, Hat Switch Count
  false, false, true,     // no X or Y, only Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering

//Define Joystick button numbers
//#define JOY_TRIM_REL_BTN 0
#define JOY_RAD_ALT_BTN 0
//#define JOY_SERVO 1
#define JOY_ENG_2_INC 1
#define JOY_ENG_2_DEC 2
#define JOY_ENG_1_INC 3
#define JOY_ENG_1_DEC 4
#define JOY_SPOT_ON 5
#define JOY_SPOT_STOW 6

int lastButtonState[9] = {0,0,0,0,0,0,0,0};
int buttonMap[9] = {JOY_RAD_ALT_BTN, JOY_ENG_2_INC, JOY_ENG_2_DEC, JOY_ENG_1_INC, JOY_ENG_1_DEC, JOY_SPOT_ON, JOY_SPOT_STOW};
int joyZAXISvalue = 0;
int sendJoystickStateUpdate = 0;
int joyHat1InputState = -1;
int joyHat1State = -1;
int joyHat1PreviousState = -1;
int joyHat2InputState = -1;
int joyHat2State = -1;
int joyHat2PreviousState = -1;
int tempInputRead = -1;

//MICRO ANALOG PIN
#define COLL_ANALOG_IN A0
#define ANALOG_AVERAGE 10
#define ANALOG_SCAN_DELAY_MS 1
#define ANALOG_FILTER 5

#define POLL_DELAY 5     //Delay in ms between I/O scanns. Keep in mind 1ms delay between each analog poll

#define TRIM_MOTOR_SPEED 3
#define TRIM_MOTOR_STEP_SIZE 75
int trimState = -1;

int collAnalogValue = 0;
int collPreviousAnalogValue = 0;

int collDiff = 0;

//Stepper Motors For Trim
const int stepsPerRevolution = 1600*4;  
Stepper collTrimMotor(stepsPerRevolution, 3, 4, 5, 6);


void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  collTrimMotor.setSpeed(TRIM_MOTOR_SPEED);
  
  pinMode(TRIM_DISABLE_OUT, OUTPUT);
  //pinMode(TRIM_RELEASE, INPUT_PULLUP);
  pinMode(TRIM_DISABLE, INPUT_PULLUP);
  pinMode(HAT_1_UP, INPUT_PULLUP);
  pinMode(HAT_1_DOWN, INPUT_PULLUP);
  pinMode(HAT_1_LEFT, INPUT_PULLUP);
  pinMode(HAT_1_RIGHT, INPUT_PULLUP);
  pinMode(HAT_2_UP, INPUT_PULLUP);
  pinMode(HAT_2_DOWN, INPUT_PULLUP);
  pinMode(HAT_2_LEFT, INPUT_PULLUP);
  pinMode(HAT_2_RIGHT, INPUT_PULLUP);
  pinMode(SPOT_ON, INPUT_PULLUP);
  pinMode(SPOT_STOW, INPUT_PULLUP);
  pinMode(ENG_1_INC, INPUT_PULLUP);
  pinMode(ENG_1_DEC, INPUT_PULLUP);
  pinMode(ENG_2_INC, INPUT_PULLUP);
  pinMode(ENG_2_DEC, INPUT_PULLUP);
  pinMode(RAD_ALT_BTN, INPUT_PULLUP);

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
    
    /*
    if(lastButtonState[JOY_SERVO] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_SERVO] = tempInputRead;
      HeliControls.setButton(JOY_SERVO, lastButtonState[JOY_SERVO]);
      #ifdef DEBUG
        Serial.print("SERVO OFF BTN ");
        Serial.println(lastButtonState[JOY_SERVO]);
      #endif
    }

    //TRIM Release Button Status
    
    tempInputRead = !digitalRead(TRIM_RELEASE);
    if(lastButtonState[JOY_TRIM_REL_BTN] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_TRIM_REL_BTN] = tempInputRead;
      HeliControls.setButton(JOY_TRIM_REL_BTN, lastButtonState[JOY_TRIM_REL_BTN]);
      #ifdef DEBUG
        Serial.print("TRIM RELEASE BTN ");
        Serial.println(lastButtonState[JOY_TRIM_REL_BTN]);
      #endif
    }
    */

    //Radar Altimeter Release Button Status
    tempInputRead = digitalRead(RAD_ALT_BTN);
    if(lastButtonState[JOY_RAD_ALT_BTN] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_RAD_ALT_BTN] = tempInputRead;
      HeliControls.setButton(JOY_RAD_ALT_BTN, lastButtonState[JOY_RAD_ALT_BTN]);
      #ifdef DEBUG
        Serial.print("RADAR ALT BTN ");
        Serial.println(lastButtonState[JOY_RAD_ALT_BTN]);
      #endif
    }

    //ENGINE 1 Increment
    tempInputRead = !digitalRead(ENG_1_INC);
    if(lastButtonState[JOY_ENG_1_INC] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_ENG_1_INC] = tempInputRead;
      HeliControls.setButton(JOY_ENG_1_INC, lastButtonState[JOY_ENG_1_INC]);
      #ifdef DEBUG
        Serial.print("ENGINE 1 INCREMENT ");
        Serial.println(lastButtonState[JOY_ENG_1_INC]);
      #endif
    }

    //ENGINE 1 Decrement
    tempInputRead = !digitalRead(ENG_1_DEC);
    if(lastButtonState[JOY_ENG_1_DEC] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_ENG_1_DEC] = tempInputRead;
      HeliControls.setButton(JOY_ENG_1_DEC, lastButtonState[JOY_ENG_1_DEC]);
      #ifdef DEBUG
        Serial.print("ENGINE 1 DECREMENT ");
        Serial.println(lastButtonState[JOY_ENG_1_DEC]);
      #endif
    }

    //ENGINE 2 Increment
    tempInputRead = !digitalRead(ENG_2_INC);
    if(lastButtonState[JOY_ENG_2_INC] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_ENG_2_INC] = tempInputRead;
      HeliControls.setButton(JOY_ENG_2_INC, lastButtonState[JOY_ENG_2_INC]);
      #ifdef DEBUG
        Serial.print("ENGINE 2 INCREMENT ");
        Serial.println(lastButtonState[JOY_ENG_2_INC]);
      #endif
    }

    //ENGINE 2 Decrement
    tempInputRead = !digitalRead(ENG_2_DEC);
    if(lastButtonState[JOY_ENG_2_DEC] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_ENG_2_DEC] = tempInputRead;
      HeliControls.setButton(JOY_ENG_2_DEC, lastButtonState[JOY_ENG_2_DEC]);
      #ifdef DEBUG
        Serial.print("ENGINE 2 DECREMENT ");
        Serial.println(lastButtonState[JOY_ENG_2_DEC]);
      #endif
    }

    //SPOT Switch - ON Position
    tempInputRead = !digitalRead(SPOT_ON);
    if(lastButtonState[JOY_SPOT_ON] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_SPOT_ON] = tempInputRead;
      HeliControls.setButton(JOY_SPOT_ON, lastButtonState[JOY_SPOT_ON]);
      #ifdef DEBUG
        Serial.print("JOY_SPOT_ON ");
        Serial.println(lastButtonState[JOY_SPOT_ON]);
      #endif
    }

    //SPOT Stow - ON Position
    tempInputRead = !digitalRead(SPOT_STOW);
    if(lastButtonState[JOY_SPOT_STOW] != tempInputRead)
    {
      sendJoystickStateUpdate = 1;
      lastButtonState[JOY_SPOT_STOW] = tempInputRead;
      HeliControls.setButton(JOY_SPOT_STOW, lastButtonState[JOY_SPOT_STOW]);
      #ifdef DEBUG
        Serial.print("JOY_SPOT_STOW ");
        Serial.println(lastButtonState[JOY_SPOT_STOW]);
      #endif
    }
    
    //Check HAT switch #1 on collective (SPOT)
    if (digitalRead(HAT_1_UP) == 0){
      joyHat1InputState = 1;
      joyHat1State = 0;
      //if(trimState == 1)
      //  pitchTrimMotor.step(TRIM_MOTOR_STEP_SIZE);
    }else if (digitalRead(HAT_1_RIGHT) == 0){
      joyHat1InputState = 1;
      joyHat1State = 90;
      //if(trimState == 1)
      //  rollTrimMotor.step(-TRIM_MOTOR_STEP_SIZE);
    }else if (digitalRead(HAT_1_DOWN) == 0){
      joyHat1InputState = 1;
      joyHat1State = 180;
      //if(trimState == 1)
      //  pitchTrimMotor.step(-TRIM_MOTOR_STEP_SIZE);
    }else if (digitalRead(HAT_1_LEFT) == 0){
      joyHat1InputState = 1;
      joyHat1State = 270;
      //if(trimState == 1)
      //  rollTrimMotor.step(TRIM_MOTOR_STEP_SIZE);
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
        Serial.print("HAT 1 SWITCH: ");
        Serial.println(joyHat1State);
      #endif
    }

    //Check HAT switch #2 on collective (RED)
    if (digitalRead(HAT_2_UP) == 0){
      joyHat2InputState = 1;
      joyHat2State = 0;
      //if(trimState == 1)
      //  collTrimMotor.step(TRIM_MOTOR_STEP_SIZE);
    }else if (digitalRead(HAT_2_RIGHT) == 0){
      joyHat2InputState = 1;
      joyHat2State = 90;
      //if(trimState == 1)
      //  rollTrimMotor.step(-TRIM_MOTOR_STEP_SIZE);
    }else if (digitalRead(HAT_2_DOWN) == 0){
      joyHat2InputState = 1;
      joyHat2State = 180;
      //if(trimState == 1)
      //  collTrimMotor.step(-TRIM_MOTOR_STEP_SIZE);
    }else if (digitalRead(HAT_2_LEFT) == 0){
      joyHat2InputState = 1;
      joyHat2State = 270;
      //if(trimState == 1)
      //  rollTrimMotor.step(TRIM_MOTOR_STEP_SIZE);
    }else{
      joyHat2State = -1;
    }
    //Update Hat Switch Position if a change was identified
    if(joyHat2InputState == 1 && (joyHat2State != joyHat2PreviousState))
    {
      joyHat2PreviousState = joyHat2State;
      HeliControls.setHatSwitch(1,joyHat2State);
      joyHat2InputState = 0;
      #ifdef DEBUG
        Serial.print("HAT 2 SWITCH: ");
        Serial.println(joyHat2State);
      #endif
    }

    //sample analog input 3 times and get average
    //rollAnalogValue = analogRead(ROLL_ANALOG_IN); //throw out first value to avoid noise on pin switch
    collAnalogValue = 0;
    int collValues[ANALOG_AVERAGE];
    int temp;  //Temporary hold for analog values
    for (int i=0; i < ANALOG_AVERAGE; i++)
    {
      temp = analogRead(COLL_ANALOG_IN);
      collValues[i] = temp;
      collAnalogValue += temp;
      delay(ANALOG_SCAN_DELAY_MS);
    }
    collAnalogValue /= ANALOG_AVERAGE;
    collDiff = collPreviousAnalogValue - collAnalogValue;

    if (collDiff > ANALOG_FILTER || collDiff < -ANALOG_FILTER ){
       collPreviousAnalogValue = collAnalogValue;
       if(joyZAXISvalue != collAnalogValue)
      {
        sendJoystickStateUpdate = 1;
        joyZAXISvalue = collAnalogValue;
        HeliControls.setZAxis(joyZAXISvalue);
        #ifdef DEBUG
          Serial.print("COLL AIN VALUES: ");
          for (int i=0; i < ANALOG_AVERAGE; i++)
          {
            Serial.print(collValues[i]);
            Serial.print(" ");
          }
          Serial.print("AVERAGE: ");
          Serial.println(collAnalogValue);
          Serial.print("JOY ROLL ANALOG: ");
          Serial.println(joyZAXISvalue);
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

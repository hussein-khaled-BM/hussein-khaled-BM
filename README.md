#include "ArduinoMotorShieldR3.h"
ArduinoMotorShieldR3 md;
#include <Wire.h> 
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX | TX

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x26,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// motor pins
int DIR_A = 12;
int PWM_A = 3;
int current = 0;     // log filtered current reading
String d1,d2,d3;
// UI inputs
int a1 = analogRead(A1);      // right POT variable
int maxPressurePOT = analogRead(A2);      // left POT variable
int maxPressurePOTconstrianed = 0;
int freqSettingCounter = 15;              // breaths per min
const int freqUpButtonPin = 2;
const int freqDownButtonPin = 6;
const int redLEDPin = A4;
const int blueLEDPin = A5;
unsigned long blueLEDonTime = 0;          // timer to turn on blue LED
unsigned long redLEDonTime = 0;          // timer to turn on blue LED
unsigned long POTblueLEDonTime = 0;       // timer for blue LED flashing with POT
int redLEDState = HIGH;         // the current state of the output pin
int blueLEDState = HIGH;         // the current state of the output pin
int buttonState = 0;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime2 = 0;  // the last time the output pin was toggled
int buttonState2 = 0;             // the current reading from the input pin
//int lastButtonState = LOW;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
boolean B=false;
int counter=15;
int volume=500;
// LEDs
int redLED = A4;
int blueLED = A5;
unsigned long pauseTimer = millis();      // stop watch for pauses
unsigned long accelTime = millis();
int mSpeed = 0;
int state =0;
int revTimeSetting = 650;       // time it takes to reverse in ms
int revSpeed = 400;             // max speed of reverse stroke
int maxFwdSpeed = -400;         // max speed of motor/acceloration limit
int currentLimit = 1000;        // current limit
int postInhaleDwell = 500;
unsigned long postExhaleDwell = 0;
unsigned long lastBreathTime = 0;
unsigned long breathPeriod = 0;
/* with no reverse acceloration
    revSpeed:    Rev time:  currentLimit:   maxFwdSpeed:
   133           800 ms     1000 (min)      -400
   133           1100 ms    1400 (max)      -400
   200           500 ms     1000 (min)      -400
   200           700 ms     1400 (max)      -400
   400           ? ms     1000 (min)      -400
   400           ? ms     1400 (max)      -400

   with reverse acceloration every
    revSpeed:    Rev time:  currentLimit:   maxFwdSpeed:
   133           ? ms     1000 (min)      -400
   133           ? ms    1400 (max)      -400
   200           750 ms     1000 (min)      -400
   200           900 ms     1400 (max)      -400

   we don't change rev speed anymore
   400           650 ms     800 (min)       -400
   400           700 ms     1000            -400
   400           ? ms       1200            -400
   400           800 ms     1400 (max)      -400
*/

void setup()
{
  md.init();
  Serial.begin(9600);
  BTSerial.begin(9600);
  Serial.println("Arduino Motor Shield R3");
  //pinMode(9, OUTPUT);           // brake pin

  // set up motor pins
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);
   lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
   lcd.setCursor(0,0);

  // set up UI pins
  pinMode(freqUpButtonPin, INPUT);
  pinMode(freqDownButtonPin, INPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  digitalWrite(freqUpButtonPin, HIGH);        // needed for the buttons to work strangely enough
  digitalWrite(freqDownButtonPin, HIGH);

  for (int i = 0; i <= 4; i++)      // flash LEDs a
  {
    digitalWrite(redLEDPin, redLEDState);   // set initial LED state
    digitalWrite(blueLEDPin, blueLEDState);   // set initial LED state
    delay(200);
    redLEDState = !redLEDState;
    blueLEDState = !blueLEDState;
  }
  digitalWrite(redLEDPin, redLEDState);     // turn back off
  digitalWrite(blueLEDPin, blueLEDState);   // turn back off
}


void loop()
{
  checkUIM();
  
  lastBreathTime = millis();
  inhale();
  postInhalePause();
  exhale();
 postExhalePause();
 
  
}



// *********** Inhale ***********
void inhale ()
{
  accelTime = millis();               // log time in ms
  mSpeed = 0;                     // clear motor speed variable
  setMotor1Speed(mSpeed);         // drive motor
  while (current < currentLimit)          // drive until current setting
  {
    if (mSpeed > maxFwdSpeed)            // acceloration
    {
      if (millis() - accelTime > 3)     // accelorate the motor by 1 every 3ms
      {
        mSpeed--;                 // - is forward direction
        accelTime = millis();
      }
    }
    setMotor1Speed(mSpeed);
  // Serial.print("forwards , ");      // don't remove these or the current readings will update so quick
   // Serial.println(current);          // thing to do: read current at regular time intervals
    checkUIM();
  }
}


// *********** post inhale pause ***********
void postInhalePause ()
{
  pauseTimer = millis();
  while (millis() - pauseTimer < postInhaleDwell)                     // dwell
  {
    int delays =millis()-pauseTimer;
    Serial.println(delays);
    setMotor1Speed(0);
    //digitalWrite(9, HIGH);          // apply brake DON'T USE
    checkUIM();
   // Serial.print("paused1 ");
    //Serial.println(current);
  }
}


// *********** exhale ***********
void exhale ()
{
  unsigned long revTimer = millis();   // log time in ms
  

  accelTime = millis();               // log time in ms for acceloration
  int revAccelSpeed = 0;                     // clear motor speed variable
  while (millis() - revTimer < revTimeSetting)           // reverse back for a set time
  {
    if (revAccelSpeed < revSpeed)            // acceloration
    {
      if (millis() - accelTime > 1)     // accelorate the motor by 2 every 3ms
      {
        revAccelSpeed++;                 // - is forward direction
        accelTime = millis();
      }
    }
    setMotor1Speed(revAccelSpeed);
    checkUIM();
    //Serial.print("reverse , ");
    //Serial.println(current);
  }
}


// *********** post exhale pause ***********
void postExhalePause ()
{
  while (millis() - lastBreathTime < breathPeriod)                     // dwell
  {
    setMotor1Speed(0);
    //digitalWrite(9, HIGH);          // apply brake DON'T USE
    checkUIM();
    //Serial.print("paused2 ");
    //Serial.println(millis() - lastBreathTime);
  }
}



// ****** drive motors ******
void setMotor1Speed(int M1speed)
{
  if (M1speed < 0) {
    M1speed = -M1speed;  // Make speed a positive quantity
    digitalWrite(DIR_A, LOW);
  }
  else {
    digitalWrite(DIR_A, HIGH);
  }
  if (M1speed > 400) M1speed = 400;  // Max PWM dutycycle
  analogWrite(PWM_A, M1speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
}

// ***** check buttons & POTs and flash LEDs *****
void checkUIM()          // check buttons and POTs
{
  current = getCurrentM1();
  
 currentLimit=700;
  revTimeSetting = map(currentLimit, 600, 800, 650, 850);      // set reversing time period
  revTimeSetting = constrain(revTimeSetting, 650, 850);         // constrain reverse time limits
 
  freqSettingCounter=15;

  freqSettingCounter = constrain(freqSettingCounter, 10, 30);       // constrain variable to 10-30 b/m
  breathPeriod = (60 / freqSettingCounter) * 1000;                  // calc time needed per breath in ms based on UI setting
  d1=String(freqSettingCounter);
  
}
void checkUI()          // check buttons and POTs
{
   if(BTSerial.available() > 0){ // Checks whether data is comming from the serial port
 state = BTSerial.read(); // Reads the data from the serial port
 }
 if (state == '0') {
 counter--;
 if(counter < 10)
 {
  counter=10;
 }
state=0;
 }
 else if (state == '1') {
 counter++;
 if(counter > 30)
 {
  counter=30;
 }
 state=0;

 }
 
  else if (state == '2') {
 volume=volume+50;
 if(volume > 800)
 {
  volume=800;
 }
 state=0;
 }
  else if (state == '3') {
 volume=volume-50;
 if(volume < 300)
 {
  volume=300;
 }
 state=0;
 }
else if (state == '4') {
  //Serial.print(counter);       //Send volume to serial monitor
  //Serial.print(" BPM");
  //Serial.print("|");
  //Serial.print(volume);          //Send volume to serial monitor
  //Serial.println(" ml");
  //Serial.println(B);
  BTSerial.print(counter);       //Send counter to mobile app using bluetooth
  BTSerial.print(" BPM");
  BTSerial.print("|");
  BTSerial.print(volume);          //Send volume to mobile app using bluetooth
  BTSerial.print(" ml");
   lcd.setCursor(3,1);
   lcd.print(volume);
    lcd.setCursor(14,1);
   lcd.print(counter);
  state=0;
 }
  else if (state == '5') {
 B=true;
 Serial.println(B);
 state=0;
 }
 else if (state == '6') {
 B=false;
 Serial.println(B);
 state=0;
 }
  //postInhaleDwell=map(analogRead(A2),50,950,0,400); // this to achieve platue pressure
  //current = getCurrentM1();
 // currentLimit = 2*volume;
 // revTimeSetting = map(currentLimit, 300, 800, 1000, 1500);      // set reversing time period
  //revTimeSetting = constrain(revTimeSetting, 1000, 1500);         // constrain reverse time limits
 // Serial.println(revTimeSetting);
  d2=String(currentLimit*2.5);
  d3=String(revTimeSetting);
  //breathPeriod = (60 /counter) * 1000;                  // calc time needed per breath in ms based on UI setting
  d1=String(counter);
  int reading = digitalRead(freqUpButtonPin);
  if (reading == LOW && lastButtonState == HIGH && millis() - lastDebounceTime > debounceDelay)     // reset the debouncing timer if state change
   {              
      if (buttonState == HIGH)
      {
        buttonState=LOW;
      }
      else {
        buttonState=HIGH;
        
      }
      
    }
   // digitalWrite(13,buttonState^B);
   /* if(B==1){
    Serial.println("Device Paused");}
    else{
    Serial.println("Device Running");}*/  
    
    lastButtonState = reading;
  
}

// **********************************************************************************************************
// BBQ Temperature controller script
// Monitors BBQ temperature and adjusts gas control knob with servo to maintain set temperature
// **********************************************************************************
// Author: Peter Glorie 2017
// **********************************************************************************

// include the library code:
#include <LiquidCrystal.h>// https://github.com/arduino-libraries/LiquidCrystal
#include <PID_v1.h>       // https://github.com/br3ttb/Arduino-PID-Library
#include <Servo.h>        // https://github.com/arduino-libraries/Servo
#include "max6675.h"      // https://github.com/adafruit/MAX6675-library


// BUTTONS
// code for controlling buttons sourced from Adafruit
// https://blog.adafruit.com/2009/10/20/example-code-for-multi-button-checker-with-debouncing/
#define DEBOUNCE 50  // button debouncer, how many ms to debounce, 5+ ms is usually plenty
byte buttons[] = {18, 19}; // the analog 0-5 pins are also known as 14-19
#define NUMBUTTONS sizeof(buttons)
// track if a button is just pressed, just released, or 'currently pressed'
volatile byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
#define holdtime 500  //Time required to hold a button before other functions are enabled
#define holdincrement 100 // time between value increments after holding button
unsigned long holdstart;  //time button hold started
byte bplus[] = {0, 1};
byte bminus[] = {1, 0};
byte bboth[] = {1, 1};


// TEMPERATURE
int ktcSO = 8;  //serial output pin
int ktcCS = 7;    //Chip select pin
int ktcCLK = 6;   // Serial clock pin
MAX6675 ktc(ktcCLK, ktcCS, ktcSO);
unsigned long temptime = 0;       //last time the temperature reading was captured
int tempint = 500;        //time between taking temperature samples
const int tempsamples = 10;     //number of temperature samples to average
int readindex = 0;              // the index of the current reading
double total = 0;                  // the running total
double average = 0;                // the average
double readings[tempsamples];      // the readings from the input
double temp;                  //start temp
double maxtemp;
double mintemp;

// PID
double PIDoutput, PIDinput;
double setval = 180; // this is the target temperature
//Specify the links and initial tuning parameters PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID myPID(&PIDinput, &PIDoutput, &setval, 20, 2, 0, DIRECT);
// these parameters have worked for me. If you need to adjust, set Ki to 0 and tune using Kp alone.
// set Ki after Kp has been tuned to the best possible.


// SERVO
int servoval;                      //servo position
Servo myservo; // create servo object to control a servo

// LCD
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//---------------------------------------------------------------------------------------------
void setup()
{
  // set up serial port
  Serial.begin(9600);

  // BUTTONS SETUP
  byte i;
  // Make input & enable pull-up resistors on switch pins
  for (i = 0; i < NUMBUTTONS; i++)
  { pinMode(buttons[i], INPUT);
    digitalWrite(buttons[i], HIGH);
  }


  // LCD SETUP
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);


  // SERVO SETUP
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  // TEMP READING SETUP
  // initialize all the readings:
  for (int i = 0; i < tempsamples; i++) {
    temp = gettemp();
  }
  maxtemp = temp;
  mintemp = temp;

  //PID SETUP
  myPID.SetOutputLimits(0, 100); //set PID output limit (0 to 100%)
  myPID.SetSampleTime(1000); // 1 second is plenty fast enough
  myPID.SetMode(AUTOMATIC); //turn the PID on


  temptime = millis();
}


//------------------------------------------------------------------------------------------------------
void loop()
{

  // Get temperature from sensor
  if ( millis() - temptime > tempint )
  {
    temp = gettemp();
    PIDinput = temp;
    temptime = millis();
    if ( temp > maxtemp ) {
      maxtemp = temp;
    }
    if ( temp < mintemp ) {
      mintemp = temp;
    }
    // Print process temp to serial
    Serial.println(temp);
  }

  // Compute PID
  myPID.Compute(); //calulate PID output

  // Drive Servo
  servoval = map(PIDoutput, 0, 100, 140, 35);     // scale PID output to suitable servo position (servo value limits between 0 and 180)
  myservo.write(servoval);                  // sets the servo position according to the scaled value

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("PV");
  lcd.print(temp, 0);
  lcd.print(" ");

  lcd.setCursor(0, 1);
  lcd.print("SV");
  lcd.print(setval, 0);
  lcd.print(" ");

  lcd.setCursor(6, 0);
  lcd.print("MX");
  lcd.print(maxtemp, 0);
  lcd.print(" ");

  lcd.setCursor(6, 1);
  lcd.print("MN");
  lcd.print(mintemp, 0);
  lcd.print(" ");

  lcd.setCursor(12, 0);
  lcd.print(PIDoutput, 0);
  lcd.print("% ");



  // Buttons
  check_switches();

  if (arraymatch(justpressed, bboth) || (arraymatch(pressed, bboth) && holdstart + holdtime < millis()) || millis() < 5000 ) {
    maxtemp = temp ;
    mintemp = temp ;
  }
  if (arraymatch(justpressed, bminus)) {
    setval = setval - 1 ;
    holdstart = millis();
  }
  if (arraymatch(justpressed, bplus)) {
    setval = setval + 1 ;
    holdstart = millis();
  }
  if (arraymatch(pressed, bminus) && holdstart + holdtime < millis()) {
    setval = setval - 1 ;
    holdstart = holdstart + holdincrement;
  }
  if (arraymatch(pressed, bplus) && (holdstart + holdtime < millis()) ) {
    setval = setval + 1 ;
    holdstart = holdstart + holdincrement;
  }

  for (byte i = 0; i < NUMBUTTONS; i++) {

    if (justpressed[i]) {
      justpressed[i] = 0;
//      Serial.print(i, DEC);
//      Serial.println(" Just pressed");
      // remember, check_switches() will CLEAR the 'just pressed' flag
    }
    if (justreleased[i]) {
      justreleased[i] = 0;
      //          Serial.print(i, DEC);
      //          Serial.println(" Just released");
      // remember, check_switches() will CLEAR the 'just pressed' flag
    }
    if (pressed[i]) {
      //          Serial.print(i, DEC);
      //          Serial.println(" pressed");
      // is the button pressed down at this moment
    }
  }
}

int gettemp()
{
  // subtract the last reading:
  total = total - readings[readindex];
  // read from the sensor:
  readings[readindex] = ktc.readCelsius();
  // add the reading to the total:
  total = total + readings[readindex];
  // advance to the next position in the array:
  readindex = readindex + 1;
  // if we're at the end of the array...
  if (readindex >= tempsamples)
  {
    // ...wrap around to the beginning:
    readindex = 0;
  }

  // calculate the average:
  average = total / tempsamples;
  Serial.print("Temp: ");
  Serial.print(average);
  Serial.print("   Set: ");
  Serial.print(setval);
  Serial.print("   Out: ");
  Serial.print(PIDoutput, 0);
  Serial.println("%  ");

  return average;
}

void check_switches()
{
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];
  static long lasttime;
  byte index;

  if (millis() < lasttime) { // we wrapped around, lets just try again
    lasttime = millis();
  }

  if ((lasttime + DEBOUNCE) > millis()) {
    // not enough time has passed to debounce
    return;
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();

  for (index = 0; index < NUMBUTTONS; index++) {
    currentstate[index] = digitalRead(buttons[index]);   // read the button

    /*
      Serial.print(index, DEC);
      Serial.print(": cstate=");
      Serial.print(currentstate[index], DEC);
      Serial.print(", pstate=");
      Serial.print(previousstate[index], DEC);
      Serial.print(", press=");
    */

    if (currentstate[index] == previousstate[index]) {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
        // just pressed
        justpressed[index] = 1;
      }
      else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) {
        // just released
        justreleased[index] = 1;
      }
      pressed[index] = !currentstate[index];  // remember, digital HIGH means NOT pressed
    }
    //Serial.println(pressed[index], DEC);
    previousstate[index] = currentstate[index];   // keep a running tally of the buttons
  }
}

byte arraymatch(byte b1[], byte b2[]) {
  if (sizeof(b1) != sizeof(b2)) {
    return 0;
  }
  for (int i = 0 ; i < sizeof(b1); i++) {
    if (b1[i] != b2[i]) {
      return 0;
    }
  }
  return 1;

}



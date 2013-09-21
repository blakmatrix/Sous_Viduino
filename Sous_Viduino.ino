//-------------------------------------------------------------------
//
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//------------------------------------------------------------------
// Modified by blakmatrix for Serial control in absence of buttons and with 20x4 LCD via i2c
#include <String.h>
// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the Adafruit RGB/LCD Shield
#include <Wire.h>


#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR    0x3F  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
// wiring info:
// SDA - A4
// SCL - A5

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 7

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 2
// blakmatrix: Not necessary with my setup
/*
#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 4
*/

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************


unsigned long lastInput = 0; // last button press
int autotune_count = 0;

byte degree[8] = // define the degree symbol 
{ 
 B00110, 
 B01001, 
 B01001, 
 B00110, 
 B00000,
 B00000, 
 B00000, 
 B00000 
}; 

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);

   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start

   lcd.begin (20,4);
  // Switch on the backlight
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home ();                   // go home
  
  lcd.createChar(1, degree); // create degree symbol from the binary
  
   lcd.print(F("    Adafruit"));
   lcd.setCursor(0, 1);
   lcd.print(F("   Sous Vide!"));

   // Start up the DS18B20 One Wire Temperature Sensor

   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Error"));
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);

   delay(3000);  // Splash screen

   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   
   lcd.clear();

   switch (opState)
   {
   case OFF:
      Off();
      break;
   case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
   case TUNE_P:
      TuneP();
      break;
   case TUNE_I:
      TuneI();
      break;
   case TUNE_D:
      TuneD();
      break;
   }
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
   myPID.SetMode(MANUAL);
   lcd.setBacklight(0);
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.print(F("    Adafruit"));
   lcd.setCursor(0, 1);
   lcd.print(F("   Sous Vide!"));

   PrintOffControlLoop();
   while(ReadSerialAny() == -1){}
   
   
   // Prepare to transition to the RUN state
   sensors.requestTemperatures(); // Start an asynchronous temperature reading

   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************
void Tune_Sp()
{
   lcd.setBacklight(HIGH);
   lcd.print(F("Set Temperature:"));
   int buttons = 0;
   
   while(true)
   {
      if (buttons != -1) {PrintTuneSp();}
      buttons = ReadSerial();

      float increment = 0.1;
      if (buttons == 0)
      {
         opState = TUNE_P;
         return;
      }
      if (buttons == 1)
      {
         opState = RUN;
         return;
      }
      if (buttons == 2)
      {
         Setpoint += increment;
         delay(200);
      }
      if (buttons == 3)
      {
        Setpoint -= increment;
        delay(200);
      }
      if (buttons == 4)
      {
         Setpoint += 10.0;
         delay(200);
      }
      if (buttons == 5)
      {
         Setpoint -= 5.0;
         delay(200);
      }
    
      if ((millis() - lastInput) > 8000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Setpoint);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneP()
{
   lcd.setBacklight(HIGH);
   lcd.print(F("Set Kp"));

   int buttons = 0;
   while(true)
   {
      if (buttons != -1) {PrintTuneP();}
      buttons = ReadSerial();

      float increment = 1.0;
      if (buttons == 0)
      {
         opState = TUNE_I;
         return;
      }
      if (buttons == 1)
      {
         opState = SETP;
         return;
      }
      if (buttons == 2)
      {
         Kp += increment;
         delay(200);
      }
      if (buttons = 3)
      {
         Kp -= increment;
         delay(200);
      }
      if (buttons == 4)
      {
         Kp += 10.0;
      }
      if (buttons == 5)
      {
         Kp -= 5.0;
      }
      if ((millis() - lastInput) > 8000)  // return to RUN after 8 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
   //lcd.setBacklight(TEAL);
   lcd.setBacklight(HIGH);
   lcd.print(F("Set Ki"));

   int buttons = 0;
   
   while(true)
   {
      if (buttons != -1) {PrintTuneI();}
      buttons = ReadSerial();

      float increment = 0.01;
      if (buttons == 0)
      {
         opState = TUNE_D;
         return;
      }
      if (buttons == 1)
      {
         opState = TUNE_P;
         return;
      }
      if (buttons == 2)
      {
         Ki += increment;
         delay(200);
      }
      if (buttons == 3)
      {
         Ki -= increment;
         delay(200);
      }
      if (buttons == 4)
      {
         Ki += 10.0;
      }
      if (buttons == 5)
      {
         Ki -= 5.0;
      }
      if ((millis() - lastInput) > 8000)  // return to RUN after 8 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneD()
{
   //lcd.setBacklight(TEAL);
   lcd.setBacklight(HIGH);
   lcd.print(F("Set Kd"));

   int buttons = 0;
   while(true)
   {
      if (buttons != -1) {PrintTuneD();}
      buttons = ReadSerial();
      float increment = 0.01;
      if (buttons == 0)
      {
        opState = RUN;
         return;
      }
      if (buttons == 1)
      {
         opState = TUNE_I;
         return;
      }
      if (buttons == 2)
      {
         Kd += increment;
         delay(200);
      }
      if (buttons == 3)
      {
         Kd -= increment;
         delay(200);
      }
      if (buttons == 4)
      {
         Kd += 10.0;
      }
      if (buttons == 5)
      {
         Kd -= 5.0;
      }
      if ((millis() - lastInput) > 8000)  // return to RUN after 8 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
   // set up the LCD's number of rows and columns: 
   lcd.print(F("Sp: "));
   lcd.print(Setpoint);
   lcd.write(1);
   lcd.print(F("C/"));
   double f = ((Setpoint*9)/5) +32.0;// Some  Fahrenheit 
   lcd.print(f);
   lcd.write(1);
   lcd.print(F("F"));

   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   int buttons = 0;
   PrintRun();
   while(true)
   {
      setBacklight();  // set backlight based on state

      
      buttons = ReadSerialInterrupt();
      if (buttons == 0
         && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
      {
         StartAutoTune();
      }
      else if (buttons == 1)
      {
        opState = SETP;
        return;
      }
      else if (buttons ==2)
      {
        opState = OFF;
        return;
      }
      
      DoControl();
      
      lcd.setCursor(0,1);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C : "));
      
      double f2 = ((Input*9)/5) +32.0;// Some more Fahrenheit 
       lcd.setCursor(0,2); // 3rd row
      lcd.print(f2);
      lcd.write(1);
      lcd.print(F("F"));
      
      float pct = map(Output, 0, WindowSize, 0, 1000);
      lcd.setCursor(10,1);
      lcd.print(F("      "));
      lcd.setCursor(10,1);
      lcd.print(pct/10);
      //lcd.print(Output);
      lcd.print("%");

      lcd.setCursor(15,0);
      if (tuning)
      {
        lcd.print("T");
      }
      else
      {
        lcd.print(" ");
      }
      
      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        Serial.print(Input);
        Serial.print(",");
        Serial.println(Output);
      }

      delay(100);
   }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
   if (tuning)
   {
     // lcd.setBacklight(VIOLET); // Tuning Mode
     lcd.setBacklight(HIGH);
     lcd.setCursor(0,3);
     lcd.print(F("<< Tuning Mode >>"));
   }
   else if (abs(Input - Setpoint) > 1.0)  
   {
      //lcd.setBacklight(RED);  // High Alarm - off by more than 1 degree
      lcd.setBacklight(HIGH);
      lcd.setCursor(0,3);
     lcd.print(F("    > 1.0"));
     lcd.write(1);
     lcd.print(F("C OFF"));
   }
   else if (abs(Input - Setpoint) > 0.2)  
   {
      //lcd.setBacklight(YELLOW);  // Low Alarm - off by more than 0.2 degrees
      lcd.setBacklight(HIGH);
      lcd.setCursor(0,3);
     lcd.print(F("    > 0.2"));
     lcd.write(1);
     lcd.print(F("C OFF"));
   }
   else
   {
      //lcd.setBacklight(WHITE);  // We're on target!
      lcd.setBacklight(HIGH);
      lcd.setCursor(0,3);
     lcd.print(F("  << ON TARGET >>   "));
   }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}



// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}


// ************************************************
// Serial Output functions
// ************************************************


void PrintOffControlLoop(){
  Serial.println("Welcome to Sous Vide!");
  Serial.println("----------------------------------------");
  Serial.print("Press any key to start: ");
}

void PrintTuneSp(){
  Serial.println("----------------------------------------");
  Serial.println("Setpoint Entry State loop");
  Serial.println("Options:");
  Serial.println("    0) -> TUNE_P");
  Serial.println("    1) <- RUN");
  Serial.println("    2) increment up");
  Serial.println("    3) increment down");
  Serial.println("    4) increment +10");
  Serial.println("    5) increment -5");
  Serial.print("Enter Choice: ");
}

void PrintTuneP(){
  Serial.println("----------------------------------------");
  Serial.println("Proportional Entry State loop");
  Serial.println("Options:");
  Serial.println("    0) -> TUNE_I");
  Serial.println("    1) <- SETP");
  Serial.println("    2) increment up");
  Serial.println("    3) increment down");
  Serial.println("    4) increment +10");
  Serial.println("    5) increment -5");
  Serial.print("Enter Choice: ");
}

void  PrintTuneI(){
  Serial.println("----------------------------------------");
  Serial.println("Integral Entry State loop");
  Serial.println("Options:");
  Serial.println("    0) -> TUNE_D");
  Serial.println("    1) <- TUNE_P");
  Serial.println("    2) increment up");
  Serial.println("    3) increment down");
  Serial.println("    4) increment +10");
  Serial.println("    5) increment -5");
  Serial.print("Enter Choice: ");
}
void PrintTuneD(){
  Serial.println("----------------------------------------");
  Serial.println("Derivative Entry State loop");
  Serial.println("Options:");
  Serial.println("    0) -> RUN");
  Serial.println("    1) <- TUNE_I");
  Serial.println("    2) increment up");
  Serial.println("    3) increment down");
  Serial.println("    4) increment +10");
  Serial.println("    5) increment -5");
  Serial.print("Enter Choice: ");
}

void PrintRun(){
  Serial.println("----------------------------------------");
  Serial.println("PID Control loop");
  Serial.println("Options:");
  Serial.println("    0) AutoTune");
  Serial.println("    1) SetPoint");
  Serial.println("    2) OFF");
  Serial.print("Enter Choice: ");
}

int ReadSerial(){
  int incomingInt = -1;
  while(incomingInt == -1 && !((millis() - lastInput) > 7000)){// stop trying to read if we've waited to long
    if(Serial.available() > 0) {
      incomingInt = serialReadInt();
    }
  }

  // say what you got:
  if(incomingInt != -1){
    Serial.println(incomingInt, DEC);
    lastInput = millis();// register on valid input
  }
  return incomingInt;
}

int ReadSerialAny(){// hack
  int incomingInt = -1;
  while(incomingInt == -1 && !((millis() - lastInput) > 7000)){// stop trying to read if we've waited to long
    if(Serial.available() > 0) {
      incomingInt = serialReadInt();
    }
  }

  // say what you got:
  if(incomingInt != -1){
    Serial.println(incomingInt, DEC);
  }
  lastInput = millis();
  return incomingInt;
}

int ReadSerialInterrupt(){// more hacks
  int incomingInt = 0;
    if(Serial.available() > 0) {
      incomingInt = serialReadInt();
      lastInput = millis();
    }
  ++autotune_count ;
  if(autotune_count%97 == 0){ // bring back up the menu
    PrintRun();
    autotune_count=0;
  }
  return incomingInt;
}

int serialReadInt()
{
   // 12 is the maximum length of a decimal representation of a 32-bit integer,
  // including space for a leading minus sign and terminating null byte
  char intBuffer[12];
  String intData = "";// too tired to care anymore
  int delimiter = (int) '\n';
  byte bytesread = 0;

  while (Serial.available() > 0) {
    delay(5);                              // Delay for terminal to finish transmitted
                                                // 5mS work great for 9600 baud (increase this number for slower baud)
    bytesread = 0;

      int ch = Serial.read();
      if (ch == -1) {
        return -1;
      }
      else if (ch == delimiter) {
        break;
      }
      else {
        intData += (char) ch;
        bytesread++;                                // ready to read next digit
      }
    
  }

    // Copy read data into a char array for use by atoi
    // Include room for the null terminator
    int intLength = intData.length() + 1;
    intData.toCharArray(intBuffer, intLength);

    // Convert ASCII-encoded integer to an int
    int i = atoi(intBuffer);
    return i;
}
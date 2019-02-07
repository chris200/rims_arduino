#include <EEPROM.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>


DeviceAddress tempProbeAddress;
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// temperature sensor
#define ONE_WIRE_BUS 2
float Kp;
float Ki;
float Kd;

struct KObject {
  float targetTempfield;
  float Kpfield;
  float Kifield;
  float Kdfield;
};

#define SAMPLE_DELAY 2000
// PID header
#define PIN_INPUT 2
#define RELAY_OUT_PIN  6



// MIN MAX RANGE
#define MIN_TARGET_TEMP 50
#define MAX_TARGET_TEMP 200

OneWire  oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//TEMP VARIABLES
double actualTemp = 0;
double targetTemp = 0;
double Setpoint, Input;
double Output = 0;
double NewOutput = 0;
int Kvalues_addr = 10;

double targettemp = 0;
unsigned long tcurrent = 0;

int state;
String relay_state;
bool isNewSample = false;


// Serial Event Variables
String inputString = "";
bool stringComplete = false;
char inChar;
String command;
String commandValue;

unsigned long tGetTemperatureSample  = 0;
int WindowSize = 10000;
unsigned long windowStartTime;
PID myPID(&actualTemp, &NewOutput, &targettemp, Kp, Ki, Kd, DIRECT);






void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  sensors.begin();
  delay(1000);
  sensors.getAddress(tempProbeAddress, 0);
  delay(1000);
  sensors.requestTemperaturesByIndex(0); // Send the command to get temperatures
  delay(1000);
  actualTemp =  sensors.getTempF(tempProbeAddress);
  pinMode(RELAY_OUT_PIN, OUTPUT); 
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Temperature");
  lcd.setCursor(0, 1);
  lcd.print("Controller");

  lcd.clear();

  windowStartTime = millis();

  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);
  
      KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;
    myPID.SetTunings(Kp, Ki, Kd);

}




void loop() {
  tcurrent = millis();
  GetTempSample();

  //Serial.println();
  lcd_current_temp();
  lcd_targetTemp();
  lcd_output();
  lcd_relay_state();

  listenTargeTemp();
  // put your main code here, to run repeatedly:
  if (stringComplete) {

    parseInputString();
    inputString = "";
    stringComplete = false;
  }
  pidfunction();
}





void listenTargeTemp() {
  if (Serial.available()) {
    inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }

  }

}

void parseInputString() {
  int colonLoc = inputString.indexOf(":");
  if (colonLoc >= 0) {
    command = inputString.substring(0, colonLoc);
    commandValue = inputString.substring(colonLoc + 1, inputString.length());

  }

  serialCommand();

}

void serialCommand() {
  
  if (command=="R_AT"){
      Serial.print("AT:");
    Serial.println(actualTemp);
  }
  if (command == "W_TT") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;



    KObject writeKvalues{
      commandValue.toFloat(),
      Kp,
      Ki,
      Kd / 1000
    };
    EEPROM.put(Kvalues_addr, writeKvalues);
    Serial.print("TT:");
    Serial.println(commandValue.toFloat());
    myPID.SetTunings(Kp, Ki, Kd);

  }
  if (command == "W_KP") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;



    KObject writeKvalues{
      targettemp,
      commandValue.toFloat(),
      Ki,
      Kd * 1000
    };
    EEPROM.put(Kvalues_addr, writeKvalues);
    Serial.print("KP:");
    Serial.println(commandValue.toFloat());
    myPID.SetTunings(Kp, Ki, Kd);
  }

  if (command == "W_KI") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;



    KObject writeKvalues{
      targettemp,
      Kp,
      commandValue.toFloat(),
      Kd * 1000
    };
    EEPROM.put(Kvalues_addr, writeKvalues);
    Serial.print("KI");
    Serial.println(commandValue.toFloat());
    myPID.SetTunings(Kp, Ki, Kd);

  }
  if (command == "W_KD") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;

    KObject writeKvalues{
      targettemp,
      Kp,
      Ki,
      commandValue.toFloat()
    };
    EEPROM.put(Kvalues_addr, writeKvalues);
    Serial.print("KD:");
    Serial.println(commandValue.toFloat());
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (command == "R_TT") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;
    //    EEPROM.put(Kvalues_addr,writeKvalues);

    Serial.print("TT:");
    Serial.println(targettemp);
    //Serial.print("KP:");
    //Serial.println(Kp);
    //Serial.print("KI:");
    //Serial.println(Ki);
    //Serial.print("KD:");
    //Serial.println(Kd);
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (command == "R_KP") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;
    //    EEPROM.put(Kvalues_addr,writeKvalues);

    //Serial.print("TT:");
    //Serial.println(targettemp);
    Serial.print("KP:");
    Serial.println(Kp);
    //Serial.print("KI:");
    //Serial.println(Ki);
    //Serial.print("KD:");
    //Serial.println(Kd);
    myPID.SetTunings(Kp, Ki, Kd);
  }
  
  if (command == "R_KI") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;
    //    EEPROM.put(Kvalues_addr,writeKvalues);

    //Serial.print("TT:");
    //Serial.println(targettemp);
    Serial.print("KI:");
    Serial.println(Ki);
    //Serial.print("KI:");
    //Serial.println(Ki);
    //Serial.print("KD:");
    //Serial.println(Kd);
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (command == "R_KD") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    targettemp = readKvalues.targetTempfield;
    Kp = readKvalues.Kpfield;
    Ki = readKvalues.Kifield;
    Kd = readKvalues.Kdfield;
    //    EEPROM.put(Kvalues_addr,writeKvalues);

    //Serial.print("TT:");
    //Serial.println(targettemp);
    Serial.print("KD:");
    Serial.println(Kd);
    //Serial.print("KI:");
    //Serial.println(Ki);
    //Serial.print("KD:");
    //Serial.println(Kd);
    myPID.SetTunings(Kp, Ki, Kd);
  }
  
    if (command == "R_RS") {

    KObject readKvalues;

    EEPROM.get(Kvalues_addr, readKvalues);

    
    
    Serial.print("RS:");
    Serial.println(state);

  }
  
}

void GetTempSample()
{

  if ( (long) (tcurrent - tGetTemperatureSample) >= 0)
  {
    actualTemp = getTemperature();
    //Serial.print("AT:");
    //Serial.println(actualTemp);
    //Serial.print(", ");
    //Serial.print(Output);
    //Serial.print(", ");
    //Serial.print(tPowerOnLength/100);
    //Serial.println();
    //Serial.print(opState);
    // Serial.println();
  }
  else
  {
    isNewSample = false;

  }

}




float getTemperature()
{
  // plan next measurement
  tGetTemperatureSample = millis() + SAMPLE_DELAY;
  sensors.requestTemperaturesByIndex(0); // Send the command to get temperatures
  //temp = temp * 1.8 + 32.0;
  //return temp;
  return sensors.getTempF(tempProbeAddress);
}

void lcd_current_temp()
{
  lcd.setCursor(0, 0);
  lcd.print("AT");
  lcd.setCursor(0, 1);
  lcd.print(actualTemp, 1);
}

void lcd_targetTemp()
{
  lcd.setCursor(6, 0);
  lcd.print("TT");
  lcd.setCursor(6, 1);
  lcd.print(targettemp, 1);
}
void lcd_state()
{
  lcd.setCursor(10, 1);
  lcd.print(state);
}


void  lcd_output()
{
  //if (Output != NewOutput)
  {
    //lcd.setCursor(12, 0);
    //lcd.print("OP");
    lcd.setCursor(12, 1);
    lcd.print("      ");
    lcd.setCursor(12, 1);
    lcd.print(Output, 1);
    Output = NewOutput;
  }
}
void lcd_relay_state()
{
  //Serial.print(state);
  if (state == 0)
  {
    lcd.setCursor(12, 0);
    relay_state = " ON";
    
  }
  else
  {
    lcd.setCursor(12, 0);
    relay_state = "OFF";
   
  }
  lcd.print(relay_state);
}






void pidfunction() {
  //Input = actualTemp;
 myPID.Compute();

  //Serial.print("Output");
  //Serial.println(Output);

 
  //Serial.print(" AT");
  //Serial.print(actualTemp);
  // Serial.print(" OP ");
  // Serial.print(NewOutput);
  // Serial.print(" TT ");
  // Serial.print(targettemp);
  // Serial.print(" mode ");
  // Serial.print(myPID.GetMode());
  // Serial.print(" P ");
  // Serial.print(myPID.GetKp());
  // Serial.print(" I ");
  // Serial.print(myPID.GetKi());
  // Serial.print(" D ");
  // Serial.print(myPID.GetKd());
  // Serial.print(" ST ");
  // Serial.println(state);



  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (NewOutput < now - windowStartTime)
  {
    digitalWrite(6, HIGH);
    state = 1;
  }
  else {
    digitalWrite(6, LOW);
    state = 0;
  }
}
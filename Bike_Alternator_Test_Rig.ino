#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
Adafruit_MCP4725 dac;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// Hall sensor input variables
//const int hallTimeMin = 1200;              // The fastest possible motor speed, in micros() between sensor state changes
// To keep the math from going haywire, we limit the speed controller to a logarithmic boundary of 10:1
//const int hallTimeMax = hallTimeMin * 10;  // The slowest " " " before we just stop it entirely
volatile int hallSenseIndex = 0;           // Counts how many times the hall sensor has changed state

// Setpoint input variables
const int setpointPin = 0;  // The setpoint is on analog pin 0

// Converted speed input and setpoint variables
float speedMax = 30000.0;    // Max speed in MPH, multiplied by 1000 to reduce floating point math
float speedMin = 3000.0;     // Min speed in MPH, multiplied by 1000 to reduce floating point math
float hallSpeed = 0.0;               // The 
long int hallSenseNow = 0;
long int hallSenseLast = 0;
long int hallSenseTimes[3] = {0,0,0};
long int errorNow = 0;
long int errorPrevious = 0;
float setpointSpeed;           // The setpoint value after we map() it to speed boundaries

// DAC output variables
const int dacOutMin = 1650;   // The lowest usable DAC output value
const int dacOutMax = 4095;   // The highest usable DAC output value
      int dacOutput = dacOutMin;          // The value sent to the DAC

// PID compute parameters
const float gainProportional = 0.005;  // How much to multiply the instataneous error between the input and setpoint
const float gainIntegral = 0.00002;      // How much to multiply the time-aggregated error between the input and the setpoint
const float gainDerivative = 0.0005;    // How much to multiply the change in error frm the previous reading
float errorProportional = 0.0;
float errorIntegral = 0.0;
float errorIntegralMax = speedMax/10000.0;
float errorIntegralMin = speedMax/10000.0 * -1.0;
float errorDerivative = 0.0;
const int computePidInterval = 111;  // How often to update the DAC in millis() (should be an even division of one second, to avoid floating point math)
long  int computePidLast = 0;        // When the setpoint was last read
float outputAdjustmentFactor = 0.0;

// Power measurement
float altVoltage = 0.0;  // Output voltage of the alternator
float altCurrent = 0.0;  // Output current of the alternator
long int measurePowerLast = 0;
const int measurePowerInterval = 37;

// Display variables
#define DEBUG                                  // Turns on Serial output
long  int updateDisplayLast = 0;
const int updateDisplayInterval = 243;

void setup() {
  ina219.begin();
  lcd.begin(16, 2);
  dac.begin(0x60);
  dac.setVoltage(dacOutput, true);
  attachInterrupt(0,hallSenseISR,CHANGE);
  TWBR = ((F_CPU / 400000L) - 16) / 2;
#ifdef DEBUG
  Serial.begin(115200);
#endif
}

void loop() {
  if ( millis() > computePidLast + computePidInterval ){
    computePidLast = millis();
    readSetpoint();
    calculateSpeed();
    setDacOutput();
  }
  if ( millis() > measurePowerLast + measurePowerInterval ){
    measurePowerLast = millis();
    measurePower();
  }
  if ( millis() > updateDisplayLast + updateDisplayInterval ){
    updateDisplayLast = millis();
    updateDisplay();
  }
}

void hallSenseISR() {
  hallSenseNow=micros();
  hallSenseTimes[hallSenseIndex]=hallSenseNow-hallSenseLast;
  hallSenseLast=hallSenseNow;
  hallSenseIndex ++;
  if ( hallSenseIndex > 2 ) hallSenseIndex=0;
}

void readSetpoint() {
  int setpointRaw = analogRead(0);
  if ( setpointRaw < 2 ) {
    setpointSpeed = 0;
    return;
  }
  setpointSpeed = map(setpointRaw,2,1023,speedMin,speedMax);
}

void calculateSpeed() {
  int hallTimeAvg = (hallSenseTimes[0] + hallSenseTimes[1] + hallSenseTimes[2]) / 3;
  // 1 count is: 0.0625 Rev / hallTimeAvg uS
  // 16 counts/rev, 24.7400 inches/rev, 3600 s/h, 63360 in/mi
  // Offline dimensional analysis gives us the following to convert to thousands of MPH
  hallSpeed = 87855113.64 / hallTimeAvg;
}

void setDacOutput() {
  if ( setpointSpeed == 0 ) {
    dac.setVoltage(0, false);
  }
  else {
    errorNow = setpointSpeed - hallSpeed;                           // Difference between setpoint and actual
    errorProportional = errorNow * gainProportional;                // Calculate the proportional adjustment value in terms of the input and setpoint units
    errorIntegral = errorIntegral + (errorNow * gainIntegral);      // Calculate the integral adjustment value " " "
    errorDerivative = (errorPrevious - errorNow) * gainDerivative;  // Calculate the derivative adjustment value " " "
    
    if ( errorIntegral > errorIntegralMax ) errorIntegral = errorIntegralMax;      // Limit the integral to prevent wind-up
    else if ( errorIntegral < errorIntegralMin ) errorIntegral = errorIntegralMin;
  
    errorPrevious = errorNow;                                                      // Remember last measurement
  
    outputAdjustmentFactor = errorProportional + errorIntegral + errorDerivative;  // Scale the output adjustment to a unit-agnostic factor
    dacOutput = dacOutput + outputAdjustmentFactor;                                // Set the output value with the adjustment factor
  
    if ( dacOutput < dacOutMin ) dacOutput = dacOutMin;                            // Limit the output to a valid range (Min)
    else if ( dacOutput > dacOutMax ) dacOutput = dacOutMax;                       // (Max)
    
    dac.setVoltage(dacOutput, false);                                              // Update the output DAC
  }
}

void measurePower(){
  altVoltage = ina219.getBusVoltage_V();
  altCurrent = ina219.getCurrent_mA();
}

void updateDisplay() {
#ifdef DEBUG
  Serial.print("E: ");
  if ( errorNow > 0 ) Serial.print(" ");
  Serial.print(errorNow);
  Serial.print("  Speed: ");
  Serial.print(hallSpeed,3);
  Serial.print("  Setpoint: ");
  Serial.print(setpointSpeed,3);
  Serial.print("\tFactor: ");
  if ( outputAdjustmentFactor > 0 ) Serial.print(" ");
  Serial.print(outputAdjustmentFactor,3);
  Serial.print("\tP: ");
  Serial.print(errorProportional,3);
  Serial.print("\tI: ");
  Serial.print(errorIntegral,3);
  Serial.print("\tD: ");
  Serial.print(errorDerivative,3);
  Serial.println("");
#endif
  lcd.setCursor(0, 0);
  lcd.print("S: ");
  if ( setpointSpeed/1000 < 10 ) lcd.print(" ");
  lcd.print(float(setpointSpeed)/1000.0,1);
  lcd.print(" ");
  lcd.setCursor(8, 0);
  lcd.print("E:  ");
  if ( altVoltage < 10 ) lcd.print(" ");
  lcd.print(altVoltage,1);
  lcd.print(" ");
  lcd.setCursor(0, 1);
  lcd.print("M: ");
  if ( hallSpeed/1000 < 10 ) lcd.print(" ");
  lcd.print(float(hallSpeed)/1000.0,1);
  lcd.print(" ");
  lcd.setCursor(8, 1);
  lcd.print("I:");
  if ( altCurrent < 1000 ) lcd.print(" ");
  if ( altCurrent < 100 ) lcd.print(" ");
  if ( altCurrent < 10 ) lcd.print(" ");
  lcd.print(altCurrent,1);
  lcd.print(" ");
}

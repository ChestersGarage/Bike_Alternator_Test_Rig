#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
Adafruit_MCP4725 dac;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// Hall sensor input variables
const int hallTimeMin = 1200;              // The fastest possible motor speed, in micros() between sensor state changes
// To keep the math from going haywire, we limit the speed controller to a logarithmic boundary of 10:1
const int hallTimeMax = hallTimeMin * 10;  // The slowest " " " before we just stop it entirely
volatile int hallSenseCount = 0;           // Counts how many times the hall sensor has changed state

// Setpoint input variables
const int setpointPin = 0;  // The setpoint is on analog pin 0

// Converted speed input and setpoint variables
unsigned int speedMax = 30000;    // Max speed in MPH, multiplied by 1000 to reduce floating point math
unsigned int speedMin = 3000;     // Min speed in MPH, multiplied by 1000 to reduce floating point math
long int hallSpeed;               // The 
long int errorNow;
long int errorPrevious = 0;
long int setpointSpeed;           // The setpoint value after we map() it to speed boundaries

// DAC output variables
const int dacOutMin = 1650;   // The lowest usable DAC output value
const int dacOutMax = 4095;   // The highest usable DAC output value
      int dacOutput;          // The value sent to the DAC

// PID compute parameters
const float gainProportional = 0.01;  // How much to multiply the instataneous error between the input and setpoint
const float gainIntegral = 0.001;      // How much to multiply the time-aggregated error between the input and the setpoint
const float gainDerivative = 0.0025;    // How much to multiply the change in error frm the previous reading
float errorProportional = 0;
float errorIntegral = 0;
float errorIntegralMax = speedMax/10;
float errorIntegralMin = float(speedMax/10.0) * -1.0;
float errorDerivative = 0;
const int computePidInterval = 49;  // How often to update the DAC in millis() (should be an even division of one second, to avoid floating point math)
long  int computePidLast = 0;        // When the setpoint was last read
float outputAdjustmentFactor;

// Power measurement
float altVoltage = 0.0;  // Output voltage of the alternator
float altCurrent = 0.0;  // Output current of the alternator
long int measurePowerLast = 0;
const int measurePowerInterval = 37;

// Display variables
//#define DEBUG                                  // Turns on Serial output
long  int updateDisplayLast = 0;
const int updateDisplayInterval = 243;

void setup() {
  ina219.begin();
  lcd.begin(16, 2);
  dac.begin(0x60);
  dacOutput = dacOutMin;
  dac.setVoltage(dacOutput, true);
  attachInterrupt(0,hallSenseCountISR,CHANGE);
  TWBR = ((F_CPU / 400000L) - 16) / 2;
#ifdef DEBUG
  Serial.begin(115200);
#endif
}

void loop() {
  if ( millis() > computePidLast + computePidInterval ){
    computePidLast = millis();
    readSetpoint();
    measureSpeed();
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

void hallSenseCountISR() {
  hallSenseCount ++;
}

void readSetpoint() {
  int setpointRaw = analogRead(0);
  if ( setpointRaw < 2 ) {
    setpointSpeed = 0;
    return;
  }
  setpointSpeed = map(setpointRaw,2,1023,speedMin,speedMax);
}

void measureSpeed() {
  int count = hallSenseCount;  // Quickly grab the counter value and rest it to avoid collision with the ISR
  hallSenseCount = 0;
  // 16 counts/rev, 11.64 inches/rev, 3600 s/h, 63360 in/mi
  // Convert count/interval to count/second.
  // Then offline dimensional analysis gives us a multiplying factor of 0.041335227 to get MPH
  // To avoid floting point math, we use the factor*1,000,000 and then divide back down
  hallSpeed = ((count * (1000 / computePidInterval)) * 41335) / 1000;  // Results in 1000*speed as noted in variable declarations above
}

void setDacOutput() {
  errorNow = setpointSpeed - hallSpeed;                           // Difference between setpoint and actual
  errorProportional = errorNow * gainProportional;                // Calculate the proportional adjustment value in terms of the input and setpoint units
  errorIntegral = errorIntegral + (errorNow * gainIntegral);      // Calculate the integral adjustment value " " "
  errorDerivative = (errorPrevious - errorNow) * gainDerivative;  // Calculate the derivative adjustment value " " "
  
  if ( errorIntegral > errorIntegralMax ) errorIntegral = errorIntegralMax;      // Limit the integral to prevent wind-up
  else if ( errorIntegral < errorIntegralMin ) errorIntegral = errorIntegralMin;

  outputAdjustmentFactor = errorProportional + errorIntegral + errorDerivative;  // Scale the output adjustment to a unit-agnostic factor
  dacOutput = dacOutput + outputAdjustmentFactor;                                // Set the output value with the adjustment factor
  if ( dacOutput < dacOutMin ) dacOutput = dacOutMin;                            // Limit the output to a valid range (Min)
  else if ( dacOutput > dacOutMax ) dacOutput = dacOutMax;                       // (Max)
  
  dac.setVoltage(dacOutput, false);                                              // Update the output DAC
  
  errorPrevious = errorNow;                                                      // Remember last measurement
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
  Serial.print(float(hallSpeed)/1000.0,3);
  Serial.print("  Setpoint: ");
  Serial.print(float(setpointSpeed)/1000.0,3);
  Serial.print("  Factor: ");
  if ( outputAdjustmentFactor > 0 ) Serial.print(" ");
  Serial.print(outputAdjustmentFactor,3);
  Serial.print("  P: ");
  Serial.print(errorProportional,3);
  Serial.print("  I: ");
  Serial.print(errorIntegral,3);
  Serial.print("  D: ");
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

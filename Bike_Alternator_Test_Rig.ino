#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
Adafruit_MCP4725 dac;

// Speed measurement variables
// Speed is based on time between hall sensor readings
// Higher time measurement = slower speed
const int hallTimeMin = 2400;           // The fastest possible motor speed, in terms of micros() between sensor pulses
const int hallTimeMax = 24000;          // The slowest " " " before we just stop it entirely
volatile boolean hallSenseNow = false;  // Did the hall sensor pulse?
long int hallSensePrior = 0;            // The last time in micros() the hall sensor pulsed
long int speedMeasured = hallTimeMax;   // Start off with the speed measurement at the lowest possible for safety

// DAC output variables
const int speedControlOutMin         = 1650; // The lowest usable DAC output value
const int speedControlOutMax         = 4095; // The highest usable DAC output value
      int speedControlOutput;                // The value sent to the DAC
const int speedControlUpdateInterval = 24;   // How often to update the DAC in millis()
long  int speedControlUpdateLast     = 0;    // When the DAC was updated last, in millis()

// Speed control input variables
      int speedSetpointRaw;                 // Bare reading from the speed setpoint input (analog pin)
long  int speedSetpointMapped;              // The speed setpoint value after we map() it to hall sensor boundaries
const int speedSetpointPin          = 0;    // The speed setpoint is on analog pin 0
long  int speedSetpointReadLast     = 0;    // When the setpoint was last read
const int speedSetpointReadInterval = 100;  // how often to read the setpoint

// Power measurement
float altVoltage = 0.0;
float altCurrent = 0.0;

// Display variables
#define DEBUG                                // Turns on Serial output
float speedSetpointMPH;                        // Calculated setpoint in MPH
float speedMeasuredMPH;                        // Calculated (calibrated!) actual speed in MPH
const int speedSamplesCount = 24;              // Average this many samples for actual speed...
long int speedSamplesArray[speedSamplesCount]; // ...using this array
long int speedSamplesIndex = 0;       
long int speedSamplesTotal = 0;
long int speedSamplesAverage = 0;
const int displayUpdatePeriod = 250;        // How often in millis() to update the display
long int displaylUpdatePrior = 0;       // When the display was updated last
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

void setup() {
  ina219.begin();
  lcd.begin(16, 2);
  dac.begin(0x60);
  speedControlOutput=speedControlOutMin;
  dac.setVoltage(speedControlOutput, true);
  attachInterrupt(0,hallSenseNowISR,FALLING);
  TWBR = ((F_CPU / 400000L) - 16) / 2;
#ifdef DEBUG
  Serial.begin(115200);
#endif
  // Initialize speed averaging array to all zeroes
  for (int tempIndex = 0; tempIndex < speedSamplesCount; tempIndex++) speedSamplesArray[tempIndex] = 0;
}

void loop() {
  readSpeedSetpoint();
  measureSpeed();
  adjustSpeedControl();
  measurePower();
  updateDisplay();
}

void hallSenseNowISR() {
  hallSenseNow = true;
}

void readSpeedSetpoint() {
  if ( millis() > speedSetpointReadLast + speedSetpointReadInterval ) {
    speedSetpointReadLast = millis();
    speedSetpointRaw = analogRead(0);
    speedSetpointMapped = map(speedSetpointRaw,2,1023,hallTimeMax,hallTimeMin);
  }
}

void measureSpeed() {
  if ( hallSenseNow ) {
    hallSenseNow = false;
    speedMeasured = micros() - hallSensePrior;
    hallSensePrior = micros();
  }
  if ( micros() > hallSensePrior + hallTimeMax ) {
    speedMeasured = hallTimeMax;
  }
  speedSamplesTotal = speedSamplesTotal - speedSamplesArray[speedSamplesIndex];         
  speedSamplesArray[speedSamplesIndex] = speedMeasured; 
  speedSamplesTotal = speedSamplesTotal + speedSamplesArray[speedSamplesIndex];       
  speedSamplesIndex++;                    
  if ( speedSamplesIndex >= speedSamplesCount ) speedSamplesIndex = 0;                           
  speedSamplesAverage = speedSamplesTotal / speedSamplesCount;
}

void adjustSpeedControl() {
  if ( millis() > speedControlUpdateLast + speedControlUpdateInterval ) {
    speedControlUpdateLast = millis();
    speedControlOutput = float(speedControlOutput)*(0.005*((1.0-(float(speedSetpointMapped)/float(speedSamplesAverage))))+1.0);
    if ( speedControlOutput < speedControlOutMin ) speedControlOutput = speedControlOutMin;
    if ( speedControlOutput > speedControlOutMax ) speedControlOutput = speedControlOutMax;
    if ( speedSetpointRaw < 2 ) speedControlOutput = speedControlOutMin;
    dac.setVoltage(speedControlOutput, false);
  }
}

void measurePower(){
  altVoltage = ina219.getBusVoltage_V();
  altCurrent = ina219.getCurrent_mA();
}

void updateDisplay() {
  if ( millis() > displaylUpdatePrior + displayUpdatePeriod ) {
    displaylUpdatePrior = millis();
//    if ( speedSetpointRaw < 2 ) {
//      speedSetpointMPH = 0;
//      speedMeasuredMPH = 0;
//    }
//    else {
      speedMeasuredMPH = (1000.0/((float(speedSamplesAverage)/1000.0)*8.0))*0.66;
      speedSetpointMPH = (1000.0/((float(speedSetpointMapped)/1000.0)*8.0))*0.66;
//    }
#ifdef DEBUG
    Serial.print("Measured: ");
    Serial.print(speedMeasured);
    Serial.print(" uSec\tSetpoint: ");
    Serial.print(speedSetpointMapped);
    Serial.print(" uSec\tDAC Out:");
    Serial.print(speedControlOutput);
    Serial.print("\tActual Speed:");
    Serial.print(speedMeasuredMPH);
    Serial.print(" MPH\tSet Speed:");
    Serial.print(speedSetpointMPH);
    Serial.println(" MPH");
#endif
    lcd.setCursor(0, 0);
    lcd.print("S: ");
    if ( speedSetpointMPH < 10.00 ) lcd.print(" ");
    lcd.print(speedSetpointMPH,1);
    lcd.print(" ");
    lcd.setCursor(8, 0);
    lcd.print("E:  ");
    if ( altVoltage < 10.00 ) lcd.print(" ");
    lcd.print(altVoltage,1);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print("M: ");
    if ( speedMeasuredMPH < 10.00 ) lcd.print(" ");
    lcd.print(speedMeasuredMPH,1);
    lcd.print(" ");
    lcd.setCursor(8, 1);
    lcd.print("I:");
    if ( altCurrent < 1000 ) lcd.print(" ");
    if ( altCurrent < 100 ) lcd.print(" ");
    if ( altCurrent < 10 ) lcd.print(" ");
    lcd.print(altCurrent,1);
    lcd.print(" ");
  }
}

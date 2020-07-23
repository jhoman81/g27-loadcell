#include <HX711.h>
#include <Joystick.h>

// Based on Trigen's code from:
// https://www.xsimulator.net/community/threads/diy-load-cell-brake-pedal-short-tuto.6042/page-3
// This uses a HX711 running at 80hz, an Arduino Micro Pro, 50kg load cell,
// setup in a wheatstone bridge, and the output from the G27 pedals all
// loaded into a generic joystick device that should work in any sim.

// DEFINES
#define calibration_factor 3000 // Calibrate loadcell to get this value
#define DOUT  6                 // DOUT pin on HX711
#define CLK  5                  // CLK pin on HX711

// How many samples to take at initialization of the library to zero
// out the offset of the load cell.
const int BRAKE_PEDAL_LOAD_CELL_TARE_REPS = 10;

HX711 loadcell;

// See Joystick.h for flag information
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_MULTI_AXIS, 4, 3,
  false, false, true, false, false, false,
  false, true, true, true, false);

// Pin variables
int throttle = A0;              // Throttle from G27 (Pin 2)
int clutch = A2;                // Clutch from G27 (Pin 4)

// Value variables -- MUST INPUT OWN VALUES
int brake = 0;
int brakeValue = 0;
int lastBrakeValue = 0;
int throttleValue = 0;
int lastThrottleValue = 0; 
int clutchValue = 0;
int lastClutchValue = 0;

void setup() {
    // Ranges are 1023 by default
    Joystick.setBrakeRange(0, 800); 
    Joystick.setThrottleRange(0, 1023); 
  
    Joystick.begin();

    // Start HX711 Loadcell
    Serial.begin(38400);
  
    // Calibrate prior to running, do not step on pedals
    loadcell.begin(DOUT, CLK);
    loadcell.set_scale(calibration_factor); 
    loadcell.tare(BRAKE_PEDAL_LOAD_CELL_TARE_REPS);
}

void loop() {
    // Debugging - shows outputs of each value when uncommented
    // You can comment out the max and mins for each variable below to get own values to avoid waking bug.
    // Serial.println (brakeValue);
    // Serial.println (throttleValue);
    // Serial.println (clutchValue);
  
    // Set values below based on maximum and minimum values that you found above.
    
    // THROTTLE
    throttleValue = analogRead(throttle);
    
    if (throttleValue > 890) {
        throttleValue = 890;
    }
    else if (throttleValue < 53) {
        throttleValue = 0;
    }
    if (lastThrottleValue != throttleValue) {
        Joystick.setThrottle(throttleValue);
        lastThrottleValue = throttleValue;
    }
    delay(1);

    // BRAKE
    brakeValue = loadcell.get_units(1); // if the value is inverted put a - sign in front like -scale.get

    // If the value starts below 0 set it to 0. Or if its above 50 set it to 0
    // This fixes the slight drift and sets it to 0 if it starts below 0
    // Sets brake then gets reading
    
    if (brakeValue < 0 or brakeValue < 8){
        brakeValue = 0;  
    }
    if (lastBrakeValue != brakeValue) {
       Joystick.setBrake(brakeValue);
       lastBrakeValue = brakeValue;
    }
    delay(1);

    // CLUTCH
    clutchValue = analogRead(clutch);
    if (clutchValue <= 53) {
        clutchValue = 0;
    }
    if (lastClutchValue != clutchValue) {
        Joystick.setZAxis(clutchValue);
        lastClutchValue = clutchValue;
    } 
    delay(1);
}

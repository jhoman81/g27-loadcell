#include <HX711.h>
#include <Joystick.h>
#define calibration_factor 2300 // Do your calibration first. default 2300
#define DOUT  4
#define CLK  5

HX711 scale;

/*
Define Joystick object - 4 buttons, 3 hatswitch?

For the true false flags, use this list. Its all in  Joystick.h 
    bool includeXAxis = true,
    bool includeYAxis = true,
    bool includeZAxis = true,
    bool includeRxAxis = true,
    bool includeRyAxis = true,
    bool includeRzAxis = true,
    bool includeRudder = true,
    bool includeThrottle = true,
    bool includeAccelerator = true,
    bool includeBrake = true,
    bool includeSteering = true);
  */


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_MULTI_AXIS, 4, 3,
  false, false, true, false, false, false,
  false, true, true, true, false);

// Pin variables
int throttle = A0;
// int brake = 2; / no need for this as we read directly from scale
int clutch = A2;

// Value variables
int brake = 0;
int throttleValue = 0;
int clutchValue = 0;

// init joystick libary
void setup() {
    // Ranges are 1023 by default
    Joystick.setBrakeRange(0, 1023); 
    Joystick.setThrottleRange(0, 1023); 
    //Joystick.setZAxisRange(0, 1023);
  
    Joystick.begin();

    // Start HX711 Loadcell
    Serial.begin(38400);
  
    // Calibrate prior to running, do not step on pedals
    scale.begin(DOUT, CLK);
    scale.set_scale(calibration_factor); 
    scale.tare();
}

void loop() {
    // Debugging - shows outputs of each value
    Serial.println (brake);
    Serial.println (throttleValue);
    Serial.println (clutchValue);
  
    throttleValue = analogRead(throttle);

    if (throttleValue > 772) {
        throttleValue = 778;
    }
    else if (throttleValue < 271) {
        throttleValue = 270;
    }
    else {
        throttleValue = analogRead(throttle);
    }
    
    Joystick.setThrottle(throttleValue);
    
    delay(1);

    brake = scale.get_units(); // if the value is inverted put a - sign in front like -scale.get
  
    // If the value starts below 0 set it to 0. Or if its above 50 set it to 0
    // This fixes the slight drift and sets it to 0 if it starts below 0
    // Sets brake then gets reading
    if (brake < 0 or brake < 8) {
        brake = 0;
    }
    else {
        brake = scale.get_units();
    }

    Joystick.setBrake(brake);
    delay(1);

    // Clutch readings
    clutchValue = analogRead(clutch);
    
    if (clutchValue < 185) {
        clutchValue = 180;
    }
    else {
        clutchValue = analogRead(clutch);
    }
    
    Joystick.setZAxis(clutchValue);
 
    delay(1);
}

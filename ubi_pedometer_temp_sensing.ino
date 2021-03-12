#include "Adafruit_FloraPixel.h"
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// Setting up Flora PixelRing 
Adafruit_FloraPixel strip = Adafruit_FloraPixel(1);

// Setting up Adafruit IMU as the Adafruit LSM303
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// Define Analog inputs Temperature Sensor
#define TEMP           9
#define VIBE          10

// Variables for IMU and Step Counting
int i;
int step_count, flag = 0; // Total Step Count and Flag Condition
const int threshold = 13;       // Threshold value to register movement as a step
float accel_vec;          // Acceleration Vector for holding movement value 
                          // which should be over a threshold to register as a step

// Variables for Temperature Sensing
int sensorValue = 0;      // Value Holder from Temp Sensor     
float degreesC = 0;       // Instantiating degrees celsius variable
int rx;
int gx;
int bx;

////////////////////////////////////////////
//              Setup                     //
//                                        //
////////////////////////////////////////////

void setup() {
  strip.begin();
  strip.show();
  displaySensorDetails();

  #ifndef ESP8266
    while (!Serial); // will pause Zero, Leonardo, etc until serial console opens
  #endif
  Serial.begin(9600);
  Serial.println("Accelerometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  accel.setRange(LSM303_RANGE_4G);
  Serial.print("Range set to: ");
  lsm303_accel_range_t new_range = accel.getRange();
  switch (new_range) {
  case LSM303_RANGE_2G:
    Serial.println("+- 2G");
    break;
  case LSM303_RANGE_4G:
    Serial.println("+- 4G");
    break;
  case LSM303_RANGE_8G:
    Serial.println("+- 8G");
    break;
  case LSM303_RANGE_16G:
    Serial.println("+- 16G");
    break;
  }

  accel.setMode(LSM303_MODE_NORMAL);
  Serial.print("Mode set to: ");
  lsm303_accel_mode_t new_mode = accel.getMode();
  switch (new_mode) {
  case LSM303_MODE_NORMAL:
    Serial.println("Normal");
    break;
  case LSM303_MODE_LOW_POWER:
    Serial.println("Low Power");
    break;
  case LSM303_MODE_HIGH_RESOLUTION:
    Serial.println("High Resolution");
    break;
  }

}




////////////////////////////////////////////
//              Loop                      //
//                                        //
////////////////////////////////////////////

void loop() {
  
  // Get new IMU sensor events
  sensors_event_t event;
  accel.getEvent(&event);

  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(event.acceleration.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(event.acceleration.z);
  Serial.print("  ");
  Serial.println("m/s^2");


  sensorValue = analogRead(TEMP);
//  degreesC = sensorValue/2.046 - 70.0;
  degreesC = ((sensorValue * 0.00488) - 0.8) * 100;
  

  Serial.print("Temp. Sensor Value = ");
  Serial.print(sensorValue);
  Serial.print("\t degrees C = ");
  Serial.println(degreesC);

  // Update RGB values based on current temperature readings

  if (degreesC < 20){
    rx = 0;
    gx = 0;
    bx = 255/3;
  } 
  else if (degreesC > 20){
    rx = 255;
    gx = 0;
    bx = 0;
  }
  else
  {
    rx = 0;
    gx = 255;   // If LEDs are green then something funky is going on. 
    bx = 0;
  }



   
  // Calculate the Acceleration Vector = sqrt(x^2+y^2+z^2)
  accel_vec = sqrt((event.acceleration.x*event.acceleration.x)+(event.acceleration.y*event.acceleration.y)+(event.acceleration.z*event.acceleration.z));
  Serial.println(accel_vec);

  // Set all LEDs on the strip to be Red 
//  colorWipe(Color(255/2, 0, 0), 50);  

  // Count Steps
  if (accel_vec > threshold && flag == 0)
  {
      step_count = step_count + 1;

      colorWipe(Color(rx, gx, bx), 50);  // Update RGB values when a step is taken


      if((step_count % 10) == 0){
        Serial.println("MILESTONE STEP: ");
        Serial.println(step_count);
        analogWrite(VIBE, 255);
        delay(250);
        analogWrite(VIBE, 0);
      }
  }
    else if (accel_vec > threshold && flag==1)
  {
    //pass
  }
    if (accel_vec < threshold && flag ==1)
  {
    flag = 0;
  }

  
  Serial.println("Steps taken: ");
  Serial.println(step_count);
  Serial.println("\n\n");
  
  accel_vec = 0; // Attempting to reset the acceleration vector from overflowing and going into infinity values.

  //Delay 500 ms before taking another sample.
  delay(500);
}


////////////////////////////////////////////
//              Functions                 //
//                                        //
////////////////////////////////////////////


// fill the LEDs sequentially with the given color
void colorWipe(RGBPixel c, uint8_t wait) {
  int i;
  
  for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}



// Create a 24 bit color value from R,G,B
RGBPixel Color(byte r, byte g, byte b)
{
  RGBPixel p;
  
  p.red = g;
  p.green = r;    // Swapping green and red since the FloraRing is for some reason getting those mixed up
  p.blue = b;
  
  return p;
}




void displaySensorDetails(void) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

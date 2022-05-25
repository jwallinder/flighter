#include <HCSR04.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

float gyroX, gyroY, gyroZ;
float gyroXcal, gyroYcal, gyroZcal;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float gyroAngleXdeg, gyroAngleYdeg, gyroAngleZdeg;
float elapsedTime, currentTime, previousTime;
float roll, pitch, yaw;
Servo servoX, servoY; 
int servoXval, servoYval;
int SERVO_X_PIN = 9;
int SERVO_Y_PIN = 10;
int SERVO_DEFAULT = 90;
int distanceAngle;

// Initialize sensor that uses digital pins 5 and 6.
const byte triggerPin = 5;
const byte echoPin = 6;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);
float distanceCm;

void displaySensorDetails(void) {
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Gyroscope Test"); Serial.println("");

  /* calibration light */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  /* setup servos */
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoX.write(SERVO_DEFAULT);
  servoY.write(SERVO_DEFAULT);
   
  
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  
  /* Initialise the sensor */
  if(!gyro.begin()) 
  //if (!gyro.begin(GYRO_RANGE_250DPS))
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  Serial.print("Starting calibration..."); 
  int calibrationCount = 4000;
  for (int cal_int = 0; cal_int < calibrationCount ; cal_int ++) {   //Take 2000 readings for calibration
    /* Get a new sensor event */ 
    sensors_event_t event; 
    gyro.getEvent(&event);

    gyroXcal += event.gyro.x;
    gyroYcal += event.gyro.y;
    gyroZcal += event.gyro.z;
    
//    gyro_signalen();                                 //Read the gyro output
//    gyro_roll_cal += gyro_roll;                      //Ad roll value to gyro_roll_cal
//    gyro_pitch_cal += gyro_pitch;                    //Ad pitch value to gyro_pitch_cal
//    gyro_yaw_cal += gyro_yaw;                        //Ad yaw value to gyro_yaw_cal
    if (cal_int % 100 == 0)Serial.print(".");        //Print a dot every 100 readings
    delay(4);                                        //Wait 4 milliseconds before the next loop
  }

  digitalWrite(LED_BUILTIN, HIGH);    //calibration done, turn on led
  Serial.println(" done!");                          //2000 measures are done!
  gyroXcal /= calibrationCount;
  gyroYcal /= calibrationCount;
  gyroZcal /= calibrationCount;
  Serial.print("gyroXCal: "); Serial.print(gyroXcal); Serial.print(" ");
  Serial.print("gyroYCal: "); Serial.print(gyroYcal); Serial.print(" ");
  Serial.print("gyroZCal: "); Serial.print(gyroZcal); Serial.print(" ");
  Serial.println();

}
void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  gyro.getEvent(&event);

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = event.timestamp;           // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  gyroX = event.gyro.x - gyroXcal;
  gyroY = event.gyro.y - gyroYcal;
  gyroZ = event.gyro.z - gyroZcal;

  gyroAngleX = gyroAngleX + gyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + gyroY * elapsedTime;
  gyroAngleZ = gyroAngleZ + gyroZ * elapsedTime;
  

  gyroAngleXdeg = gyroAngleX*180/3.14159;
  gyroAngleYdeg = gyroAngleY*180/3.14159;
  gyroAngleZdeg = gyroAngleZ*180/3.14159;
  
  /* Display the results (speed is measured in rad/s) */;
  Serial.print("time: "); Serial.print(currentTime); Serial.print(" ");
  //Serial.print("elapsedtime: "); Serial.print(elapsedTime); Serial.print(" ");
  
  //Serial.print("X: "); Serial.print(gyroX); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(gyroY); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(gyroZ); Serial.print("  ");
  //Serial.print("rad/s ");
  
  //Serial.print("gyroAngleX: "); Serial.print(gyroAngleX); Serial.print(" "); 

  // read the distance 
  distanceCm = distanceSensor.measureDistanceCm();
  Serial.print("distance: "); Serial.print(distanceCm); Serial.print("cm ");


  // distances from 2cm to 12cm avaw adds 5deg/cm
  // 2cm => -25
  // 7cm => 0
  // 12cm => 25
  
  
  if (distanceCm >= 2 && distanceCm <= 12) {
    distanceAngle = 5* (distanceCm - 7);     
  } else if (distanceCm >= 0 &&  distanceCm <2) {
    distanceAngle = -25;
  } else if (distanceCm >= 12){
    distanceAngle = 25; 
  }
  
  Serial.print("distanceAngle: "); Serial.print(distanceAngle); Serial.print("deg ");
  
  servoXval = SERVO_DEFAULT + gyroAngleXdeg + distanceAngle;
  servoYval = SERVO_DEFAULT + gyroAngleYdeg + distanceAngle;
  

  servoX.write(servoXval); //just write the gyro angle to servo
  servoY.write(servoYval); //just write the gyro angle to servo
    
  outputX();
  outputY();
//  outputZ();
  
  Serial.println();
  delay(10);


}

void outputX() {
  Serial.print("gyroAngleXdeg: "); Serial.print(gyroAngleXdeg); Serial.print("deg ");
}
void outputY() {
  Serial.print("gyroAngleYdeg: "); Serial.print(gyroAngleYdeg); Serial.print("deg ");
}

void outputZ() {
  Serial.print("gyroAngleZdeg: "); Serial.print(gyroAngleZdeg); Serial.print("deg ");
}

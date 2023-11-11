#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <DFRobot_BMP3XX.h>
#include <Servo.h>

File myFile;
String nameGlobal;
int ballslowbrass = 0;

Servo myservo;
int pos = 0;

const int IDLE = 0; // Idle is when not doing anything (before coasting)
const int RECORD = 1; //Record is when only recording data
const int ACTIVE = 2; //Active is when airbrake control is used
//int mode = IDLE;
int mode = ACTIVE;
const double activeThresh = 100; //altitude in meters, when rocket goes from idel to active
const int xPin = A2;
const int yPin = A1;
const int zPin = A0;
double initialX = 0.0;
double initialY = 0.0;
double initialZ = 0.0;
double currX = 0.0;
double currY = 0.0;
double currZ = 0.0;
double xRaw = 0.0, yRaw = 0.0, zRaw = 0.0;
const double samples = 10.0;
double barometricAcceleration = 0.0;

DFRobot_BMP390L_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

double altitude = 0.0; 
double prevAltitude = 0.0;
double time = 0.0;
double prevTime = 0.0;
double velocity = 0.0;
double prevVelocity = 0.0;

const int mmExtend = 1; //how many millimeters to extend actuator per loop
int extended = 0;

const double g = 9.81; 
const double mass = 0.5652; 
const double target = 250; 
double apogee = 0.0;

int begin = 5;

void setup() {
  Serial.begin(460800);

  pinMode(8, OUTPUT);
  SD.begin(8);
  int i = 0;
  String nameStart = "d";
  String nameMid = nameStart + i;
  String name = nameMid + ".txt";
  while (SD.exists(name)) {
    i++;
    nameMid = nameStart + i;
    name = nameMid + ".txt";
  }
  nameGlobal = name;

  myservo.attach(9);
  myservo.writeMicroseconds(2000);
  delay(3000);

  sensor.begin();
  sensor.setSamplingMode(sensor.eUltraPrecision);
  delay(100);
  #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
    sensor.calibratedAbsoluteDifference(0.0);
  #endif
}

void loop() {
  time = millis()/1000.0;
  altitude = sensor.readAltitudeM();
  /*
  if (mode == IDLE && altitude > activeThresh){ //change mode to active when past the altitude threshold (coasting phase)
    mode == ACTIVE;
  }
  */
  updateVelocity();
  updateAcceleration();
  if (mode == ACTIVE) {
    if (velocity > 0) {
      if (currX < 0) {
        updateApogee(currX);
      }
    }
    //updatePos();
  }
  writeData();
  addDelay();
  prevAltitude = altitude;
  prevVelocity = velocity;
  prevTime = time;
}

void addDelay() {
  delay(100);

  /*
  if (extended == 0 && mode == ACTIVE)
    delay(100 * mmExtend); //artificial delay to keep delta T constant
  if (mode == RECORD)
    delay(recordDelay);
  if (mode == IDLE)
    delay(idleDelay);
  */
}

void updateAcceleration() {
  barometricAcceleration = (velocity - prevVelocity)/(time - prevTime);
  if (begin==0) {
    xRaw = 0.0; 
    yRaw = 0.0; 
    zRaw = 0.0;
    for(int i=0;i<samples;i++)
    {
      xRaw+=analogRead(xPin);
      yRaw+=analogRead(yPin);
      zRaw+=analogRead(zPin);
    }
    xRaw/=samples;
    yRaw/=samples;
    zRaw/=samples;
    
    currX = map(xRaw*100.0, initialX*100.0 - 1400.0, initialX*100.0 + 1400.0, -98100.0, 98100.0)/10000.0;
    currY = map(yRaw*100.0, initialY*100.0 - 1400.0, initialY*100.0 + 1400.0, -98100.0, 98100.0)/10000.0;
    currZ = map(zRaw*100.0, initialZ*100.0 - 2800.0, initialZ*100.0, -98100.0, 98100.0)/10000.0;
  } else {
    initialX=0.0;
    initialY=0.0;
    initialZ=0.0;
    for(int i=0;i<samples;i++)
    {
      initialX+=analogRead(xPin);
      initialY+=analogRead(yPin);
      initialZ+=analogRead(zPin);
    }
    initialX/=samples;
    initialY/=samples;
    initialZ/=samples;

    begin--;
  }

}

void updateVelocity(){
  velocity = (altitude - prevAltitude)/(time - prevTime);
}

void updatePos() {
  if (target < apogee){
    extended = 1; //how many millimeters to extend actuator
    int maxExtend = min(pos + 6*mmExtend, 180);
    for (int i = pos; i <= maxExtend; i += 6)
    {
      pos = i;
      myservo.write(pos);
      delay(200);
    }
    pos = maxExtend;
    myservo.write(pos);
  } else if (target > apogee){
    extended = -1;
    int maxExtend = max(pos - 6*mmExtend, 0);
    for (int i = pos; i >= maxExtend; i -= 6)
    {
      pos = i;
      myservo.write(pos);
      delay(200);
    }
    pos = maxExtend;
    myservo.write(pos);
  }
}

void updateApogee(double accel) {
  double Cd = 0.0;
	
  double num = -2.0 * (accel * mass + mass*9.81);
  double den = 1.0 * velocity * velocity;
  
  Cd = 1.0 * num/den;
  
  double deltaApogee = 0.0 + ((mass / (Cd))*log((mass*9.81 + 0.5*Cd*velocity*velocity) / (mass*9.81)));
  
  apogee = 0.0 + deltaApogee + altitude; 
}

void writeData() {
  myFile = SD.open(nameGlobal, FILE_WRITE);
  if (myFile) {
    myFile.print(nameGlobal);
    myFile.print("\n");
    myFile.print(time);
    myFile.println(" seconds");



    myFile.print("Accelerometer Data"); 
    myFile.print("\n");


    myFile.print("Raw Accelerometer 0G Acceleration (x, y, z)");
    myFile.print("\t");

    myFile.print(initialX);
    myFile.print("\t");

    myFile.print(initialY);
    myFile.print("\t");

    myFile.print(initialZ);
    myFile.print("\n");


    myFile.print("Raw Accelerometer Acceleration (x, y, z)");
    myFile.print("\t");

    myFile.print(xRaw);
    myFile.print("\t");

    myFile.print(yRaw);
    myFile.print("\t");

    myFile.print(zRaw);
    myFile.print("\n");


    myFile.print("Calculated Acceleration (m/s)^2 (x, y, z)");
    myFile.print("\t");

    myFile.print(currX);
    myFile.print("\t");

    myFile.print(currY);
    myFile.print("\t");

    myFile.print(currZ);
    myFile.print("\n");



    myFile.print("Barometer Data");
    myFile.print("\n");


    myFile.print("Barometric Altitude (m)");
    myFile.print("\t");
    myFile.print(altitude); 
    myFile.print("\n");


    myFile.print("Barometric Velocity (m/s)");
    myFile.print("\t");
    myFile.print(velocity); 
    myFile.print("\n");


    myFile.print("Barometric Acceleration (m/s)^2");
    myFile.print("\t");
    myFile.print(barometricAcceleration);
    myFile.print("\n");



    if (mode == ACTIVE) {
      myFile.print("Estimated Apogee (m)");
      myFile.print("\t");
      myFile.print(apogee);
      myFile.print("\n");

      myFile.print("Actuator Extension (mm)");
      myFile.print("\t");
      myFile.print(pos);
      myFile.print("\n");
    }
    if (extended != 0){
      myFile.print("Extending: ");
      myFile.println(extended);
      
      myFile.print("Extended Actuator (mm)");
      myFile.print("\t");
      myFile.println((mmExtend*extended));
    }
    myFile.close();


    Serial.print(nameGlobal);
    Serial.print("\n");
    Serial.print(time);
    Serial.println(" seconds");



    Serial.print("Accelerometer Data"); 
    Serial.print("\n");


    Serial.print("Raw Accelerometer 0G Acceleration (x, y, z)");
    Serial.print("\t");

    Serial.print(initialX);
    Serial.print("\t");

    Serial.print(initialY);
    Serial.print("\t");

    Serial.print(initialZ);
    Serial.print("\n");


    Serial.print("Raw Accelerometer Acceleration (x, y, z)");
    Serial.print("\t");

    Serial.print(xRaw);
    Serial.print("\t");

    Serial.print(yRaw);
    Serial.print("\t");

    Serial.print(zRaw);
    Serial.print("\n");


    Serial.print("Calculated Acceleration (m/s)^2 (x, y, z)");
    Serial.print("\t");

    Serial.print(currX);
    Serial.print("\t");

    Serial.print(currY);
    Serial.print("\t");

    Serial.print(currZ);
    Serial.print("\n");



    Serial.print("Barometer Data");
    Serial.print("\n");


    Serial.print("Barometric Altitude (m)");
    Serial.print("\t");
    Serial.print(altitude); 
    Serial.print("\n");


    Serial.print("Barometric Velocity (m/s)");
    Serial.print("\t");
    Serial.print(velocity); 
    Serial.print("\n");


    Serial.print("Barometric Acceleration (m/s)^2");
    Serial.print("\t");
    Serial.print(barometricAcceleration);
    Serial.print("\n");



    if (mode == ACTIVE) {
      Serial.print("Estimated Apogee (m)");
      Serial.print("\t");
      Serial.print(apogee);
      Serial.print("\n");

      Serial.print("Actuator Extension (mm)");
      Serial.print("\t");
      Serial.print(pos);
      Serial.print("\n");
    }
    if (extended != 0){
      Serial.print("Extending: ");
      Serial.println(extended);
      
      Serial.print("Extended Actuator (mm)");
      Serial.print("\t");
      Serial.println((mmExtend*extended));
    }
  }
}

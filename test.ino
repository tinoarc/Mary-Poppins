#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <DFRobot_BMP3XX.h>
#include <Servo.h>

File myFile;
String nameGlobal;

Servo myservo;
int pos = 0;

const int IDLE = 0; // Idle is when not doing anything (before coasting)
const int RECORD = 1; //Record is when only recording data
const int ACTIVE = 2; //Active is when airbrake control is used
int mode = ACTIVE;
const double activeThresh = 100; //altitude in meters, when rocket goes from idel to active
const int recordDelay = 50; //how quickly the arduino asks for data from sensors (in ms)
const int idleDelay = 100;

const int xPin = A2;
const int yPin = A1;
const int zPin = A0;
double initialX = 0; //might have to change these integers to doubles
double initialY = 0;
double initialZ = 0;
double currX = 0;
double currY = 0;
double currZ = 0;
double xRaw = 0.0, yRaw = 0.0, zRaw = 0.0;
const int samples = 10;
double barometricAcceleration = 0.0;

DFRobot_BMP390L_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

double altitude = 0; 
double prevAltitude = 0;
unsigned long time = 0; //is in milliseconds
unsigned long prevTime = 0;
double velocity = 0;
double prevVelocity = 0.0;

const int mmExtend = 1; //how many millimeters to extend actuator per loop
int extended = 0;

const double g = 9.81; //gravity
const double rho = 1.16; //1.2 kg/m^3 is placeholder rn
const double baseSA = 0.004046; //base surface area without extension
const double mass = 0.512; //in kg
const double target = 250; 
double apogee;

int begin = 5; //begin - 1 is the number of beginning values ignored

void setup() {
  Serial.begin(460800); //might need to change this
  pinMode(8, OUTPUT);
  if (!SD.begin(8)) {
    Serial.println("Initialization failed!");
    while (1) delay(10); //infinite loop possibility
  }
    
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

  initialX+=analogRead(xPin);
  initialY+=analogRead(yPin);
  initialZ+=analogRead(zPin);

  //barometer setup
  int rslt;
  while( ERR_OK != (rslt = sensor.begin()) ){
    if(ERR_DATA_BUS == rslt){
      Serial.println("Data bus error!!!");
    }else if(ERR_IC_VERSION == rslt){
      Serial.println("Chip versions do not match!!!");
    }
    delay(3000); //delay might need to be changed
  }
  Serial.println("Begin ok!");

  while( !sensor.setSamplingMode(sensor.eUltraPrecision) ){
    Serial.println("Set sampling mode fail, retrying....");
    delay(3000);
  }

  delay(100);
  #ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
  if( sensor.calibratedAbsoluteDifference(0) ){
    Serial.println("Absolute difference base value set successfully!");
  }
  #endif
}

void loop() {
  extended = 0;
  altitude = sensor.readAltitudeM();
  if (mode == IDLE && altitude > activeThresh){ //change mode to active when past the altitude threshold (coasting phase)
    mode == ACTIVE;
  }
    
  time = millis();
  updateVelocity();
  updateAcceleration();
  if (mode == ACTIVE){
    updateApogee();
    updatePos();
  }
  writeData();
  prevAltitude = altitude;
  prevVelocity = velocity;
  prevTime = time;
  addDelay();
}
void addDelay(){
  if (extended == 0 && mode == ACTIVE)
    delay(200 * mmExtend); //artificial delay to keep delta T constant
  if (mode == RECORD)
    delay(recordDelay);
  if (mode == IDLE)
    delay(idleDelay);
}
void updateAcceleration() {
  double timeElapsed = (time - prevTime)/1000.0;
  barometricAcceleration = (velocity - prevVelocity)/(timeElapsed);
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
    
    currX = map(xRaw, initialX - 14.0, initialX + 14.0, -981.0, 981.0)/100.0;
    currY = map(yRaw, initialY - 14.0, initialY + 14.0, -981.0, 981.0)/100.0;
    currZ = map(zRaw, initialZ - 28.0, initialZ, -981.0, 981.0)/100.0;
    //currZ = -18;
  } else { //might need to be changed for efficiency
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
  double timeElapsed = (time - prevTime)/1000;
  velocity = (altitude - prevAltitude)/(timeElapsed);
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

void updateApogee() {
  double Cd = 0;
  double acceleration = currX; //temporary, might be currY or currZ
  double den = -2 * (acceleration * mass + mass*9.81);
  double num = velocity * velocity;
  
  Cd = den/num;
  
  double deltaApogee = (mass / (Cd))*log((mass*9.81 + 0.5*Cd*velocity*velocity) / (mass*9.81));
  
  apogee = deltaApogee + altitude; 
}

void writeData() {
  if (mode != IDLE){
    myFile = SD.open(nameGlobal, FILE_WRITE);
    if (myFile){
      Serial.print(nameGlobal);
      Serial.print("\n");
      // write necessary data to SD Card here
      myFile.print("Time (milliseconds)");
      myFile.print("\t");
      myFile.print(time); 
      myFile.print("\t");
      double timeInSeconds = time/1000.0;
      myFile.print(timeInSeconds); 
      myFile.print("\n");

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

      if (mode == ACTIVE){
        Serial.println();
        Serial.print("Predicted apogee: ");
        Serial.println(apogee);

        Serial.print("Velocity: ");
        Serial.println(velocity);

        myFile.print("Estimated Apogee (m)");
        myFile.print("\t");
        myFile.println(apogee);

        myFile.print("Actuator Extension (degrees)");
        myFile.print("\t");
        myFile.println(pos);
      }
      if (extended != 0){
        Serial.println();
        Serial.print("Extending: ");
        Serial.println(extended);
        
        myFile.print("Extended Actuator (mm)");
        myFile.print("\t");
        myFile.println((mmExtend*extended));
      }
      myFile.println(""); 
      myFile.close();

      Serial.println("done.");
    } else {
      Serial.println("error opening test.txt");
    }
  }
}

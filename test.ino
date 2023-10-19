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
const int samples = 10;

DFRobot_BMP390L_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

double altitude = 0; 
double prevAltitude = 0;
double time = 0;
double prevTime = 0;
double velocity = 0;
const int mmExtend = 1; //how many millimeters to extend actuator per loop
int extended = 0;

const double g = 9.81; //gravity
const double rho = 1.16; //1.2 kg/m^3 is placeholder rn
const double baseSA = 0.004046; //base surface area without extension
const double mass = 0.512; //in kg
const double target = 250; 
double apogee;

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
  updateAcceleration();
  if (mode == ACTIVE){
    updateVelocity();
    updateApogee();
    updatePos();
  }
  writeData();
  prevAltitude = altitude;
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
  int xRaw = 0, yRaw = 0, zRaw = 0;
  for(int i=0;i<samples;i++)
  {
    xRaw+=analogRead(xPin);
    yRaw+=analogRead(yPin);
    zRaw+=analogRead(zPin);
  }
  xRaw/=samples;
  yRaw/=samples;
  zRaw/=samples;
  currX = map(xRaw, initialX - 14, initialX + 14, -981, 981)/100.0;
  currY = map(yRaw, initialY - 14, initialY + 14, -981, 981)/100.0;
  currZ = 0;
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
      Serial.print("\t");
      // write necessary data to SD Card here
      myFile.print("Accelerometer Data: "); 
      myFile.print(currX);
      myFile.print(",\t");

      myFile.print(currY);
      myFile.print(",\t");

      myFile.print(currZ);
      myFile.print("\t");

      myFile.print("Barometer Data: "); 
      myFile.print(altitude);
      myFile.print("m"); //might need to be changed
      myFile.print("\t");
      if (mode == ACTIVE){
        myFile.print("Predicted apogee: ");
        myFile.print(apogee);
        myFile.print("m");
        myFile.print("\t");
        Serial.println();
        Serial.print("Predicted apogee: ");
        Serial.println(apogee);
        myFile.print("Velocity Data: "); 
        myFile.print(velocity);
        myFile.print("m/s"); //might need to be changed
        myFile.print("\t");
        Serial.print("Velocity: ");
        Serial.println(velocity);
      }
      if (extended != 0){
        Serial.println();
        Serial.print("Extending: ");
        Serial.println(extended);
        myFile.print("Extended Actuator: ");
        myFile.print((mmExtend*extended));
        myFile.print("mm");
      }
      myFile.println("");
      myFile.close();

      Serial.println("done.");
    } else {
      Serial.println("error opening test.txt");
    }
  }
}

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <DFRobot_BMP3XX.h>
//#include <PID_v1.h>  // Include the PID library
#include <Servo.h>

File myFile;
Servo myservo;
String nameGlobal;

const int IDLE = 0; // Idle is when not doing anything (before coasting)
const int RECORD = 1; //Record is when only recording data
const int ACTIVE = 2; //Active is when airbrake control is used
int mode = IDLE;
const double activeThresh = 100; //altitude in meters, when rocket goes from idel to active
const int recordDelay = 50; //how quickly the arduino asks for data from sensors (in ms)
const int idleDelay = 100;

const int xPin = A2;
const int yPin = A1;
const int zPin = A0;    
const int samples = 10;
const double g = 9.81; //gravity
const double rho = 1.16; //1.2 kg/m^3 is placeholder rn
const double baseSA = 0.004046; //base surface area without extension
const double mass = 0.512; //in kg

DFRobot_BMP390L_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

// Define PID constants (you need to tune these)
// double Kp = 1.0;  // Proportional gain
// double Ki = 0.0;  // Integral gain
// double Kd = 0.0;  // Derivative gain

double setpoint = 100.0;  // Setpoint altitude (adjust as needed)
double startAlt = 310.0; //in feet, of height of motor burnout

// Define PID variables
double altitude; //current altitude being read by barometer
double prevAlt = -1; //altitude detected in previous loop, used to get instantaneous velocity
const int mmExtend = 1; //how many millimeters to extend actuator per loop
int pos = 0;
int extended = 0; //is 0 if no extension, 1 if extended outwards, -1 if extended inwards
// Create an instance of PID controller
//PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(460800); //might need to change this
    myservo.attach(9);

    //SD Card reader setup
    pinMode(8, OUTPUT);
    if (!SD.begin(8)) {
        Serial.println("Initialization failed!");
        while (1) delay(10); //infinite loop possibility
    }
  
    // Rest of your setup code...
    // Setting up a new log file for data
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
    if( sensor.calibratedAbsoluteDifference(72.0) ){
        Serial.println("Absolute difference base value set successfully!");
    }
    #endif

    // Initialize PID controller
    //myPID.SetMode(AUTOMATIC); 
}

void loop() {
    extended = 0;

    //setup for accelerometer data, need to change map values according to calibration
    int xRaw = analogRead(xPin), yRaw = analogRead(yPin), zRaw = analogRead(zPin);
    double xCalc = map(xRaw, 140, 560, -16*98, 16*98)/10, yCalc = map(yRaw, 140, 560, -16*98, 16*98)/10, zCalc = map(zRaw, 140, 560, -16*98, 16*98)/10;

    // Read altitude from the sensor
    altitude = sensor.readAltitudeM(); //need to check if its reading out meters or feet/inches

    // Calculate the PID output
    //myPID.Compute();
    if (mode == IDLE && altitude > activeThresh){ //change mode to active when past the altitude threshold (coasting phase)
      mode == ACTIVE;
    }
    
    double apogee = -1; //placeholder 
    if (mode == ACTIVE){ //check if mode is active
      double target = 250; //in meters
      double v = calculateVel(altitude, prevAlt); //need to calculate velocity
      double k = calculateK(v, xCalc/10); //axis may be up for change depend on mount orientation
      apogee = predictApogee(mass, k, v) + altitude;
      // Adjust the airbrakes position based on the PID output
      // Replace this with your code to control the airbrakes
      // For example, you can map the output to the position of the airbrakes servo/motor

      //6s for extension, 30 degrees per second, 1/5 second for 6 degree extension/retraction
      
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
      
      // Serial.print("Control Output: ");
      // Serial.println(output);
    }
    
    prevAlt = altitude; //seting previous altitude to current altitude
    // Rest of your loop code...
    //writing data to sd card

    //reformat logging stuffs
    if (mode != IDLE){
      myFile = SD.open(nameGlobal, FILE_WRITE);
      if (myFile){
          Serial.print(nameGlobal);
          Serial.print("\t");
          // write necessary data to SD Card here
          myFile.print("Accelerometer Data: "); 
          myFile.print(xCalc/10);
          myFile.print(",\t");

          myFile.print(yCalc/10);
          myFile.print(",\t");

          myFile.print(zCalc/10);
          myFile.print("\t");

          myFile.print("Barometer Data: "); 
          myFile.print(altitude);
          myFile.print("m"); //might need to be changed
          myFile.print("\t");
          if (mode == ACTIVE){}
            myFile.print("Predicted apogee: ");
            myFile.print(apogee);
            myFile.print("m");
            myFile.print("\t");
            Serial.println(apogee);
          if (extended != 0){
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
    if (extended == 0 && mode == ACTIVE)
      delay(200 * mmExtend); //artificial delay to keep delta T constant
    if (mode == RECORD)
      delay(recordDelay);
    if (mode == IDLE)
      delay(idleDelay);
}

double calculateVel(double altitude, double prevAltitude){
    return (altitude - prevAltitude)/(200/1000 * mmExtend);
}

double calculateK(double currVel, double currAcc){
    //double A = calculateSA();
    double Cd = calculateCd(currVel, currAcc); //, A
    //double k = 0.5*rho*Cd*A; //rho is air density, Cd is drag coefficient, A is surface area
    double k = 0.5*Cd;
    return k;
}

//need to recalculate Drag coefficient
double calculateCd(double currVel, double currAcc){ //, double SA
    // double Cd = 0;
    // double den = -2 * (currAcc * mass + mass*g);
    // double num = rho * SA * currVel * currVel;
        
    // Cd = den/num;
    // return Cd;
    double Cd = 0;
    double den = -2 * (currAcc * mass + mass*g);
    double num = currVel * currVel;
    Cd = den/num;
    return Cd;
}

double calculateSA(){
    double airbrakeArea = sin(0.00421*pos)*3;
    return baseSA + airbrakeArea; //width =54.5, 60mm full extend angle is 43 degrees and 
}

double predictApogee(double M, double k, double v){
    double yc = (M / (2*k))*log((M*g + k*v*v) / (M*g));
    return yc;
}

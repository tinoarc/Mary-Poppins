#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <DFRobot_BMP3XX.h>
#include <PID_v1.h>  // Include the PID library
#include <Servo.h>

File myFile;
Servo myservo;
String nameGlobal;

const int xPin = A2;
const int yPin = A1;
const int zPin = A0;
const int samples = 10;

DFRobot_BMP390L_I2C sensor(&Wire, sensor.eSDOVDD);
#define CALIBRATE_ABSOLUTE_DIFFERENCE

// Define PID constants (you need to tune these)
double Kp = 1.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain

double setpoint = 100.0;  // Setpoint altitude (adjust as needed)
double startAlt = 310.0; //in feet, of height of motor burnout

// Define PID variables
double input, output;

int pos = 0;
bool extended = false;
int delay = 1000;
// Create an instance of PID controller
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(460800);
    myservo.attach(9);

    //SD Card reader setup
    pinMode(8, OUTPUT);
    if (!SD.begin(8)) {
        Serial.println("Initialization failed!");
        while (1) delay(10);
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
        delay(3000);
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
    myPID.SetMode(AUTOMATIC); 
}

void loop() {
    extended = false;
    // Rest of your loop code...
    //setup for accelerometer data
    int xRaw = analogRead(xPin), yRaw = analogRead(yPin), zRaw = analogRead(zPin);
    float xCalc = map(xRaw, 140, 560, -16*98, 16*98)/10, yCalc = map(yRaw, 140, 560, -16*98, 16*98)/10, zCalc = map(zRaw, 140, 560, -16*98, 16*98)/10;

    // Read altitude from the sensor
    input = sensor.readAltitudeM();

    // Calculate the PID output
    myPID.Compute();

    //6s for extension, 30 degrees per second, 1/5 second for 6 degree extension/retraction
    double apogee = 900;
    double target = 812;
    if (target < apogee){
        extended = true;
        int mmExtend = 5; //how many millimeters to extend actuator
        int maxExtend = min(pos + 6*mmExtend, 180);
        for (int i = pos; i <= maxExtend; pos += 6)
        {
            pos = i;
            myservo.write(pos);
            delay(200);
        }
        pos = maxExtend;
        myservo.write(pos);
    } else {
        extended = true;
        int mmExtend = -5; //how many millimeters to extend actuator
        int maxExtend = max(pos + 6*mmExtend, 0);
        for (int i = pos; i >= maxExtend; pos -= 6)
        {
            pos = i;
            myservo.write(pos);
            delay(200);
        }
        pos = maxExtend;
        myservo.write(pos);
    }
    // Adjust the airbrakes position based on the PID output
    // Replace this with your code to control the airbrakes
    // For example, you can map the output to the position of the airbrakes servo/motor

    Serial.print("Control Output: ");
    Serial.println(output);

    // Rest of your loop code...
    //writing data to sd card
    myFile = SD.open(nameGlobal, FILE_WRITE);
    if (myFile){
        Serial.print(nameGlobal);
        Serial.print("\t");
        // write necessary data to SD Card here
        myFile.println("Accelerometer Data"); 
        myFile.print(xCalc/10);
        myFile.print("\t");

        myFile.print(yCalc/10);
        myFile.print("\t");

        myFile.print(zCalc/10);
        myFile.println();

        myFile.println("Barometer Data"); 
        myFile.println(sensor.readAltitudeM());
        myFile.close();

        Serial.println("done.");
    } else {
        Serial.println("error opening test.txt");
    }
    delay(200); //may need this/????
}

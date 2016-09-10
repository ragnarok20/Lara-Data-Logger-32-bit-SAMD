//
// Lara Data Logger 32 bit SAMD
//
// record lever angles and distance traveled
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author 		Sage Thayer
// 				Sage Thayer
//
// Date			8/17/16 2:32 PM
// Version		<#version#>
//
// Copyright	© Sage Thayer, 2016
// Licence		<#licence#>
//
// See         ReadMe.txt for references
//


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
    #include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
    #include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
    #include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
    #include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
    #include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
    #include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
    #include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
    #include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
    #include "Arduino.h"
#elif defined(ESP8266) // ESP8266 specific
    #include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
    #include "Arduino.h"
#else // error
    #error Platform not defined
#endif // end IDE


// Include application, user and local libraries
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "MPU9250.h"
#include "I2Cdev.h"
#include <RTClib.h>
//#include <avr/sleep.h>
//#include <avr/wdt.h>
#include "Vector.h"
#include "wiring_private.h"
#include "timersSAMD.h"

#define Serial SerialUSB


//---- Defines for different outputs ------//
//#define ECHO        // Sets up a serial Monitor at 115200 baud
#define LOG      //logging macro define (takes a bit of memory to log)
//#define SET_rtc       //resets rtc time and date to the time of this compilation
#define POT         //pot lever angle measurement test

#define sampleFreq 500.0f		// sample frequency in Hz
#define gyroSampleFreq 50.0f		// sample frequency in Hz

//int logFreq = 30;
//int sampleFreq = 30;
#define SAMPLE_RATE_DIV (1000/gyroSampleFreq) - 1


//----------- Prototypes --------------//
void sleepNow();
void halfSleep();        //sort of sleep for tests. will slow donw sample rate to 1hz & loging rate to 1hz
void wakeUpNow();   //ISR
//void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickAHRSupdateIMU(Vector3<float> gyro, Vector3<float> acc, float q[4], Vector3<float> &ypr);
float invSqrt(float x);
void IMU_int();


//-----------Define variables and constants-----------//
const uint8_t MY_LED = 13;
long blink_start = 0;
// variables to figure out cycle rate
double begin_of_loop = 0;
double loop_time = 0;
double measured_cycle_rate = sampleFreq;
int delay_time = 0;

//----------SD Card & Logging-------------------//
// set up variables using the SD utility library functions:

bool SD_card_present = false;
bool file_exists = false;
// the logging file
String filenameString;
char filename[13];      // limited to 12 characters with FAT partitioned SD card...
File logfile;
double logTime = 0;
byte bytes_written = 0;

// Build a buffer array to write to a file
const int bufferSize = 5000;
//const int bufferSize = 480;
//const int bufferSize = 18;
uint8_t buffer[bufferSize];
uint8_t dataToWrite[bufferSize];
bool timeToWrite = false;
uint8_t dataBlock = 50;     //how much data to store per cycle in bytes
int writeCounter= 0;
boolean hour_transition_began = false;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
const int chipSelect = 10; //chip select for SD card

//------------------MPU-9250-------------------//
//9dof board
const int NCS1 = 8;   //chip select for mpu9250 1 Right
const int NCS2 = 9;   //chip select for mpu9250 2 Left
MPU9250 IMU1(NCS1,1000000);   //initialize for SPI at 1 MHz for 1st IMU
MPU9250 IMU2(NCS2,1000000);   //initialize for SPI at 1 MHz for 2nd IMU

bool IMU1_online = false;
bool IMU2_online = false;

uint16_t fifoCount[2] = {0,0};
uint16_t packet_count;
uint8_t fifoBufferIMU1[512];
uint8_t fifoBufferIMU2[512];
uint8_t Remainder[2] = {0,0};
uint8_t cyclesBehind = 0;
uint8_t numberPackets[2];

bool dataRdyIntSet = false;
bool gyroDataRdy = false;

//extern volatile float beta;				// algorithm gain
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

Vector3<int16_t> Acc_raw_right;   //elem 1: x, 2: y, 3: z
Vector3<int16_t> Gyro_raw_right;  //elem 1: x, 2: y, 3: z

Vector3<float> Acc_right;   //elem 1: x, 2: y, 3: z
Vector3<float> Gyro_right;  //elem 1: x, 2: y, 3: z

Vector3<int16_t> Acc_raw_left;   //elem 1: x, 2: y, 3: z
Vector3<int16_t> Gyro_raw_left;  //elem 1: x, 2: y, 3: z

Vector3<float> Acc_left;   //elem 1: x, 2: y, 3: z
Vector3<float> Gyro_left;  //elem 1: x, 2: y, 3: z

// attitude will represent the lever in euler angles
Vector3<float> Attitude_right_lever(0.0f,0.0f,0.0f);    //elem 1: yaw, 2: pitch, 3: roll
Vector3<float> Attitude_left_lever(0.0f,0.0f,0.0f);     //elem 1: yaw, 2: pitch, 3: roll


// http://quat.zachbennett.com
float quanternion_right[4] = {0.41f, 0.58f, -0.47f, -0.52f};  //sensor frame relative to the earth frame
float quanternion_left[4] = {0.28f, 0.64f, 0.26f, 0.67f};
volatile float beta = 1;                                // algorithm gain

// --------- Sleep Handeling -----------//
volatile bool mpu_int_triggered = false;
double motionTimer = 0;
//double motionTimePeriod = 20000;       // period of no motion to shutdown/stop logging (currently 20 seconds)
double motionTimePeriod = 10*60*1000;       // period of no motion to shutdown/stop logging (currently 10 minutes)
bool wdt_int_occured = false;
uint8_t wakeUpCause = 0;
bool sleeping = false;
bool softWakeUp = false;
void midCycleSleep(uint8_t counterPreload);
uint8_t counterPreload = 255;
bool timerInterrupt = false;


//---------rtc-----------------//
RTC_DS1307 rtc;
// a variable to reference miliseconds to to have a millisecond counter relative to the hour
double millisAtTopHour = 0;
int prevHour;
bool relMillisSet = false;
long milliseconds;
long millisecondsFIFORead;
uint8_t currentMinute = 0;
uint8_t previousMinute = 0;
uint32_t currentMillisecond = 0;
uint8_t MINUTE,SECOND;

DateTime Now;
String Day;
String Month;
String Hour;

//-------- Reed Switch Encoder ----------//
// 2 reed switches are used
//input pull up applied so 1 is no read; 0 is magnet read

// 2nd prototype    (uses white cable with the reed switch lines)
const int reedLeft[2] = {0,7};      // 7 is white wire
const int reedRight[2] = {5,4};     // 5 is white wire

//1st prototype     (uses grean and white, blach, blue mix for reed switch lines)
//const int reedRight[2] = {6,7};
//const int reedLeft[2] = {4,5};

bool reed_stat_left[2] = {1,1};
bool reed_stat_right[2] = {1,1};

// will help the logic with the reed switch states
typedef enum {UNDETERMINED, FORWARD, REVERSE, TRIGGER_FORWARD, TRIGGER_REVERSE, RELEASE_FORWARD, RELEASE_REVERSE} direction;

direction WheelLeft = UNDETERMINED;
direction WheelRight = UNDETERMINED;
direction ReedTriggeredLeft = UNDETERMINED;
direction ReedTriggeredRight = UNDETERMINED;

// reed swithces ISR's
void reedRightIntFirst();
void reedRightIntSecond();

void reedLeftIntFirst();
void reedLeftIntSecond();

double reedRightTime[2] = {0,0};
double reedLeftTime[2] = {0,0};
double reedRightTimePrev[2] = {0,0};
double reedLeftTimePrev[2] = {0,0};
const double minumumReedTimeInterrupt = 10;   // ms

// The 4 possible states of the 2 reed switches per side
typedef enum {NONE, FIRST, BOTH, LAST} state;
state EncoderLeft = NONE;
state EncoderRight = NONE;

// rotation counters
int WheelLeftRot = 0;
int WheelRightRot = 0;

float wheelResolution = 4;    // how many magnets on the wheel.

//pot
int potVal;
int potAngle;

//----------- Setup ----------------/
void setup()
{
    //trigger reset on start just in case
    //wdt_int_reset();

    pinMode(13, OUTPUT);
    pinMode(reedLeft[0], INPUT_PULLUP);
    pinMode(reedLeft[1], INPUT_PULLUP);
    pinMode(reedRight[0], INPUT_PULLUP);
    pinMode(reedRight[1], INPUT_PULLUP);
    
    Wire.begin();
    SPI.begin();

    
    
#ifdef ECHO
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
#endif
#ifdef POT
    pinMode(A0,INPUT);
#endif
    
    //---------- Set up rtc --------------------//
    rtc.begin();
    
    if (!rtc.isrunning()) {
        // following line sets the rtc to the date & time this sketch was compiled
        rtc.adjust(DateTime(__DATE__, __TIME__));
    }
    
#ifdef SET_rtc
    rtc.adjust(DateTime(__DATE__, __TIME__));
    //rtc.adjust(DateTime(2016,8,9,15,59,0));
#endif
    
    
    //--------IMU initialization------------//
#ifdef  ECHO
    Serial.println("Initializing MPU & Calibrating Offsets");
#endif
    //IMU int pin ISR for motion detection
    attachInterrupt(3,IMU_int, RISING); // use interrupt 3(pin 3) and run function
    
    //Reed Switch interrupts
    attachInterrupt(reedRight[0],reedRightIntFirst, CHANGE); // rising is when the switch opens up
    attachInterrupt(reedRight[1],reedRightIntSecond, CHANGE); // rising is when the switch opens up
    
    attachInterrupt(reedLeft[0],reedLeftIntFirst, CHANGE); // rising is when the switch opens up
    attachInterrupt(reedLeft[1],reedLeftIntSecond, CHANGE); // rising is when the switch opens up
    
    IMU1.initialize();
    IMU2.initialize();
    
    // check the who am i register to see if the IMU is responding
    if(IMU1.testConnection()) {
        IMU1_online = true;
        IMU1.calibrateGyro();   //DONT move the MPU when Calibrating
        IMU1.initialize();      // reinitialize to reset sample rate dividers and DLPFs....
        IMU1.setFullScaleGyroRange(MPU9250_GYRO_FS_250);
        IMU1.setFullScaleAccelRange(MPU9250_ACCEL_FS_2);
        IMU1.setRate((uint8_t)SAMPLE_RATE_DIV);
        //IMU1.setDataRdyInt();
    }
    else {
        IMU1_online = false;
    }
    
    // check the who am i register to see if the IMU is responding
    if(IMU2.testConnection()) {
        IMU2_online = true;
        IMU2.calibrateGyro();   //DONT move the MPU when Calibrating
        IMU2.initialize();      // reinitialize to reset sample rate dividers and DLPFs....
        IMU2.setFullScaleGyroRange(MPU9250_GYRO_FS_250);
        IMU2.setFullScaleAccelRange(MPU9250_ACCEL_FS_2);
        IMU2.setRate((uint8_t)SAMPLE_RATE_DIV);
        //IMU2.setDataRdyInt(); //sets the INT pin on the MPU to send an interrupt when it senses motion
    }
    else {
        IMU2_online = false;
    }
    
    
    //--------- Set up Logging -----------------//
    
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        SD_card_present = false;
    }
    else {
        SD_card_present = true;
    }
    
    
    // create a new file
    Now = rtc.now();
    Day = String(Now.day());
    Month = String(Now.month());
    Hour = String(Now.hour());
    
    filenameString = String(Month+"-"+Day+"-"+Hour);
    filenameString.toCharArray(filename, sizeof(filename));
    logfile = SD.open(filename, FILE_WRITE);
    
    
    // ------------ Screen output ------------------//
#ifdef ECHO
    
    //rtc Flags
    if (!rtc.isrunning()) {
        Serial.println("rtc is not set... setting to this compile time and date");
        // following line sets the rtc to the date & time this sketch was compiled
        rtc.adjust(DateTime(__DATE__, __TIME__));
    }
    else {
        Serial.println("rtc Running and Set");
    }
    
    // IMU 1 online flag
    if(IMU1_online) {
        Serial.println("MPU_9250 1 Online");
    }
    else {
        Serial.println("MPU_9250 1 not responding");
    }
    
    // IMU 2 online flag
    if(IMU2_online) {
        Serial.println("MPU_9250 2 Online");
    }
    else {
        Serial.println("MPU_9250 2 not responding");
    }
    
    // Check for SD card flags
    if(SD_card_present) {
        Serial.println("SD Card Online");
    }
    else {
        Serial.println("No SD Card");
    }
    
    delay(5000);
    
#endif
    
    
    // -------------- Initialize WDT and reset-------------------//
    initTimers();
    wdtInit();
    wdtClear();
}

//----------- loop ----------------//
void loop()
{
    
    begin_of_loop = millis();
    if(!dataRdyIntSet) {
        IMU1.setDataRdyInt();
        IMU2.setDataRdyInt(); //sets the INT pin on the MPU to send an interrupt when it senses motion
        dataRdyIntSet = true;
    }
    
    //----------- IMU Get Data & process ----------------//
    
    if(IMU1_online && gyroDataRdy) {
        IMU1.getMotion6(&Acc_raw_right[0], &Acc_raw_right[1], &Acc_raw_right[2], &Gyro_raw_right[0], &Gyro_raw_right[1], &Gyro_raw_right[2]);
        
        Acc_right[0] = Acc_raw_right[0] * 2.0f/32768.0f; // 2 g full range for accelerometer
        Acc_right[1] = Acc_raw_right[1] * 2.0f/32768.0f; // 2 g full range for accelerometer
        Acc_right[2] = Acc_raw_right[2] * 2.0f/32768.0f; // 2 g full range for accelerometer
        
        Gyro_right[0] = Gyro_raw_right[0] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        Gyro_right[1] = Gyro_raw_right[1] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        Gyro_right[2] = Gyro_raw_right[2] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        
        
        Gyro_right = Gyro_right * PI/180.0f;    //convert to radians
        
        //update quanternion & yaw pitch Roll vector
        //Attitude_right_lever = MadgwickAHRSupdateIMU(Gyro_right, Acc_right, quanternion_right);
        MadgwickAHRSupdateIMU(Gyro_right, Acc_right, quanternion_right, Attitude_right_lever);
        
    }
    
    if(IMU2_online && gyroDataRdy) {
        IMU2.getMotion6(&Acc_raw_left[0], &Acc_raw_left[1], &Acc_raw_left[2], &Gyro_raw_left[0], &Gyro_raw_left[1], &Gyro_raw_left[2]);
        
        Acc_left[0] = Acc_raw_left[0] * 2.0f/32768.0f; // 2 g full range for accelerometer
        Acc_left[1] = Acc_raw_left[1] * 2.0f/32768.0f; // 2 g full range for accelerometer
        Acc_left[2] = Acc_raw_left[2] * 2.0f/32768.0f; // 2 g full range for accelerometer
        
        Gyro_left[0] = Gyro_raw_left[0] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        Gyro_left[1] = Gyro_raw_left[1] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        Gyro_left[2] = Gyro_raw_left[2] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        
        Gyro_left = Gyro_left * PI/180.0f;    //convert to radians
        
        //update quanternion & yaw pitch Roll vector
        MadgwickAHRSupdateIMU(Gyro_left, Acc_left, quanternion_left, Attitude_left_lever);
        
    }
     
    
    // pot
#ifdef POT
    potVal = analogRead(A0);
    potAngle = map(potVal, 200, 445, 0, 90);
#endif
        
    //------------------ SD Card Logging -------------------------------------//
    
#ifdef LOG
    
    // if the file is available, write to it:
    if (SD_card_present && gyroDataRdy) {
        Now = rtc.now();
        MINUTE = Now.minute();
        SECOND = Now.second();
        
        // reset millis every hour
        if(Now.hour() != prevHour) {
            //upon a reset not at top of hour, will get a millisecond counter
            
            millisAtTopHour = (long)(Now.minute()*60000) + (long)(Now.second() * 1000);
            millisAtTopHour = -millisAtTopHour;
            
            prevHour = Now.hour();
        }
        
        //---------- Fill up the buffer ---------------------//
        
        milliseconds = (uint32_t)millis();
        
        // Begin building the buffer array
        buffer[writeCounter+0] = MINUTE;
        buffer[writeCounter+1] = SECOND;
        
        //split the long into 4 bytes little endian
        buffer[writeCounter+5] = milliseconds >> 24 & 0xFF;  //MSB
        buffer[writeCounter+4] = milliseconds >> 16 & 0xFF;  //CMSB
        buffer[writeCounter+3] = milliseconds >> 8 & 0xFF;  //CLSB
        buffer[writeCounter+2] = milliseconds & 0xFF;  //LSB
        
        //----------- IMU Data -------------------//
        if (IMU1_online) {
            // lever angle from madgwick algorithm
            buffer[writeCounter+6] = (int)Attitude_right_lever[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+7] = (int)Attitude_right_lever[1] & 0xFF;        //LSB
            
            //raw gyro
            buffer[writeCounter+8] = Gyro_raw_right[0] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+9] = Gyro_raw_right[0] & 0xFF;        //LSB
            buffer[writeCounter+10] = Gyro_raw_right[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+11] = Gyro_raw_right[1] & 0xFF;        //LSB
            buffer[writeCounter+12] = Gyro_raw_right[2] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+13] = Gyro_raw_right[2] & 0xFF;        //LSB
            
            //raw acc
            buffer[writeCounter+14] = Acc_raw_right[0] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+15] = Acc_raw_right[0] & 0xFF;        //LSB
            buffer[writeCounter+16] = Acc_raw_right[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+17] = Acc_raw_right[1] & 0xFF;        //LSB
            buffer[writeCounter+18] = Acc_raw_right[2] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+19] = Acc_raw_right[2] & 0xFF;        //LSB
        }
        else {
            // lever angle from madgwick algorithm
            buffer[writeCounter+6] = 0xFF;   // MSB
            buffer[writeCounter+7] = 0xFF;        //LSB
            
            //raw gyro
            buffer[writeCounter+8] = 0xFF;   // MSB
            buffer[writeCounter+9] = 0xFF;        //LSB
            buffer[writeCounter+10] = 0xFF;   // MSB
            buffer[writeCounter+11] = 0xFF;        //LSB
            buffer[writeCounter+12] = 0xFF;   // MSB
            buffer[writeCounter+13] = 0xFF;        //LSB
            
            //raw acc
            buffer[writeCounter+14] = 0xFF;   // MSB
            buffer[writeCounter+15] = 0xFF;        //LSB
            buffer[writeCounter+16] = 0xFF;   // MSB
            buffer[writeCounter+17] = 0xFF;        //LSB
            buffer[writeCounter+18] = 0xFF;   // MSB
            buffer[writeCounter+19] = 0xFF;        //LSB
        }
        
        if (IMU2_online) {
            // lever angle from madgwick algorithm
            buffer[writeCounter+20] = (int)Attitude_left_lever[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+21] = (int)Attitude_left_lever[1] & 0xFF;        //LSB
            
            //raw gyro
            buffer[writeCounter+22] = Gyro_raw_left[0] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+23] = Gyro_raw_left[0] & 0xFF;        //LSB
            buffer[writeCounter+24] = Gyro_raw_left[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+25] = Gyro_raw_left[1] & 0xFF;        //LSB
            buffer[writeCounter+26] = Gyro_raw_left[2] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+27] = Gyro_raw_left[2] & 0xFF;        //LSB
            
            //raw acc
            buffer[writeCounter+28] = Acc_raw_left[0] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+29] = Acc_raw_left[0] & 0xFF;        //LSB
            buffer[writeCounter+30] = Acc_raw_left[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+31] = Acc_raw_left[1] & 0xFF;        //LSB
            buffer[writeCounter+32] = Acc_raw_left[2] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+33] = Acc_raw_left[2] & 0xFF;        //LSB
        }
        else {
            // lever angle from madgwick algorithm
            buffer[writeCounter+20] = 0xFF;   // MSB
            buffer[writeCounter+21] = 0xFF;        //LSB
            
            //raw gyro
            buffer[writeCounter+22] = 0xFF;   // MSB
            buffer[writeCounter+23] = 0xFF;        //LSB
            buffer[writeCounter+24] = 0xFF;   // MSB
            buffer[writeCounter+25] = 0xFF;        //LSB
            buffer[writeCounter+26] = 0xFF;   // MSB
            buffer[writeCounter+27] = 0xFF;        //LSB
            
            //raw acc
            buffer[writeCounter+28] = 0xFF;   // MSB
            buffer[writeCounter+29] = 0xFF;        //LSB
            buffer[writeCounter+30] = 0xFF;   // MSB
            buffer[writeCounter+31] = 0xFF;        //LSB
            buffer[writeCounter+32] = 0xFF;   // MSB
            buffer[writeCounter+33] = 0xFF;        //LSB
        }
        
        //------------- Reed Data ----------------//
        //raw reed data and counter
        buffer[writeCounter+34] = reed_stat_left[0] >> 8 & 0xFF;   // MSB
        buffer[writeCounter+35] = reed_stat_left[0] & 0xFF;        //LSB
        buffer[writeCounter+36] = reed_stat_left[1] >> 8 & 0xFF;   // MSB
        buffer[writeCounter+37] = reed_stat_left[1] & 0xFF;        //LSB
        buffer[writeCounter+38] = WheelLeftRot >> 8 & 0xFF;       // MSB
        buffer[writeCounter+39] = WheelLeftRot & 0xFF;            //LSB
        
        buffer[writeCounter+40] = reed_stat_right[0] >> 8 & 0xFF;   // MSB
        buffer[writeCounter+41] = reed_stat_right[0] & 0xFF;        //LSB
        buffer[writeCounter+42] = reed_stat_right[1] >> 8 & 0xFF;   // MSB
        buffer[writeCounter+43] = reed_stat_right[1] & 0xFF;        //LSB
        buffer[writeCounter+44] = WheelRightRot >> 8 & 0xFF;       // MSB
        buffer[writeCounter+45] = WheelRightRot & 0xFF;            //LSB
        
        // ----------- Sample Rate ------------//
        buffer[writeCounter+46] = (int)measured_cycle_rate >> 8 & 0xFF;   // MSB
        buffer[writeCounter+47] = (int)measured_cycle_rate & 0xFF;        //LSB
        
        // ----------- pot angle ------------//
        buffer[writeCounter+48] = potAngle >> 8 & 0xFF;   // MSB
        buffer[writeCounter+49] = potAngle & 0xFF;        //LSB
        
        writeCounter += dataBlock; //increase the counter
    
        
        
        // Hour transition handling
        currentMinute = Now.minute();
        if (previousMinute > currentMinute ) {      // minute 59 goes to 0
            // transition occured, write all data to old file
            bytes_written = logfile.write(buffer,(writeCounter-dataBlock));  //dont write last packet since its past the hour;
            logfile.close();
            
            //move the unwritten part of the block to the front of the buffer
            for (int j=0; j<dataBlock; j++) {
                buffer[j] = buffer[(writeCounter-dataBlock) + j];
            }
            writeCounter = dataBlock;   //restart the counter at block 2
            
            // open new file
            Now = rtc.now();
            Day = String(Now.day());
            Month = String(Now.month());
            Hour = String(Now.hour());
            
            filenameString = String(Month+"-"+Day+"-"+Hour);
            filenameString.toCharArray(filename, sizeof(filename));
            
            logfile = SD.open(filename, FILE_WRITE);
        }
        previousMinute = currentMinute;
        
        //write the binary data if buffer is full or we are sleeping write every wake up
        if (writeCounter >= bufferSize || sleeping) {
            // turn on and reset FIFO module
            IMU1.writeSPI(MPU9250_RA_USER_CTRL, 0b01010000);	//fifo enable
            IMU1.writeSPI(MPU9250_RA_USER_CTRL, 0b01010100);	//reset the fifo
            
            IMU2.writeSPI(MPU9250_RA_USER_CTRL, 0b01010000);	//fifo enable
            IMU2.writeSPI(MPU9250_RA_USER_CTRL, 0b01010100);	//reset the fifo
            
            while(!gyroDataRdy);                                //wait for data ready interrupt to sync
            gyroDataRdy = false;
            millisecondsFIFORead = (uint32_t)millis();  //timestamp
            
            
            IMU1.writeSPI(MPU9250_RA_FIFO_EN, 0b01111000);      //gyro and acc values
            IMU2.writeSPI(MPU9250_RA_FIFO_EN, 0b01111000);      //gyro and acc values
            
            
            bytes_written = logfile.write(buffer,bufferSize);
            //write bytes
            logfile.flush();
            writeCounter = 0;
            
            while(!gyroDataRdy);                                //wait for data ready interrupt to sync
            gyroDataRdy = false;
            
            //disable FIFO once we are done writing to SD card
            IMU1.writeSPI(MPU9250_RA_FIFO_EN, 0x00);        // Disable gyro and acc storage for FIFO
            IMU2.writeSPI(MPU9250_RA_FIFO_EN, 0x00);        // Disable gyro and acc storage for FIFO
            
            //get the FIFO count
            fifoCount[0] = IMU1.getFIFOCount();
            fifoCount[1] = IMU2.getFIFOCount();
            
            Remainder[0] = fifoCount[0]%12;     //see if we did not capture a full packet
            Remainder[1] = fifoCount[1]%12;     //see if we did not capture a full packet
            
            numberPackets[0] = (fifoCount[0] - Remainder[0])/12;
            numberPackets[1] = (fifoCount[1] - Remainder[1])/12;
            
#ifdef ECHO
            if(Remainder[0] > 0 || Remainder[1] > 0) {
                Serial.println(Remainder[0]);
                Serial.println(numberPackets[0]);
                Serial.println(Remainder[1]);
                Serial.println(numberPackets[1]);
                delay(1000);
            }
#endif
            
            int ii;
            for(ii = 0; ii<Remainder[0]; ii++) {
                //skip the partial packet
                //invoke read
                IMU1.getFIFOCount();
            }
            int iii;
            for(iii = 0; iii<Remainder[1]; iii++) {
                //skip the partial packet
                //invoke read
                IMU2.getFIFOCount();
            }
            
            // if they are not equal, equate them to the smallest packet so re reads wont occur
            if(numberPackets[1] < numberPackets[0]) {
                numberPackets[0] = numberPackets[1];
            }
            if(numberPackets[0] < numberPackets[1]) {
                numberPackets[1] = numberPackets[0];
            }
            
            // re set up the buffer
            uint8_t j;
            for(j = 0;j<numberPackets[1];j++) {
                //right lever
                IMU1.getMotion6FIFO(&Acc_raw_right[0], &Acc_raw_right[1], &Acc_raw_right[2], &Gyro_raw_right[0], &Gyro_raw_right[1], &Gyro_raw_right[2]);
                
                Acc_right[0] = Acc_raw_right[0] * 2.0f/32768.0f; // 2 g full range for accelerometer
                Acc_right[1] = Acc_raw_right[1] * 2.0f/32768.0f; // 2 g full range for accelerometer
                Acc_right[2] = Acc_raw_right[2] * 2.0f/32768.0f; // 2 g full range for accelerometer
                
                Gyro_right[0] = Gyro_raw_right[0] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
                Gyro_right[1] = Gyro_raw_right[1] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
                Gyro_right[2] = Gyro_raw_right[2] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
                
                
                Gyro_right = Gyro_right * PI/180.0f;    //convert to radians
                
                //update quanternion & yaw pitch Roll vector
                //Attitude_right_lever = MadgwickAHRSupdateIMU(Gyro_right, Acc_right, quanternion_right);
                MadgwickAHRSupdateIMU(Gyro_right, Acc_right, quanternion_right, Attitude_right_lever);
                
                //left lever
                IMU2.getMotion6FIFO(&Acc_raw_left[0], &Acc_raw_left[1], &Acc_raw_left[2], &Gyro_raw_left[0], &Gyro_raw_left[1], &Gyro_raw_left[2]);
                
                Acc_left[0] = Acc_raw_left[0] * 2.0f/32768.0f; // 2 g full range for accelerometer
                Acc_left[1] = Acc_raw_left[1] * 2.0f/32768.0f; // 2 g full range for accelerometer
                Acc_left[2] = Acc_raw_left[2] * 2.0f/32768.0f; // 2 g full range for accelerometer
                
                Gyro_left[0] = Gyro_raw_left[0] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
                Gyro_left[1] = Gyro_raw_left[1] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
                Gyro_left[2] = Gyro_raw_left[2] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
                
                Gyro_left = Gyro_left * PI/180.0f;    //convert to radians
                
                //update quanternion & yaw pitch Roll vector
                MadgwickAHRSupdateIMU(Gyro_left, Acc_left, quanternion_left, Attitude_left_lever);
                
                //---------- Fill up the buffer ---------------------//
                //fix the time stamp for the previous data packets
                //milliseconds = millisecondsFIFORead - (numberPackets[0] - j)*(uint8_t)(1000/gyroSampleFreq);
                milliseconds = millisecondsFIFORead + j*(uint8_t)(1000/gyroSampleFreq);
                
                
                
                // Begin building the buffer array
                buffer[writeCounter+0] = MINUTE;
                buffer[writeCounter+1] = SECOND;
                
                //split the long into 4 bytes little endian
                buffer[writeCounter+5] = milliseconds >> 24 & 0xFF;  //MSB
                buffer[writeCounter+4] = milliseconds >> 16 & 0xFF;  //CMSB
                buffer[writeCounter+3] = milliseconds >> 8 & 0xFF;  //CLSB
                buffer[writeCounter+2] = milliseconds & 0xFF;  //LSB
                
                //----------- IMU Data -------------------//
                if (IMU1_online) {
                    // lever angle from madgwick algorithm
                    buffer[writeCounter+6] = (int)Attitude_right_lever[1] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+7] = (int)Attitude_right_lever[1] & 0xFF;        //LSB
                    
                    //raw gyro
                    buffer[writeCounter+8] = Gyro_raw_right[0] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+9] = Gyro_raw_right[0] & 0xFF;        //LSB
                    buffer[writeCounter+10] = Gyro_raw_right[1] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+11] = Gyro_raw_right[1] & 0xFF;        //LSB
                    buffer[writeCounter+12] = Gyro_raw_right[2] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+13] = Gyro_raw_right[2] & 0xFF;        //LSB
                    
                    //raw acc
                    buffer[writeCounter+14] = Acc_raw_right[0] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+15] = Acc_raw_right[0] & 0xFF;        //LSB
                    buffer[writeCounter+16] = Acc_raw_right[1] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+17] = Acc_raw_right[1] & 0xFF;        //LSB
                    buffer[writeCounter+18] = Acc_raw_right[2] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+19] = Acc_raw_right[2] & 0xFF;        //LSB
                }
                else {
                    // lever angle from madgwick algorithm
                    buffer[writeCounter+6] = 0xFF;   // MSB
                    buffer[writeCounter+7] = 0xFF;        //LSB
                    
                    //raw gyro
                    buffer[writeCounter+8] = 0xFF;   // MSB
                    buffer[writeCounter+9] = 0xFF;        //LSB
                    buffer[writeCounter+10] = 0xFF;   // MSB
                    buffer[writeCounter+11] = 0xFF;        //LSB
                    buffer[writeCounter+12] = 0xFF;   // MSB
                    buffer[writeCounter+13] = 0xFF;        //LSB
                    
                    //raw acc
                    buffer[writeCounter+14] = 0xFF;   // MSB
                    buffer[writeCounter+15] = 0xFF;        //LSB
                    buffer[writeCounter+16] = 0xFF;   // MSB
                    buffer[writeCounter+17] = 0xFF;        //LSB
                    buffer[writeCounter+18] = 0xFF;   // MSB
                    buffer[writeCounter+19] = 0xFF;        //LSB
                }
                
                if (IMU2_online) {
                    // lever angle from madgwick algorithm
                    buffer[writeCounter+20] = (int)Attitude_left_lever[1] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+21] = (int)Attitude_left_lever[1] & 0xFF;        //LSB
                    
                    //raw gyro
                    buffer[writeCounter+22] = Gyro_raw_left[0] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+23] = Gyro_raw_left[0] & 0xFF;        //LSB
                    buffer[writeCounter+24] = Gyro_raw_left[1] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+25] = Gyro_raw_left[1] & 0xFF;        //LSB
                    buffer[writeCounter+26] = Gyro_raw_left[2] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+27] = Gyro_raw_left[2] & 0xFF;        //LSB
                    
                    //raw acc
                    buffer[writeCounter+28] = Acc_raw_left[0] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+29] = Acc_raw_left[0] & 0xFF;        //LSB
                    buffer[writeCounter+30] = Acc_raw_left[1] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+31] = Acc_raw_left[1] & 0xFF;        //LSB
                    buffer[writeCounter+32] = Acc_raw_left[2] >> 8 & 0xFF;   // MSB
                    buffer[writeCounter+33] = Acc_raw_left[2] & 0xFF;        //LSB
                }
                else {
                    // lever angle from madgwick algorithm
                    buffer[writeCounter+20] = 0xFF;   // MSB
                    buffer[writeCounter+21] = 0xFF;        //LSB
                    
                    //raw gyro
                    buffer[writeCounter+22] = 0xFF;   // MSB
                    buffer[writeCounter+23] = 0xFF;        //LSB
                    buffer[writeCounter+24] = 0xFF;   // MSB
                    buffer[writeCounter+25] = 0xFF;        //LSB
                    buffer[writeCounter+26] = 0xFF;   // MSB
                    buffer[writeCounter+27] = 0xFF;        //LSB
                    
                    //raw acc
                    buffer[writeCounter+28] = 0xFF;   // MSB
                    buffer[writeCounter+29] = 0xFF;        //LSB
                    buffer[writeCounter+30] = 0xFF;   // MSB
                    buffer[writeCounter+31] = 0xFF;        //LSB
                    buffer[writeCounter+32] = 0xFF;   // MSB
                    buffer[writeCounter+33] = 0xFF;        //LSB
                }
                
                //------------- Reed Data ----------------//
                //raw reed data and counter
                buffer[writeCounter+34] = reed_stat_left[0] >> 8 & 0xFF;   // MSB
                buffer[writeCounter+35] = reed_stat_left[0] & 0xFF;        //LSB
                buffer[writeCounter+36] = reed_stat_left[1] >> 8 & 0xFF;   // MSB
                buffer[writeCounter+37] = reed_stat_left[1] & 0xFF;        //LSB
                buffer[writeCounter+38] = WheelLeftRot >> 8 & 0xFF;       // MSB
                buffer[writeCounter+39] = WheelLeftRot & 0xFF;            //LSB
                
                buffer[writeCounter+40] = reed_stat_right[0] >> 8 & 0xFF;   // MSB
                buffer[writeCounter+41] = reed_stat_right[0] & 0xFF;        //LSB
                buffer[writeCounter+42] = reed_stat_right[1] >> 8 & 0xFF;   // MSB
                buffer[writeCounter+43] = reed_stat_right[1] & 0xFF;        //LSB
                buffer[writeCounter+44] = WheelRightRot >> 8 & 0xFF;       // MSB
                buffer[writeCounter+45] = WheelRightRot & 0xFF;            //LSB
                
                // ----------- Sample Rate ------------//
                buffer[writeCounter+46] = (int)measured_cycle_rate >> 8 & 0xFF;   // MSB
                buffer[writeCounter+47] = (int)measured_cycle_rate & 0xFF;        //LSB
                
                // ----------- pot angle ------------//
                buffer[writeCounter+48] = potAngle >> 8 & 0xFF;   // MSB
                buffer[writeCounter+49] = potAngle & 0xFF;        //LSB
                
                writeCounter += dataBlock; //increase the counter
            }//end of for
        }
            
    }//end of if
    else {
        //Serial.println("data not ready");
    }
    gyroDataRdy = false;        //reset new data flag
    
    
#endif //log
    
#ifdef ECHO
    
    if(IMU1_online && rtc.isrunning()) {
        
        Serial.print("lever angle right: ");
        Serial.print(Attitude_right_lever[1]);
    }
    if (IMU2_online && rtc.isrunning() ) {
        Serial.print("\t lever angle left: ");
        Serial.print(Attitude_left_lever[1]);
    }
    
    Serial.print("\t gz right: ");
    Serial.print(Gyro_raw_right[2]);
    Serial.print("\t ");
    
    Serial.print("\t gz left: ");
    Serial.print(Gyro_raw_left[2]);
    Serial.print("\t ");
    
#ifdef POT
    Serial.print("\t Pot: ");
    Serial.print(potAngle);
    Serial.print("\t ");
#endif
    /*
    Serial.print("Left rot: ");
    Serial.print(WheelLeftRot);
    
    Serial.print("\t Right rot: ");
    Serial.print(WheelRightRot);
    
    Serial.print("\t right direction: ");
    Serial.print(WheelRight);
    Serial.print("Left Direction: ");
    Serial.print(WheelLeft);
    
    Serial.print("\t Right reed 1: ");
    Serial.print(reed_stat_right[0]);
    Serial.print("\t Right reed 2: ");
    Serial.print(reed_stat_right[1]);
    
    Serial.print("\t left reed 1: ");
    Serial.print(reed_stat_left[0]);
    Serial.print("\t left reed 2: ");
    Serial.print(reed_stat_left[1]);
     */
    /*
     Serial.print("q right: ");
     Serial.print(quanternion_right[0]);
     Serial.print("\t ");
     Serial.print(quanternion_right[1]);
     Serial.print("\t ");
     Serial.print(quanternion_right[2]);
     Serial.print("\t ");
     Serial.print(quanternion_right[3]);
     
     Serial.print("\t q left: ");
     Serial.print(quanternion_left[0]);
     Serial.print("\t ");
     Serial.print(quanternion_left[1]);
     Serial.print("\t ");
     Serial.print(quanternion_left[2]);
     Serial.print("\t ");
     Serial.print(quanternion_left[3]);
     */
    
   // Serial.print("\t ");
   // Serial.print(mpu_int_triggered);
   // Serial.print("\t ");
    Serial.println(measured_cycle_rate);
    
    
    /*
     DateTime now = rtc.now();
     
     Serial.print(now.year(), DEC);
     Serial.print('/');
     Serial.print(now.month(), DEC);
     Serial.print('/');
     Serial.print(now.day(), DEC);
     Serial.print(' ');
     Serial.print(now.hour(), DEC);
     Serial.print(':');
     Serial.print(now.minute(), DEC);
     Serial.print(':');
     Serial.print(now.second(), DEC);
     Serial.println();
     */
    
#endif //echo
    
    //sample rate handling
    
    loop_time = millis() - begin_of_loop;     //milliseconds
    measured_cycle_rate = 1000*(1/loop_time);   //hz
    
    //if we are going too fast delay
    if (measured_cycle_rate > sampleFreq) {
        delay_time = ((float)(1/sampleFreq)*1000) - loop_time;
        
        counterPreload = 256 - delay_time*(16);     // 32khz clock and delay time is in ms
        //Serial.println(counterPreload);
        //midCycleSleep(counterPreload);              //sleep off excess time
        delay(delay_time);
    }
    // remeasure
    loop_time = millis() - begin_of_loop;     //milliseconds
    measured_cycle_rate = 1000*(1/loop_time);   //hz

    // ----------- Sleep Handeling -------------//
    // MPU ISR flag handeling
    
    if(IMU1_online) {
        // Motion sensing processing
        if (mpu_int_triggered) {
            motionTimer = millis();
            //Serial.println("IMU interrupt occured");
            //delay(1000);
        }
        
        //reset the trigger if it occured
        mpu_int_triggered = false;
        
        if ((millis() - motionTimer) >= motionTimePeriod) {
            //sleepNow();
        }
        
    }
    if(wdt_int_occured) {
        //Serial.println("wdt int occured!");
        //delay(1000);
        wdt_int_occured = false;
    }
    if(timerInterrupt) {
        timerInterrupt = false;
        //Serial.println("timer int occured!");
    }
    /*wakeUpCause = PM->RCAUSE.reg;
    Serial.println(wakeUpCause);
    delay(1000);*/
    wdtClear();
    
    
}   // END OF LOOP



// ---------------------- ISRs --------------------------//
// IMU int pin ISR
void IMU_int() {
    // timers will not work in an ISR
    // So just set a flag
    mpu_int_triggered = true;
    gyroDataRdy = true;
    //Serial.println("in interrupt");
    /*
    if(sleeping){
        //wake up and set the interrupt pin to data ready interrupt
        sleeping = false;
        IMU1.setDataRdyInt();
        IMU2.setDataRdyInt();
    }
    else {
        //poll data
        //IMU 1
        IMU1.getMotion6(&Acc_raw_right[0], &Acc_raw_right[1], &Acc_raw_right[2], &Gyro_raw_right[0], &Gyro_raw_right[1], &Gyro_raw_right[2]);
        
        Acc_right[0] = Acc_raw_right[0] * 2.0f/32768.0f; // 2 g full range for accelerometer
        Acc_right[1] = Acc_raw_right[1] * 2.0f/32768.0f; // 2 g full range for accelerometer
        Acc_right[2] = Acc_raw_right[2] * 2.0f/32768.0f; // 2 g full range for accelerometer
        
        Gyro_right[0] = Gyro_raw_right[0] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        Gyro_right[1] = Gyro_raw_right[1] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        Gyro_right[2] = Gyro_raw_right[2] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        
        
        Gyro_right = Gyro_right * PI/180.0f;    //convert to radians
        
        //update quanternion & yaw pitch Roll vector
        MadgwickAHRSupdateIMU(Gyro_right, Acc_right, quanternion_right, Attitude_right_lever);
        
        //imu 2
        IMU2.getMotion6(&Acc_raw_left[0], &Acc_raw_left[1], &Acc_raw_left[2], &Gyro_raw_left[0], &Gyro_raw_left[1], &Gyro_raw_left[2]);
        
        Acc_left[0] = Acc_raw_left[0] * 2.0f/32768.0f; // 2 g full range for accelerometer
        Acc_left[1] = Acc_raw_left[1] * 2.0f/32768.0f; // 2 g full range for accelerometer
        Acc_left[2] = Acc_raw_left[2] * 2.0f/32768.0f; // 2 g full range for accelerometer
        
        Gyro_left[0] = Gyro_raw_left[0] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        Gyro_left[1] = Gyro_raw_left[1] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        Gyro_left[2] = Gyro_raw_left[2] * 250.0f/32768.0f; // 250 deg/s full range for gyroscope
        
        Gyro_left = Gyro_left * PI/180.0f;    //convert to radians
        
        //update quanternion & yaw pitch Roll vector
        MadgwickAHRSupdateIMU(Gyro_left, Acc_left, quanternion_left, Attitude_left_lever);
        
        //---------- Fill up the buffer ---------------------//
        
        milliseconds = (uint32_t)millis();
        
        // Begin building the buffer array
        buffer[writeCounter+0] = MINUTE;
        buffer[writeCounter+1] = SECOND;
        
        //split the long into 4 bytes little endian
        buffer[writeCounter+5] = milliseconds >> 24 & 0xFF;  //MSB
        buffer[writeCounter+4] = milliseconds >> 16 & 0xFF;  //CMSB
        buffer[writeCounter+3] = milliseconds >> 8 & 0xFF;  //CLSB
        buffer[writeCounter+2] = milliseconds & 0xFF;  //LSB
        
        //----------- IMU Data -------------------//
        if (IMU1_online) {
            // lever angle from madgwick algorithm
            buffer[writeCounter+6] = (int)Attitude_right_lever[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+7] = (int)Attitude_right_lever[1] & 0xFF;        //LSB
            
            //raw gyro
            buffer[writeCounter+8] = Gyro_raw_right[0] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+9] = Gyro_raw_right[0] & 0xFF;        //LSB
            buffer[writeCounter+10] = Gyro_raw_right[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+11] = Gyro_raw_right[1] & 0xFF;        //LSB
            buffer[writeCounter+12] = Gyro_raw_right[2] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+13] = Gyro_raw_right[2] & 0xFF;        //LSB
            
            //raw acc
            buffer[writeCounter+14] = Acc_raw_right[0] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+15] = Acc_raw_right[0] & 0xFF;        //LSB
            buffer[writeCounter+16] = Acc_raw_right[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+17] = Acc_raw_right[1] & 0xFF;        //LSB
            buffer[writeCounter+18] = Acc_raw_right[2] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+19] = Acc_raw_right[2] & 0xFF;        //LSB
        }
        else {
            // lever angle from madgwick algorithm
            buffer[writeCounter+6] = 0xFF;   // MSB
            buffer[writeCounter+7] = 0xFF;        //LSB
            
            //raw gyro
            buffer[writeCounter+8] = 0xFF;   // MSB
            buffer[writeCounter+9] = 0xFF;        //LSB
            buffer[writeCounter+10] = 0xFF;   // MSB
            buffer[writeCounter+11] = 0xFF;        //LSB
            buffer[writeCounter+12] = 0xFF;   // MSB
            buffer[writeCounter+13] = 0xFF;        //LSB
            
            //raw acc
            buffer[writeCounter+14] = 0xFF;   // MSB
            buffer[writeCounter+15] = 0xFF;        //LSB
            buffer[writeCounter+16] = 0xFF;   // MSB
            buffer[writeCounter+17] = 0xFF;        //LSB
            buffer[writeCounter+18] = 0xFF;   // MSB
            buffer[writeCounter+19] = 0xFF;        //LSB
        }
        
        if (IMU2_online) {
            // lever angle from madgwick algorithm
            buffer[writeCounter+20] = (int)Attitude_left_lever[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+21] = (int)Attitude_left_lever[1] & 0xFF;        //LSB
            
            //raw gyro
            buffer[writeCounter+22] = Gyro_raw_left[0] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+23] = Gyro_raw_left[0] & 0xFF;        //LSB
            buffer[writeCounter+24] = Gyro_raw_left[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+25] = Gyro_raw_left[1] & 0xFF;        //LSB
            buffer[writeCounter+26] = Gyro_raw_left[2] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+27] = Gyro_raw_left[2] & 0xFF;        //LSB
            
            //raw acc
            buffer[writeCounter+28] = Acc_raw_left[0] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+29] = Acc_raw_left[0] & 0xFF;        //LSB
            buffer[writeCounter+30] = Acc_raw_left[1] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+31] = Acc_raw_left[1] & 0xFF;        //LSB
            buffer[writeCounter+32] = Acc_raw_left[2] >> 8 & 0xFF;   // MSB
            buffer[writeCounter+33] = Acc_raw_left[2] & 0xFF;        //LSB
        }
        else {
            // lever angle from madgwick algorithm
            buffer[writeCounter+20] = 0xFF;   // MSB
            buffer[writeCounter+21] = 0xFF;        //LSB
            
            //raw gyro
            buffer[writeCounter+22] = 0xFF;   // MSB
            buffer[writeCounter+23] = 0xFF;        //LSB
            buffer[writeCounter+24] = 0xFF;   // MSB
            buffer[writeCounter+25] = 0xFF;        //LSB
            buffer[writeCounter+26] = 0xFF;   // MSB
            buffer[writeCounter+27] = 0xFF;        //LSB
            
            //raw acc
            buffer[writeCounter+28] = 0xFF;   // MSB
            buffer[writeCounter+29] = 0xFF;        //LSB
            buffer[writeCounter+30] = 0xFF;   // MSB
            buffer[writeCounter+31] = 0xFF;        //LSB
            buffer[writeCounter+32] = 0xFF;   // MSB
            buffer[writeCounter+33] = 0xFF;        //LSB
        }
        
        //------------- Reed Data ----------------//
        //raw reed data and counter
        buffer[writeCounter+34] = reed_stat_left[0] >> 8 & 0xFF;   // MSB
        buffer[writeCounter+35] = reed_stat_left[0] & 0xFF;        //LSB
        buffer[writeCounter+36] = reed_stat_left[1] >> 8 & 0xFF;   // MSB
        buffer[writeCounter+37] = reed_stat_left[1] & 0xFF;        //LSB
        buffer[writeCounter+38] = WheelLeftRot >> 8 & 0xFF;       // MSB
        buffer[writeCounter+39] = WheelLeftRot & 0xFF;            //LSB
        
        buffer[writeCounter+40] = reed_stat_right[0] >> 8 & 0xFF;   // MSB
        buffer[writeCounter+41] = reed_stat_right[0] & 0xFF;        //LSB
        buffer[writeCounter+42] = reed_stat_right[1] >> 8 & 0xFF;   // MSB
        buffer[writeCounter+43] = reed_stat_right[1] & 0xFF;        //LSB
        buffer[writeCounter+44] = WheelRightRot >> 8 & 0xFF;       // MSB
        buffer[writeCounter+45] = WheelRightRot & 0xFF;            //LSB
        
        // ----------- Sample Rate ------------//
        buffer[writeCounter+46] = (int)measured_cycle_rate >> 8 & 0xFF;   // MSB
        buffer[writeCounter+47] = (int)measured_cycle_rate & 0xFF;        //LSB
        
        writeCounter += dataBlock; //increase the counter
    }
    
    if(writeCounter >= bufferSize) {
        int iii = 0;
        //push over data to write array
        for (iii = 0; iii < bufferSize; iii++){
            dataToWrite[iii] = buffer[iii];
        }
        timeToWrite = true;
        writeCounter = 0;
        
    }
    
    Serial.println(writeCounter);
    Serial.println("end interrupt");
     */
}

//Rotary Encoder Right Wheel
void reedRightIntFirst () {
    //timestamp
    reedRightTime[0] = begin_of_loop;
    
    // read the reed switch
    reed_stat_right[0] = digitalRead(reedRight[0]);
    reed_stat_right[1] = digitalRead(reedRight[1]);
    
    //determine state of the encoder
    // 0 is when the reed switch is closed
    if (reed_stat_right[0] && reed_stat_right[1]) {
        EncoderRight = NONE;
    }
    if (!reed_stat_right[0] && reed_stat_right[1]) {
        EncoderRight = FIRST;
    }
    if (reed_stat_right[0] && !reed_stat_right[1]) {
        EncoderRight = LAST;
    }
    if (!reed_stat_right[0] && !reed_stat_right[1]) {
        EncoderRight = BOTH;
    }
    
    //This one on first
    if (WheelRight == UNDETERMINED && EncoderRight == FIRST) {
        WheelRight = TRIGGER_FORWARD;
    }
    
    // Time differential is also included to minimze debounce triggers with the mechanical reed switch
    else if(WheelRight == TRIGGER_REVERSE && EncoderRight == BOTH && reedRightTime[0] - reedRightTimePrev[0] > minumumReedTimeInterrupt) {
        WheelRight = REVERSE;
    }
    
    else if (WheelRight == FORWARD && EncoderRight == LAST) {
        WheelRight = RELEASE_FORWARD;
    }
    
    //count
    else if(WheelRight == RELEASE_REVERSE && EncoderRight ==  NONE) {
        WheelRightRot --;
        WheelRight = UNDETERMINED;
    }
    // if we get here for some reason reset
    else if (EncoderRight == NONE){
        WheelRight = UNDETERMINED;
    }
    
    reedRightTimePrev[0] = reedRightTime[0];
}
void reedRightIntSecond () {
    //timestamp
    reedRightTime[1] = begin_of_loop;
    
    // read the reed switch
    reed_stat_right[0] = digitalRead(reedRight[0]);
    reed_stat_right[1] = digitalRead(reedRight[1]);
    
    
    //determine state of the encoder
    // 0 is when the reed switch is closed
    if (reed_stat_right[0] && reed_stat_right[1]) {
        EncoderRight = NONE;
    }
    if (!reed_stat_right[0] && reed_stat_right[1]) {
        EncoderRight = FIRST;
    }
    if (reed_stat_right[0] && !reed_stat_right[1]) {
        EncoderRight = LAST;
    }
    if (!reed_stat_right[0] && !reed_stat_right[1]) {
        EncoderRight = BOTH;
    }
    
    //This one on first
    if (WheelRight == UNDETERMINED && EncoderRight == LAST) {
        WheelRight = TRIGGER_REVERSE;
    }
    
    // Time differential is also included to minimze debounce triggers with the mechanical reed switch
    else if(WheelRight == TRIGGER_FORWARD && EncoderRight == BOTH && reedRightTime[1] - reedRightTimePrev[1] > minumumReedTimeInterrupt) {
        WheelRight = FORWARD;
    }
    
    else if (WheelRight == REVERSE && EncoderRight == FIRST) {
        WheelRight = RELEASE_REVERSE;
    }
    
    //count
    else if(WheelRight == RELEASE_FORWARD && EncoderRight ==  NONE) {
        WheelRightRot ++;
        WheelRight = UNDETERMINED;
    }
    //if we get here reset
    else if (EncoderRight == NONE){
        WheelRight = UNDETERMINED;
    }
    reedRightTimePrev[1] = reedRightTime[1];
    
}

//Rotary Encoder Left wheel
void reedLeftIntFirst () {
    //timestamp
    reedLeftTime[0] = begin_of_loop;
    
    // read the reed switch
    reed_stat_left[0] = digitalRead(reedLeft[0]);
    reed_stat_left[1] = digitalRead(reedLeft[1]);
    
    //determine state of the encoder
    // 0 is when the reed switch is closed
    if (reed_stat_left[0] && reed_stat_left[1]) {
        EncoderLeft = NONE;
    }
    if (!reed_stat_left[0] && reed_stat_left[1]) {
        EncoderLeft = FIRST;
    }
    if (reed_stat_left[0] && !reed_stat_left[1]) {
        EncoderLeft = LAST;
    }
    if (!reed_stat_left[0] && !reed_stat_left[1]) {
        EncoderLeft = BOTH;
    }
    
    //This one on first
    if (WheelLeft == UNDETERMINED && EncoderLeft == FIRST) {
        WheelLeft = TRIGGER_FORWARD;
    }
    
    // Time differential is also included to minimze debounce triggers with the mechanical reed switch
    else if(WheelLeft == TRIGGER_REVERSE && EncoderLeft == BOTH && reedLeftTime[0] - reedLeftTimePrev[0] > minumumReedTimeInterrupt) {
        WheelLeft = REVERSE;
    }
    
    else if (WheelLeft == FORWARD && EncoderLeft == LAST) {
        WheelLeft = RELEASE_FORWARD;
    }
    
    //count
    else if(WheelLeft == RELEASE_REVERSE && EncoderLeft ==  NONE) {
        WheelLeftRot --;
        WheelLeft = UNDETERMINED;
    }
    // if we get here for some reason reset
    else if (EncoderRight == NONE) {
        WheelLeft = UNDETERMINED;
    }
    
    reedLeftTimePrev[0] = reedLeftTime[0];
}
void reedLeftIntSecond () {
    //timestamp
    reedLeftTime[1] = begin_of_loop;
    
    // read the reed switch
    reed_stat_left[0] = digitalRead(reedLeft[0]);
    reed_stat_left[1] = digitalRead(reedLeft[1]);
    
    //determine state of the encoder
    // 0 is when the reed switch is closed
    if (reed_stat_left[0] && reed_stat_left[1]) {
        EncoderLeft = NONE;
    }
    if (!reed_stat_left[0] && reed_stat_left[1]) {
        EncoderLeft = FIRST;
    }
    if (reed_stat_left[0] && !reed_stat_left[1]) {
        EncoderLeft = LAST;
    }
    if (!reed_stat_left[0] && !reed_stat_left[1]) {
        EncoderLeft = BOTH;
    }
    
    //This one on first
    if (WheelLeft == UNDETERMINED && EncoderLeft == LAST) {
        WheelLeft = TRIGGER_REVERSE;
    }
    
    
    // Time differential is also included to minimze debounce triggers with the mechanical reed switch
    else if(WheelLeft == TRIGGER_FORWARD && EncoderLeft == BOTH && reedLeftTime[1] - reedLeftTimePrev[1] > minumumReedTimeInterrupt) {
        WheelLeft = FORWARD;
    }
    
    else if (WheelLeft == REVERSE && EncoderLeft == FIRST) {
        WheelLeft = RELEASE_REVERSE;
    }
    
    //count
    else if(WheelLeft == RELEASE_FORWARD && EncoderLeft ==  NONE) {
        WheelLeftRot ++;
        WheelLeft = UNDETERMINED;
    }
    // if we get here for some reason reset
    else if (EncoderRight == NONE){
        WheelLeft = UNDETERMINED;
    }
    
    
    reedLeftTimePrev[1] = reedLeftTime[1];
    
}

//WDT early interrupt warning
void WDT_Handler() {
    // we reach here, clear the WDT
    // one cycle will occur until the sleep function is called again
    wdtClear();
    wdt_int_occured = true;
    
    WDT->INTFLAG.bit.EW = 1;    //clear the interrupt flag
}

void TC4_Handler() {
    timerInterrupt = true;
    TC4->COUNT8.INTENCLR.bit.OVF = 1;   //clear the flag
}

//-------------------- Sleep Function ---------------------------------//
void sleepNow() {
    
    sleeping = true;
    //set wake up interrupt on motion detection
    IMU1.setMotionIntProcess();
    IMU2.setMotionIntProcess();
    
    //turn gyros off to save power
    IMU1.gyrosOff();
    IMU2.gyrosOff();
    
#ifdef ECHO
    Serial.println("going to sleep");
#endif
    
    NVMCTRL->CTRLB.bit.SLEEPPRM = 3; // disable power reduction mode during sleep
    
    //---------------- External interrupts clock ------------//
    //set up clock for external interrupt module
    GCLK->CLKCTRL.bit.GEN = 0x3;    //general clock 3
    GCLK->CLKCTRL.bit.ID = GCM_EIC;    //EIC clock
    GCLK->CLKCTRL.bit.CLKEN = 1;    // enable the clock
    while (GCLK->STATUS.bit.SYNCBUSY);
    
    //32kHz reference oscillator
    GCLK->GENCTRL.bit.ID = 0x03;    //general clock 3
    GCLK->GENCTRL.bit.SRC = 0x03;      //OSCULP32K
    GCLK->GENCTRL.bit.GENEN = 1;    //enable generator
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
    
    //PM->APBAMASK.reg = 0x00000050;   //keep external interrupt controller clock on and WDT
    EIC->WAKEUP.bit.WAKEUPEN9 = 1;  //enable interrupt wake up on pin 3
    Serial.println("did we make it?");
    //disable brown out detector
    //SYSCTRL->BOD33.bit.ACTION = 0;
    //SYSCTRL->VREG.bit.RUNSTDBY = 1;
    
    //sleep modes
    //PM->SLEEP.bit.IDLE = 0x02;  //CPU, AHB, and APB clocks are stopped
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  //put into deep sleep mode
    
    //go to sleep
    __DSB();
    __WFI();
    
    //we are here after wake up and respective interrupts
    IMU1.gyrosOn();
    IMU2.gyrosOn();
    
}

void midCycleSleep(uint8_t counterPreload) {
    
    //IMU1.gyrosOff();
    //IMU2.gyrosOff();
    
#ifdef ECHO
    Serial.println("going to mid cycle sleep");
#endif
    
    Serial.println("did we make it?");
    //TC4->COUNT8.CTRLA.bit.ENABLE = 0;
    //TC4->COUNT8.COUNT.reg = counterPreload; //load with calculated value
    //TC4->COUNT8.CTRLA.bit.ENABLE = 1;
    
    //sleep modes
    //PM->SLEEP.bit.IDLE = 0x02;  //CPU, AHB, and APB clocks are stopped
    //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  //put into deep sleep mode
    Serial.println("going to mid cycle sleep");

    Serial.println("we woke up!");
    //we are here after wake up and respective interrupts
   // IMU1.gyrosOn();
    //IMU2.gyrosOn();
    
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update with euler output

void MadgwickAHRSupdateIMU(Vector3<float> gyro, Vector3<float> acc, float q[4], Vector3<float> &ypr) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    //Vector3<float> ypr;
    
    // Rate of change of quaternion from gyro[1]roscope
    qDot1 = 0.5f * (-q[1] * gyro[0] - q[2] * gyro[1] - q[3] * gyro[2]);
    qDot2 = 0.5f * (q[0] * gyro[0] + q[2] * gyro[2] - q[3] * gyro[1]);
    qDot3 = 0.5f * (q[0] * gyro[1] - q[1] * gyro[2] + q[3] * gyro[0]);
    qDot4 = 0.5f * (q[0] * gyro[2] + q[1] * gyro[1] - q[2] * gyro[0]);
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((acc[0] == 0.0f) && (acc[1] == 0.0f) && (acc[2] == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        acc[0] *= recipNorm;
        acc[1] *= recipNorm;
        acc[2] *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q[0];
        _2q1 = 2.0f * q[1];
        _2q2 = 2.0f * q[2];
        _2q3 = 2.0f * q[3];
        _4q0 = 4.0f * q[0];
        _4q1 = 4.0f * q[1];
        _4q2 = 4.0f * q[2];
        _8q1 = 8.0f * q[1];
        _8q2 = 8.0f * q[2];
        q0q0 = q[0] * q[0];
        q1q1 = q[1] * q[1];
        q2q2 = q[2] * q[2];
        q3q3 = q[3] * q[3];
        
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * acc[0] + _4q0 * q1q1 - _2q1 * acc[1];
        s1 = _4q1 * q3q3 - _2q3 * acc[0] + 4.0f * q0q0 * q[1] - _2q0 * acc[1] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * acc[2];
        s2 = 4.0f * q0q0 * q[2] + _2q0 * acc[0] + _4q2 * q3q3 - _2q3 * acc[1] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * acc[2];
        s3 = 4.0f * q1q1 * q[3] - _2q1 * acc[0] + 4.0f * q2q2 * q[3] - _2q2 * acc[1];
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q[0] += qDot1 * (1.0f / gyroSampleFreq);
    q[1] += qDot2 * (1.0f / gyroSampleFreq);
    q[2] += qDot3 * (1.0f / gyroSampleFreq);
    q[3] += qDot4 * (1.0f / gyroSampleFreq);
    
    // Normalise quaternion
    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
    
    //calculate euler
    
    ypr[0] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    ypr[1] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    ypr[2] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    
    ypr = ypr * 180.0f / PI;
    
    //return ypr;
    
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root


float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


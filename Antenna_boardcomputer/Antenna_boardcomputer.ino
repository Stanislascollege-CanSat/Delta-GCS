/// Importing libraries


// AIM ANT
#include <Adafruit_GPS.h>
#include "A4988.h"
#include "wiring_private.h"

// COMM
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

// AIM ANT
#define HORI_MOTOR_STEPS 200
#define HORI_DIR 10
#define HORI_STEP 11
#define HORI_RPM 1
#define HORI_END_STOP 16

#define VERT_MOTOR_STEPS 200
#define VERT_DIR 6
#define VERT_STEP 12
#define VERT_RPM 1
#define VERT_END_STOP 15


#define SerialAD_RX 0
#define SerialAD_TX 1

A4988 hori_motor(HORI_MOTOR_STEPS, HORI_DIR, HORI_STEP);
A4988 vert_motor(VERT_MOTOR_STEPS, VERT_DIR, VERT_STEP);


Uart Serial2(&sercom2, 3, 4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM2_Handler(){
  Serial2.IrqHandler();
}


// COMM
// RADIO CHANNELS
const unsigned short int RH_CHANNEL_GS_ALPHA = 1;   //
const unsigned short int RH_CHANNEL_GS_DELTA = 2;   //
const unsigned short int RH_CHANNEL_MU = 3;         // Available radio-network-channels
const unsigned short int RH_CHANNEL_BETA = 4;       //
const unsigned short int RH_CHANNEL_RHO = 5;        //

const unsigned short int RH_CHANNEL_LOCAL = RH_CHANNEL_GS_DELTA; // Set local channel, used by the programme

// PIN DEFINITIONS
const unsigned short int PIN_RH_RST = 17;    //
const unsigned short int PIN_RH_CS = 18;     // Setting: RHDriver pins
const unsigned short int PIN_RH_INT = 9;    //

const float RHDriverFreq = 868.0;   // RHDriver Frequency

// RADIO DECLARATION
RH_RF95 RHDriver(PIN_RH_CS, PIN_RH_INT);
RHReliableDatagram RHNetwork(RHDriver, RH_CHANNEL_LOCAL);

bool reading;
String readCommand;

int prevRSSI;
int currRSSI;



// setting up some variables

// AIM ANT
// Constants
const float distancePerDegree = 111317.0996; //meters//
const float vert_lengthPerMotorRotation = 0.8; //cm//
const float hori_anglePerMotorRotation = 60; //degrees//

// Variables
float dLatitude;
float dLongitude;
float horizontalDistance;

float antLatitude;
float antLongitude;

float canLatitude;
float canLongitude;
float canAltitude;

// calculatory variables

  // ==> vertical aim
        float vert_aimAngle;
        float vert_angleAlpha;
        float vert_angleBeta;
  const float vert_rodLength = 21.21; //cm//
        float vert_screwLength; //cm//
        float vert_currentScrewLength; //cm//
        bool vert_motorResetting;


  // ==> horizontal aim
        float hori_aimAngle;
        float hori_northAngle;
        float hori_currentAngle;
        float hori_antennaAimAngle;
        bool hori_motorResetting;

// Setting up some classes

//Adafruit_GPS Sensor_GPS(&Serial2);


void HARDWARE_INTERRUPT_HORI_END_PIN(){
  hori_motorResetting = false;
  hori_motor.rotate(0);
  hori_currentAngle = 60;
  for(int i = 0; i < 5; ++i){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  Serial.println("Interrupt");
}

void HARDWARE_INTERRUPT_VERT_END_PIN(){
  vert_motorResetting = false;
  vert_motor.rotate(0);
  vert_currentScrewLength = 0;
  for(int i = 0; i < 5; ++i){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}



//
//  SETUP FUNCTION
//


void setup(){


  // COMM
  // --------------- Set RF reset HIGH -------------------- //
  pinMode(PIN_RH_RST, OUTPUT);
  digitalWrite(PIN_RH_RST, HIGH);

  // --------------- Starting serial @ 115200 -------------------- //
  Serial.begin(115200);
  while(!Serial);
  Serial.print("{SGS:2;F:LOG,[GCS] Delta started on @RHchannel " + String(RH_CHANNEL_LOCAL)+";}");

  // --------------- Force RFM95W reset -------------------- //
  digitalWrite(PIN_RH_RST, LOW);
  delay(10);
  digitalWrite(PIN_RH_RST, HIGH);
  delay(10);

  // --------------- Initializing RH_Datagram -------------------- //
  if(!RHNetwork.init()){
    Serial.print("{SGR:0;}");
    while(1);
  }
  Serial.print("{SGR:2;}");

  // --------------- Setting RH_Driver frequency -------------------- //

  if(!RHDriver.setFrequency(RHDriverFreq)){
    //Serial.println("ERR: 12 -> RHDriver setFrequency failed. Check the connection with the radio chip.");
    Serial.print("{SGF:0;}");
    while(1);
  }
  Serial.print("{SGF:2;F:LOG,[GCS] Frequency set to: "); Serial.print(RHDriverFreq); Serial.print("MHz;}");

  // --------------- Setting RH_Driver TxPower to 23 (maximum) -------------------- //

  RHDriver.setTxPower(23, false);
  Serial.print("{SGP:2;}");

  // --------------- Setting #retries for RH_Datagram -------------------- //

  RHNetwork.setRetries(2);

  // --------------- Setting duration timeout for RH_Datagram -------------------- //

  RHNetwork.setTimeout(0);


  reading = false;
  readCommand = "";
  prevRSSI = 0;




  // AIM ANT


  

//  Serial2.begin(115200);
//  pinPeripheral(3, PIO_SERCOM_ALT);
//  pinPeripheral(4, PIO_SERCOM_ALT);

  //while(!Serial2){}



  // Setting up the stepper motors
  hori_motor.begin(HORI_RPM, 1);
  vert_motor.begin(VERT_RPM, 1);


  // HARDWARE INTERRUPTS
  pinMode(HORI_END_STOP, INPUT_PULLDOWN);
  pinMode(VERT_END_STOP, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(HORI_END_STOP), HARDWARE_INTERRUPT_HORI_END_PIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(VERT_END_STOP), HARDWARE_INTERRUPT_VERT_END_PIN, HIGH);

  
  
  // Adafruit_GPS
//  Sensor_GPS.begin(9600);
//  Sensor_GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  Sensor_GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
//  Sensor_GPS.sendCommand(PGCMD_ANTENNA);

  // First value for variables
  antLatitude = 0;
  antLongitude = 0;
  

  canLatitude = 0;
  canLongitude = 0;
  canAltitude = 0;


  hori_motorResetting = true;
  vert_motorResetting = true;

  while(hori_motorResetting || vert_motorResetting){
    if(hori_motorResetting){
      hori_motor.rotate(1);
    }
    if(vert_motorResetting){
      vert_motor.rotate(-1);
    }
  }

  vert_currentScrewLength = 0;

  hori_motor.rotate(-360);
  hori_currentAngle = 0;
  hori_northAngle = 0;




  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i < 5; ++i){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  // 
}




float sqr(float a){
  return a*a;
}




//
// LOOP FUNCTION
//

void loop(){


  //
  // STANDARD DELTA SCRIPT
  //


  // RECEIVE COMMAND FROM COMPUTER
  if(Serial.available()){
    String reader = Serial.readString();
    Serial.println(reader);
    for(int i = 0; i < reader.length(); ++i){
//      switch(reader.charAt(i)){
//        case '[':
//          readCommand = "";
//          reading = true;
//          break;
//        case ']':
//          reading = false;
//          commandList.push_back(readCommand);
//          break;
//        default:
//          if(reading){
//            readCommand += reader.charAt(i);
//          }
//          break;
//      }
      if(reader.charAt(i) == '['){
        readCommand = "";
        reading = true;
      }else if(reader.charAt(i) == ']'){
        reading = false;
        String s = readCommand;
        Serial.print("{F:LOG,Received '" + s + "';}");
        if (s.equals("testcom")){
          //Serial.println("testcom");
          s = "[testcom]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_BETA);
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_RHO);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        } else if (s.equals("SAD")) {
          //Serial.print("{F:LOG,"+ s + ";}");
          s = "[SAD]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_BETA);
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_RHO);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        } else if (s.equals("DEP")) {
          s = "[DEP]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        } else if (s.equals("OPR")) {
          s = "[OPR]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        } else if (s.equals("CLR")) {
          s = "[CLR]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        } else if (s.equals("OPP")) {
          s = "[OPP]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        } else if (s.equals("CLP")) {
          s = "[CLP]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        } else if (s.equals("FLIGHT_MODE")){
          //Serial.print("{F:LOG,YES;}");
          s = "[FLIGHT_MODE]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_BETA);
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_RHO);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        }else if(s.equals("DSY")){
          s = "[DSY]";
          RHDriver.setModeTx();
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_BETA);
          RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_RHO);
          RHNetwork.waitPacketSent();
          RHDriver.setModeRx();
        }else if(s.length() > 7){
          if(s.substring(0, 7).equals("AIM_LAT")){
            //try{
              canLatitude = s.substring(7, s.length()-1).toFloat();
            //}catch(exception){}
          }else if(s.substring(0, 7).equals("AIM_LON")){
            //try{
              canLongitude = s.substring(7, s.length()-1).toFloat();
            //}catch(int i){}
          }else if(s.substring(0, 7).equals("AIM_ALT")){
            //try{
              canAltitude = s.substring(7, s.length()-1).toFloat();
            //}catch(int i){}
          }
        }
      }else{
        if(reading){
          readCommand += reader.charAt(i);
        }
      }
    }
    // GATHERED COMMANDS FROM COMPUTER (format: [command_name]) in std::vector<String> commandList.

    // CYCLING THROUGH commandList AND EXECUTING ALL COMMANDS
//    for(String s : commandList){
//      Serial.print("{F:LOG,Received '" + s + "';}");
//      if (s.equals("testcom")){
//        //Serial.println("testcom");
//        s = "[testcom]";
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_BETA);
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_RHO);
//        RHNetwork.waitPacketSent();
//      } else if (s.equals("SAD")) {
//        //Serial.print("{F:LOG,"+ s + ";}");
//        s = "[SAD]";
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_BETA);
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_RHO);
//        RHNetwork.waitPacketSent();
//      } else if (s.equals("DEP")) {
//        s = "[DEP]";
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
//        RHNetwork.waitPacketSent();
//      } else if (s.equals("OPR")) {
//        s = "[OPR]";
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
//        RHNetwork.waitPacketSent();
//      } else if (s.equals("CLR")) {
//        s = "[CLR]";
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
//        RHNetwork.waitPacketSent();
//      } else if (s.equals("OPP")) {
//        s = "[OPP]";
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
//        RHNetwork.waitPacketSent();
//      } else if (s.equals("CLP")) {
//        s = "[CLP]";
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
//        RHNetwork.waitPacketSent();
//      } else if (s.equals("FLIGHT_MODE")){
//        //Serial.print("{F:LOG,YES;}");
//        s = "[FLIGHT_MODE]";
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_MU);
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_BETA);
//        RHNetwork.sendtoWait((uint8_t*)s.c_str(), s.length(), RH_CHANNEL_RHO);
//        RHNetwork.waitPacketSent();
//      }
//    }
  }//End: if(Serial.available()){

  RHDriver.setModeRx();

  uint8_t BUF[RH_RF95_MAX_MESSAGE_LEN] = "";
  uint8_t LEN = sizeof(BUF);
  uint8_t FROM_ADDRESS;
  uint8_t TO_ADDRESS;

  if(RHNetwork.recvfromAck(BUF, &LEN, &FROM_ADDRESS, &TO_ADDRESS)){
    Serial.print((char*) BUF);
  }

  currRSSI = int(RHDriver.lastRssi());
  if(!(prevRSSI == currRSSI)){
    Serial.print("{CAN:" + String(RH_CHANNEL_LOCAL) + ";RS:" + String(currRSSI) + ";}");
  }
  prevRSSI = currRSSI;
  








  //
  // AIMING ANTENNA SCRIPT
  //
  

//  antLatitude = (Sensor_GPS.fix ? Sensor_GPS.latitudeDegrees : 0);
//  antLongitude = (Sensor_GPS.fix ? Sensor_GPS.longitudeDegrees : 0);



  // Use the calculated heading-angle to turn the motor
  dLatitude = canLatitude - antLatitude;
  dLongitude = canLongitude - antLongitude;
  hori_aimAngle = atan( (dLatitude * distancePerDegree) / (dLongitude * distancePerDegree) );
  hori_antennaAimAngle = hori_aimAngle - hori_northAngle;
  if(hori_currentAngle >= -60 && hori_currentAngle <= 60){
    hori_motor.rotate(hori_antennaAimAngle - hori_currentAngle);
    hori_currentAngle = hori_antennaAimAngle;
  }



  // Use the calculated altitude-angle to calculate the length of the screw-thread
  horizontalDistance = sqrt( sqr(dLatitude * distancePerDegree) + sqr(dLongitude * distancePerDegree) );
  vert_aimAngle = atan(canAltitude/horizontalDistance);
  vert_angleAlpha = 90 - vert_aimAngle;
  vert_angleBeta = 90 - vert_angleAlpha/2;
  vert_screwLength = sqrt( sqr(vert_rodLength - (vert_rodLength*cos(vert_angleAlpha))) + sqr(vert_rodLength*sin(vert_angleAlpha)) );
  
  

  // Use the calculated length of the screw-thread to turn the motor
  if(vert_screwLength < 29 && vert_screwLength > 0){
    vert_motor.rotate((vert_screwLength - vert_currentScrewLength)/vert_lengthPerMotorRotation);
    vert_currentScrewLength = vert_screwLength;
  }

  
}





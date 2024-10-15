/*
  * UDP Grade Control Module Code
  * For OpenGradeX   ONLY WORKS WITH OPENGRADEX NOT REGULAR OPENGRADE
  * 4 Feb 2022, Black Ace 
  * Like all Arduino code - copied from somewhere else
  * So don't claim it as your own
  *
  * Huge Thanks to Brian Tischler For doing all the legwork to make projects like this possible  
  * Check out his Git-Hub https://github.com/farmerbriantee   
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <MPU6050_light.h>

///
/// BUILD VERSION
///
const char *version = "1.3.1.0";

// Function STUBS for Platform IO

// GRADECONTROL
bool SetupGradeControlModule();
bool SetAutoState();

// UDP
bool SetupUdp();
bool SendUdpData(int _header);
bool RecvUdpData();

// WIFI
void ConnectToOGX();
void ReconnectToOGX();


/// UDP Variables
WiFiUDP UdpGradeControl;  // Creation of wifi UdpGradeControl instance
const char *ssid = {"OpenGradeX"};

char packetBuffer[255];
uint16_t openGradePort = 9999; //OpenGrade Server Port
uint16_t gradeControlPort = 7777; // GradeControl  Port
uint16_t antennaPort = 8888; // Antenna Port
uint16_t senderPort;
///Ip Addresses
IPAddress openGradeIP(192,168,1,156);   //OpenGradeX Server
IPAddress gradeControlIP(192,168,1,255);   // GradeControl Module IP
IPAddress antennaIP(192,168,1,155);   // Antenna Module IP
IPAddress gatewayIP(192,168,1,1);   // what we want the sp 32 IPAddress to be
IPAddress Subnet(255, 255, 255, 0);
IPAddress Dns(8,8,8,8);
IPAddress senderIP;

///////////////////////PINS///////////////////////
#define SERIAL_BAUD 115200
#define SCL_PIN 22      // I2C SCL PIN
#define SDA_PIN 21      // I2C SCL PIN
#define RXD2 16  // Diagnostic RX
#define TXD2 17 // Diagnostic TX
#define DEBUG Serial
#define RTK Serial1
#define BUILTIN_LED 2

//UDP HEADERS
#define DATA_HEADER 10001
#define GPS_HEADER 10003
#define IMU_HEADER 10004
#define RESET_HEADER 10100
#define SYSTEM_HEADER 10101
#define WIFI_HEADER 10102

///////////////////////led////////////////////////
#define PIN 13 // // WS2812 chip - Data In
#define Pixel_Number 11
#define Styrke 50           //max 255//
#define Bk (0, 0, 0)
#define Rd (Styrke, 0, 0)
#define Gn  (0, Styrke, 0)
#define Bl (0, 0, Styrke)
#define Yl (Styrke, Styrke, 0)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(Pixel_Number, PIN, NEO_GRB + NEO_KHZ800);
float Height;

////////////////////////mpu6050///////////////////
MPU6050 mpu(Wire);
unsigned long timer = 0;
int Angle;
float Adjust=100;
#define Antenna_Height 200          //antenna height (same as on tablet)//
float Actual_Height;

/////////////IMU///////////////
char *OG_data[255];
int16_t dataSize = sizeof(OG_data);

/////////////div//////////////////////
int b_autoState = 0, b_deltaDir = 0, b_cutDelta = 0;
///////////// Com Bytes///////////////
byte b_Ki, b_Kp, b_Kd;




/////////////// CNH Valve /////////////////////////

bool isAutoActive = false;
bool isCutting = false;

//loop time variables in milliseconds
const u16_t LOOP_TIME = 50; //20hz  
const u16_t LOOP_TIME2 = 30000; //.033HZ    
const u16_t LOOP_TIME3 = 500; //2HZ  

unsigned long lastTime = LOOP_TIME;
unsigned long lastTime2 = LOOP_TIME2;
unsigned long lastTime3 = LOOP_TIME3;  
unsigned long currentTime = 0; 

//Communication with OpenGradeX
bool isDataFound = false, isSettingFound = false, isOGXConnected = false;
int header = 0, tempHeader = 0;
unsigned long watchdogTimer = 0;   //make sure we are talking to OGX
const int OGXTimeout = 15;      


///////////////////////Initalize Objects///////////////////////
// I2C
TwoWire esp = TwoWire(5); 

void setup()
{ 
  ConnectToOGX();  
  SetupGradeControlModule();  
  strip.begin(); 
  Wire.begin();
 byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}
  

void loop(){  //Loop triggers every 50 msec (20hz) and sends back offsets Pid ect

  currentTime = millis();  

  RecvUdpData();  // Read Udp Data if Available  
  //(WiFi.status() == WL_CONNECTED) ? digitalWrite(BUILTIN_LED, HIGH) : digitalWrite(BUILTIN_LED, LOW);
  
  if (currentTime - lastTime >= LOOP_TIME) // 10 HZ
  {  
    watchdogTimer++;
    lastTime = currentTime;

    if (watchdogTimer > OGXTimeout){
      isOGXConnected = false;
      digitalWrite(BUILTIN_LED, LOW);
    }    
    else{
      isOGXConnected = true;
      digitalWrite(BUILTIN_LED, HIGH); // make sure connected to OGX   Time
    }
   
    
    
    (watchdogTimer > OGXTimeout*5000)? watchdogTimer = 50 : watchdogTimer; // Prevent overflow


    SendUdpData(DATA_HEADER);  // Send Data To OpenGradeX    
  }
  
  if (currentTime - lastTime2 >= LOOP_TIME2){ // .33 HZ
    lastTime2 = currentTime;
    ReconnectToOGX();
    SendUdpData(SYSTEM_HEADER);  // Send System info to OenGradeX
  }
  
  if (currentTime - lastTime3 >= LOOP_TIME3){ // 2 HZ
    lastTime3 = currentTime;
  }
  if (b_deltaDir == 0){Height = -(int)b_cutDelta;}  
  else {Height = (int)b_cutDelta;}
    
  mpu.update();

  if (mpu.getAngleX() < 0 ) {Angle = mpu.getAngleX() * -1 ;}
  else {Angle = mpu.getAngleX();}

  if(Angle >= 0 && Angle < 5){Adjust = 100;}
  else if(Angle >= 5 && Angle < 10){Adjust = 99;}
  else if(Angle >= 10 && Angle < 13){Adjust = 98;}
  else if(Angle >= 13 && Angle < 15){Adjust = 97;}
  else if(Angle >= 15 && Angle < 17){Adjust = 96;}
  else if(Angle >= 17 && Angle < 19){Adjust = 95;}
  else if(Angle >= 19 && Angle < 21){Adjust = 94;}
  else if(Angle >= 21 && Angle < 22){Adjust = 93;}
  else if(Angle >= 22 && Angle < 24){Adjust = 92;}
  else if(Angle >= 24 && Angle < 25){Adjust = 91;}
  else if(Angle >= 25 && Angle < 27){Adjust = 90;}
  else if(Angle >= 27 && Angle < 28){Adjust = 89;}
  else if(Angle >= 28 && Angle < 29){Adjust = 88;}
  else if(Angle >= 29 && Angle < 30){Adjust = 87;}
  else if(Angle >= 30 && Angle < 31){Adjust = 86;}
  else if(Angle >= 31 && Angle < 32){Adjust = 85;}
  else if(Angle >= 32 && Angle < 33){Adjust = 84;}
  else if(Angle >= 33 && Angle < 34){Adjust = 83;}
  else if(Angle >= 34 && Angle < 35){Adjust = 82;}
  else if(Angle >= 35 && Angle < 36){Adjust = 81;}
  else if(Angle >= 36 && Angle < 37){Adjust = 80;}
  else if(Angle >= 37 && Angle < 38){Adjust = 79;}
  else if(Angle >= 38 ){Adjust = 78;}
 
 if (Angle < 3) 
 {strip.setPixelColor(4, strip.Color Gn);}
 else if (Angle >= 3 && Angle < 35) 
 {strip.setPixelColor(4, strip.Color Yl);}
 else if (Angle >= 35) 
 {strip.setPixelColor(4, strip.Color Rd);}

  Actual_Height = Height - Antenna_Height + (Adjust * (Antenna_Height / 100));
  
  if(Actual_Height < -5){
 strip.setPixelColor(0, strip.Color Bl);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  
}
 else if(Actual_Height >= -5 && Actual_Height <= -4){
 strip.setPixelColor(0, strip.Color Bl);
 strip.setPixelColor(1, strip.Color Bl);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  
}
 else if(Actual_Height >= -4 && Actual_Height <= -3){
 strip.setPixelColor(0, strip.Color Bl);
 strip.setPixelColor(1, strip.Color Bl);
 strip.setPixelColor(2, strip.Color Bl);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  

} else if(Actual_Height >= -3 && Actual_Height <= -2){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bl);
 strip.setPixelColor(2, strip.Color Bl);
 strip.setPixelColor(3, strip.Color Gn);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  

} else if(Actual_Height >= -2 && Actual_Height <= -1){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bl);
 strip.setPixelColor(3, strip.Color Gn);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  

} else if(Actual_Height >= -1 && Actual_Height <= 0){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Gn);
 strip.setPixelColor(5, strip.Color Gn);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  

} else if(Actual_Height >= 0 && Actual_Height <= 1){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Gn);
 strip.setPixelColor(6, strip.Color Gn);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  
}
 else if(Actual_Height >= 1 && Actual_Height <= 2){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Gn);
 strip.setPixelColor(6, strip.Color Rd);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  
}
 else if(Actual_Height >= 2 && Actual_Height <= 3){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Gn);
 strip.setPixelColor(6, strip.Color Rd);
 strip.setPixelColor(7, strip.Color Rd);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  
}
 else if(Actual_Height >= 3 && Actual_Height <= 4){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Rd);
 strip.setPixelColor(7, strip.Color Rd);
 strip.setPixelColor(8, strip.Color Rd);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  
}
 else if(Actual_Height >= 4 && Actual_Height <= 5){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Rd);
 strip.setPixelColor(8, strip.Color Rd);
 strip.setPixelColor(9, strip.Color Rd);
 strip.setPixelColor(10, strip.Color Bk);
 strip.show();  
}
 else if(Actual_Height >= 5 && Actual_Height <= 6){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Rd);
 strip.setPixelColor(9, strip.Color Rd);
 strip.setPixelColor(10, strip.Color Rd);
 strip.show();  
} 
 else if(Actual_Height >= 6 && Actual_Height <= 7){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Rd);
 strip.setPixelColor(10, strip.Color Rd);
 strip.show();  
 }
 else if(Actual_Height >= 7){
 strip.setPixelColor(0, strip.Color Bk);
 strip.setPixelColor(1, strip.Color Bk);
 strip.setPixelColor(2, strip.Color Bk);
 strip.setPixelColor(3, strip.Color Bk);
 strip.setPixelColor(5, strip.Color Bk);
 strip.setPixelColor(6, strip.Color Bk);
 strip.setPixelColor(7, strip.Color Bk);
 strip.setPixelColor(8, strip.Color Bk);
 strip.setPixelColor(9, strip.Color Bk);
 strip.setPixelColor(10, strip.Color Rd);
 strip.show();  
}

  Serial.print("  Angle  "); Serial.print(Angle); 
  Serial.print("  Adjust  "); Serial.print(Adjust); 
  Serial.print("  Antenna_Height  "); Serial.print(Antenna_Height);
  Serial.print("  Height  "); Serial.print(Height);
  Serial.print("  Actual_Height  "); Serial.print(Actual_Height);
  Serial.print("  mpu.getAngleX()  "); Serial.println(mpu.getAngleX());
}

////////////////
//GRADECONTROL// 
////////////////

bool SetupGradeControlModule()
{

  pinMode(BUILTIN_LED, OUTPUT);  // Initialize the BUILTIN_LED pin as an output
  esp.begin(SDA_PIN , SCL_PIN);

  //set the baud rate
  DEBUG.begin(SERIAL_BAUD);    

  digitalWrite(2, HIGH); delay(500); digitalWrite(2, LOW); delay(500); digitalWrite(2, HIGH); delay(500);
  digitalWrite(2, LOW); delay(500); digitalWrite(2, HIGH); delay(500); digitalWrite(2, LOW); delay(500);

  

  SetupUdp(); 

   

  return true;

}


///////
//UDP// 
///////

bool SetupUdp(){ 
  
  UdpGradeControl.begin(gradeControlIP, gradeControlPort); //  this UDP address and port  
  return true;
}

bool SendUdpData(int _header)
{ 
  switch (_header){
    

    case GPS_HEADER:
        
        break;

    case IMU_HEADER:
        
        break;

    case RESET_HEADER:
        break;

    case SYSTEM_HEADER:
      UdpGradeControl.beginPacket(openGradeIP,openGradePort);   //Initiate transmission of data
      UdpGradeControl.print(_header);
      UdpGradeControl.print(",");
      UdpGradeControl.print(255);
      UdpGradeControl.print(",");
      UdpGradeControl.print(version);     
      UdpGradeControl.print("\r\n");   // End segment    
      UdpGradeControl.endPacket();  // Close communication

      DEBUG.printf("Version sent V%s", version);       
      DEBUG.println();
        break;


    default:
        break; 
  }

  return true;
}

bool RecvUdpData()
{ 
  
  char *strings[20];
  char *ptr = NULL;  

  //RECEPTION
  int packetSize = UdpGradeControl.parsePacket();   // Size of packet to receive
  
  if (packetSize) {       // If we received a package
    //reset watchdog
    watchdogTimer = 0;
    
    senderIP = UdpGradeControl.remoteIP();  //Sent from IP
    senderPort = UdpGradeControl.remotePort();  //Sent from IP
    //DEBUG.printf("Message Received from IP-> %s Port-> %u ", senderIP.toString(), senderPort);
    
    UdpGradeControl.read(packetBuffer, sizeof(packetBuffer));  

    byte index = 0;
    ptr = strtok(packetBuffer, ",");  // takes a list of delimiters    
    
    while(ptr != NULL)
    {
      strings[index] = ptr;      
      index++;
      ptr = strtok(NULL, ",");  // takes a list of delimiters
    }
    
    for(int n = 0; n < index; n++)
    { 
      OG_data[n] = strings[n];        
    }
    
    // convert to int to read couldnt read PTR fro some rsn   
    header = atoi(OG_data[0]);         
      
    switch (header)
    {
      case DATA_HEADER:
        DEBUG.println("DATA");
        b_deltaDir =  atoi(OG_data[1]);   // Cut Delta Dir
        b_autoState =  atoi(OG_data[2]);    // Cut Delta 
        b_cutDelta =  atoi(OG_data[3]);   // Auto State
        return true;
      break;




      case SYSTEM_HEADER:     
        
        if (atoi(OG_data[1]) != 0){
          SendUdpData(SYSTEM_HEADER);
        }
      break;

      case RESET_HEADER:
        ESP.restart(); 
        return true;
      break;

      default:
      break;
    }  

    return true;
  }
  return false;
}

////////
//WIFI//
////////
void ConnectToOGX() {
  WiFi.mode(WIFI_STA);
  WiFi.config(gradeControlIP , gatewayIP, Subnet);
  WiFi.begin(ssid);
  DEBUG.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG.print('.');
    delay(1000);
  }
  DEBUG.println(WiFi.localIP());
}

void ReconnectToOGX(){
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG.print(millis());
    DEBUG.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();   
  }
}
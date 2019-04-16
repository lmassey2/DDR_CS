#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

// variables for Wifi //
static char ssid[] = "";    // your network SSID (name)
static char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
static int keyIndex = 0;    // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
// variables for UDP
unsigned int localPort = 5000;      // local port to listen on
char packetBuffer[1023]; //buffer to hold incoming packet
WiFiUDP Udp;
// structure for speed and angle of robot
struct Velocity {
  int v;
  int theta;
};
// initializing structs
struct Velocity a;







void setup() {
  APMode();  // Settup uC as an access point for the pi
  Udp.begin(localPort);  // Open the ports needed for UDP messages
  // InitStructs();  Initialize data structures
  InitPins();  //Settup the pins needed for each component
  // SetIMU();  Establish connection with IMU
}

void loop() {
  // CheckIMU();  Check for data from IMU
  // a = CheckUDP();  Get data structure from UDP (v, theta)
  // SetDir(a.theta);  Set direction of robot, includes closed-loop system
  // SetSpeed(a.v);  Set speed of motors, includes closed-loop system
  // if (TRUE == pickedup){  If CheckIMU() found that the robot was
  //    Mr = 0;              picked up, stop the motors
  //    Ml = 0;
  // }
}


// make the microcontroller an access point
void APMode(){
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Access Point Mode");
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);
  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }
  // wait 10 seconds for connection:
  delay(10000);
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}


// initialize the pins for every component being used
void InitPins(){
  
}




////// OLD SKELETON //////
/*
void setup() {
  // APMode();  Settup uC as an access point for the pi
  // OpenPorts();  Open the ports needed for UDP messages
  // InitStructs();  Initialize data structures
  // SetPins();  Settup the pins needed for each component
  // SetIMU();  Establish connection with IMU
}

void loop() {
  // CheckIMU();  Check for data from IMU
  // a = CheckUDP();  Get data structure from UDP (v, theta)
  // SetDir(a.theta);  Set direction of robot, includes closed-loop system
  // SetSpeed(a.v);  Set speed of motors, includes closed-loop system
  // if (TRUE == pickedup){  If CheckIMU() found that the robot was
  //    Mr = 0;              picked up, stop the motors
  //    Ml = 0;
  // }
}
*/

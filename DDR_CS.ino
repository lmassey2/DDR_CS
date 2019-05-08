//////////////////////////////////////////////////////////////////////////////////////////////////
/*
//Copyright <2019> <LIAM R. MASSEY>//

Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions 
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of 
conditions and the following disclaimer in the documentation and/or other materials provided with 
the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to 
endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//////////////////////////////////////////////////////////////////////////////////////////////////


#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <math.h>
#include <time.h>
#include <Wire.h>
#include <LSM303.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

//pin definitions
#define MOTOR_R_1 5   // right motor's h-bridge PWM
#define MOTOR_R_2 6
#define MOTOR_L_1 13  // left motor
#define MOTOR_L_2 12
#define IRPIN_R 10
#define IRPIN_L 11
//constants for robot dimensions
#define BASE_L 95   //distance between wheels(mm)
#define WHEEL_R 15  //radius of wheels(mm)
#define TICKS 2     //number of rising edge ticks of encoder
#define GEAR_RATIO 75.81    //gear ratio of motor
#define DELTA_THETA_R ((2*PI/TICKS)/GEAR_RATIO)   //radians wheel travels per tick
#define PT (WHEEL_R*DELTA_THETA_R)    //distance wheel travels per tick
#define PHI atan2(PT, BASE_L)   //angle change of robot per tick (rads)
#define MAX_RPM 180           // maximum RPM of the motor
#define RPM_TO_PWM 1.417      // ratio of max RPM to max PWM
#define PERC_TO_RPM 1.8       // ration of max percentage to max RPM 
#define PERC_TO_PWM 2.55    // ration of max percentage to max PWM
#define TURN_SPEED 200      // speed (in terms of PWM) a wheel will turn to change theta
#define KP 0.5                 // proportional gain for the closed-loop system for speed of robot 
// variables for Wifi //
char ssid[] = "FeatherThyme";        // your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
// structure for speed and angle of robot
struct Velocity {
  int v;
  int theta;
};
// structure for what is sent back to the Pi
struct __attribute__((__packed__)) Backmsg {
  double odo[3];
  double imu[6];
  double heading;
};
// initializing structs
struct Velocity a;
struct Backmsg response;
// variables for UDP
unsigned int recvPort = 5000;      // local port to listen on
unsigned int sendPort = 4242;
char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP RecvUdp;
WiFiUDP SendUdp;
IPAddress sendIP(192,168,1,100);
// variables for IMU
#define MAG_RATIO 0.160  // magnetometer ratio in spec sheet
#define GRAV_ACC  9.81   // m/s^2 of gravity
LSM303 compass;
int pickedup = 0;
volatile float cur_imu_angle = 0;
float starting_theta;
float starting_rad;
// variables for odometry (matrix based)
const Matrix<4,4> id_matrix = {   //identity matrix
   1, 0, 0, 0,
   0, 1, 0, 0,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> right_angle_matrix = {    //angle matrix for right wheel
   cos(PHI), -sin(PHI), 0, 0,
   sin(PHI), cos(PHI), 0, BASE_L/2,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> right_delta_matrix = {    //delta matrix for right wheel
   1, 0, 0, 0,
   0, 1, 0, -BASE_L,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> right_full_matrix = right_angle_matrix * right_delta_matrix;    //combination of both right wheel matrices
const Matrix<4,4> left_angle_matrix = {   //angle matrix for left wheel
   cos(-PHI), -sin(-PHI), 0, 0,
   sin(-PHI), cos(-PHI), 0, -BASE_L/2,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> left_delta_matrix = {   //delta matrix for left wheel
   1, 0, 0, 0,
   0, 1, 0, BASE_L,
   0, 0, 1, 0,
   0, 0, 0, 1};
const Matrix<4,4> left_full_matrix = left_angle_matrix * left_delta_matrix;   //combination of both left wheel matrices
Matrix<4,4> TGR = id_matrix;    //matrix for robot's position and angle (global to robot)
volatile float matrix_delta_x = 0;   //delta x prime found using the matrix method (TGR[0][4])
volatile float matrix_delta_y = 0;   //delta y prime found using the matrix method (TGR[1][4])
volatile float cur_theta = 0;   //current z angle after using inverse trig of a value in the rotation matrix of TGR (degrees)
// PWM variables for setting the motors' speeds
volatile int Mr = 0;
volatile int Ml = 0;
volatile int new_theta = 1;
volatile int correct_theta = 0;
// variables for finding current speed of robot, found via IR interrupts
// do not need two sets of variables for each wheel since this speed
// is only used when angle is already correct and now moving forward
volatile float cur_speed = 0;  // current forward speed of robot (rad/ms)
volatile float tprev = 0;     // times used for knowing current speed
volatile float tcur = 0;
// timers for when to set odometry matrix angle equal to IMU angle, to reduce error
volatile float tick;
volatile float tock;


void setup() {
  APMode();  // Settup uC as an access point for the pi
  RecvUdp.begin(recvPort);  // Open the ports needed for UDP messages
  SendUdp.begin(sendPort);
  InitPins();  // Settup the pins needed for each component
  SetIMU();  // Establish connection with IMU
  InitStructs();  // Initialize data structures
  Serial.print("Setup Complete!\n");
  tick = millis();
}

void loop() {
  CheckAP();   // See if device connects
  CheckIMU();  // Check for data from IMU
  CheckUDP();  // Get data structure from UDP (v, theta)
  if (pickedup){  // If CheckIMU() found that the robot was
    Mr = 0;               // picked up, stop the motors
    Ml = 0;
  }
  else if (new_theta){
    SetDir();    // turn if needed
  }
  else if (correct_theta){
    SetSpeed();   // go forward once looking at correct angle
  }
  UpdateMotors();
  matrix_delta_x = TGR(0,3);
  matrix_delta_y = TGR(1,3);
  tock = millis();
  // every second make odometry angle equal IMU angle
  if (1000 < (tock-tick)){
    TGR(0,0) = cos(cur_imu_angle*PI/180);
    TGR(0,1) = -sin(cur_imu_angle*PI/180);
    TGR(1,0) = sin(cur_imu_angle*PI/180);
    TGR(1,1) = cos(cur_imu_angle*PI/180);
    tick = millis();
  }
  UpdateMsg();
}


// make the microcontroller an access point
void APMode(){
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
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


// initialize structures and variables
void InitStructs(){
  Serial.print("Initializing Structs...\n");
  a.v = 0;
  a.theta = 0;
  compass.read();
  starting_theta = compass.heading();
  Serial.print("Starting Angle = ");
  Serial.print(starting_theta);
  Serial.print("\n");
  cur_theta = starting_theta;
  cur_imu_angle = starting_theta;
  starting_rad = starting_theta*PI/180;
  TGR(0,0) = cos(starting_rad);
  TGR(0,1) = -sin(starting_rad);
  TGR(1,0) = sin(starting_rad);
  TGR(1,1) = cos(starting_rad);
}


// initialize the pins for every component being used
void InitPins(){
  Serial.print("Initializing Pins...\n");
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(IRPIN_R, INPUT);
  pinMode(IRPIN_L, INPUT);
  attachInterrupt(IRPIN_R, rightISR, FALLING);
  attachInterrupt(IRPIN_L, leftISR, FALLING);
  digitalWrite(IRPIN_R, LOW);
  digitalWrite(IRPIN_L, LOW);
}


// initialize what is needed for IMU
void SetIMU(){
  Serial.print("Initializing IMU...\n");
  Wire.begin();
  compass.init();
  compass.enableDefault();
  // values found via running the Calibration example in the library
  compass.m_min = (LSM303::vector<int16_t>){ -3223,  -6301,  -3167};
  compass.m_max = (LSM303::vector<int16_t>){ +3573,   +252,  +4339};
}


// connect device to AP mode
void CheckAP(){
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      byte remoteMac[6];

      // a device has connected to the AP
      Serial.print("Device connected to AP, MAC address: ");
      WiFi.APClientMacAddress(remoteMac);
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
}


// checking data from IMU to see if robot is picked up
void CheckIMU(){
  compass.read();
  // IMU specs specify to drop 4 last bits, leaving us with mg
  // divide by 1000 to turn it into g
  int mgs = compass.a.z >> 4;
  float realgs = (float)((float)mgs/1000);
  // check if acceleration goes past a threshold, signalling
  // it has been picked up
  if (0.7 > realgs){
    pickedup = 1;
    Serial.print("Picked up! Stopping!\n");
  }

  cur_imu_angle = compass.heading();
}


// getting data from UDP if there is any
void CheckUDP(){
  // if there's data available, read a packet
  int packetSize = RecvUdp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = RecvUdp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(RecvUdp.remotePort());

    // read the packet into packetBufffer
    int len = RecvUdp.read(packetBuffer, 255);
    sendIP = RecvUdp.remoteIP();
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    char curch = 'n';
    int i = 0;
    String inString = packetBuffer;
    String vString = "";
    String thetaString = "";
    int afound = 0;       // is a char in between the two values of the struct
    int bfound = 0;       // b, q, and r are ending chars for the messages   
    int qfound = 0;
    while (0==bfound){
      curch = inString[i];
      Serial.print(curch);
      Serial.print("\n");
      if ('a' == curch){
        Serial.print("Found a!\n");
        afound = 1;
      }
      else if ('b' == curch){
        Serial.print("Found b!\n");
        bfound = 1;
      }
      else if ('q' == curch){
        qfound = 1;
        bfound = 1;
        Serial.print("Found q!\n");
      }
      else if ('r' == curch){
        pickedup = 0;
        bfound = 1;
        Serial.print("Found r!\n");
      }
      else if (0 == afound){
        vString += curch;
      }
      else{
        thetaString += curch;
      }
      i++;
    }
    a.v = vString.toInt();
    int in_theta = thetaString.toInt();
    if (in_theta != a.theta){
      new_theta = 1;
    }
    else{
      new_theta = 0;
    }
    a.theta = in_theta;
    // construct and send a response string based on the c struct
    // turning the struct into a string is currently what makes the UDP
    // message send properly from the microcontroller.
    if (1==qfound){
      String outString = "\n\r"+String(response.odo[0])+" = x pos, "+String(response.odo[1])+" = y pos, "+String(response.odo[2])+" = z pos\n\r";
      outString = outString+String(response.imu[0])+" = ax, "+String(response.imu[1])+" = ay, "+String(response.imu[2])+" = az, ";
      outString = outString+String(response.imu[3])+" = mx, "+String(response.imu[4])+" = my, "+String(response.imu[5])+" = mz\n\r";
      outString = outString+String(response.heading)+" = heading\n\r";
      SendUdp.beginPacket(sendIP, sendPort);
      SendUdp.print(outString);
      SendUdp.endPacket();
    }
  }
}


// setting z angle of robot
// outputs 1 when looking in correct direction
void SetDir(){
  float cur_enc_theta = (acos(TGR(0,0))*(180/PI));
  // convert +/- 180 degrees --> 0 to 360 degrees
  if (0 > cur_enc_theta){
    cur_enc_theta = 360 + cur_enc_theta;
  }
  cur_theta = cur_imu_angle;
  Serial.print(cur_theta);
  Serial.print("\n");
  if(cur_theta < ((float)a.theta - 5)){  // +/-5 a small range of acceptable angles
    Mr = TURN_SPEED;                     // for the robot to be facing, this way it system
    Ml = 0;                              // isn't constantly currecting its z-angle and never
    tcur = 0;
    tprev = 0;
    correct_theta = 0;
  }
  else if(cur_theta > ((float)a.theta + 5)){
    Mr = 0;
    Ml = TURN_SPEED;
    tcur = 0;
    tprev = 0;
    correct_theta = 0;
  }
  else{
    new_theta = 0;
    correct_theta = 1;
    Mr = 0;
    Ml = 0;
    tprev = millis(); // now that we are facing the right dirrect, start
  }                   // timer for calculating speed
}


// setting the speed of the motors when moving forward
// only occurs once facing the correct direction
void SetSpeed(){
  // current speed is in rad/ms. /1000 to make it rad/s, /60 to rad/m, then ratio to be percentage
  float speed_perc = ((float)cur_speed/(float)(60*1000*(PERC_TO_RPM)));
  int speed_delta = (int)(a.v - speed_perc);
  // if the below controller does not work, dirrectly take the a.v
  //Mr = Mr + (speed_delta * KP * PERC_TO_PWM);  // remember, analogWrite input is 0-255
  //Ml = Ml + (speed_delta * KP * PERC_TO_PWM);

  Mr = a.v * PERC_TO_PWM;
  Ml = Mr;
  
  if (Mr > 255){
    Mr = 255;
  }
  if (Ml > 255){
    Ml = 255;
  }
  if (Mr < 0){
    Mr = 0;
  }
  if (Ml < 0){
    Ml = 0;
  }
}


// PWM to update the rotation of the motors
void UpdateMotors(){
  // right motor
  analogWrite(MOTOR_R_1, Mr);
  analogWrite(MOTOR_R_2, 0);
  // left motor
  analogWrite(MOTOR_L_1, Ml);
  analogWrite(MOTOR_L_2, 0);
}

// updates the UDP response of data to the Pi
void UpdateMsg(){
  response.odo[0] = (double)TGR(0,3);
  response.odo[1] = (double)TGR(1,3);
  response.odo[2] = (double)TGR(2,3);
  // IMU specs specify to drop 4 last bits, leaving us with mg
  // divide by 1000 to turn it into g
  int xmgs = compass.a.x >> 4;
  int ymgs = compass.a.y >> 4;
  int zmgs = compass.a.z >> 4;
  float xrealgs = (float)((float)xmgs/1000);
  float yrealgs = (float)((float)ymgs/1000);
  float zrealgs = (float)((float)zmgs/1000);
  // multiply by 9.81m/s^2 to get readable acceleration values
  response.imu[0] = (double)xrealgs * GRAV_ACC;
  response.imu[1] = (double)yrealgs * GRAV_ACC;
  response.imu[2] = (double)zrealgs * GRAV_ACC;
  // magnetometer readings come in as milligauss multiplied by a conversion factor
  // to go from gauss to degrees must use inverse tangent, so the conversion factor
  // is not needed as it would be cancelled out in the division within the trig
  response.imu[3] = atan2(compass.m.y, compass.m.z) * (180/PI);
  response.imu[4] = atan2(compass.m.x, compass.m.z) * (180/PI);
  response.imu[5] = atan2(compass.m.x, compass.m.y) * (180/PI);
  response.heading = (double)cur_imu_angle;
}


//interrupt routine for the right wheel
//updates current position and angle every right wheel tick based on predefined constants
void rightISR(){
  // odometry
  TGR = TGR * right_full_matrix;
  // get speed, only if already looking in correct direction
  if (correct_theta){
    tcur = millis();
    cur_speed = 2*PI/(TICKS*(tcur-tprev));
    tprev = tcur;
  }
}


//interrupt routine for the left wheel
//updates current position and angle every rleft wheel tick based on predefined constants
void leftISR(){
  // odometry
  TGR = TGR * left_full_matrix;
  // since speed is only being calculated when already looking at the correct angle,
  // don't need to find current speed in both wheels' interrupts
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

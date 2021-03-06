/* An edited version of the DDR_CS.ino.  This version is dedicated to
 *  the first test:  moving 50cm on the x-axis, making a righhand 90 degree
 *  turn, moving another 50cm forward, now on the global y-axis, finally printing
 *  out current odometry to the Raspberry Pi.
 *  NOTE: this does not use the IMU, as requested.
 *  NOTE: distances changed to 20cm each as there is currently not
 *        a flat area with 50x50cm of space
 */

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
#define MOTOR_L_1 5   // right motor's h-bridge PWM
#define MOTOR_L_2 6
#define MOTOR_R_1 12  // left motor
#define MOTOR_R_2 13
#define IRPIN_R 10
#define IRPIN_L 11
//constants for robot dimensions
#define BASE_L 95   //distance between wheels(mm)
#define WHEEL_R 15  //radius of wheels(mm)
#define TICKS 2     //number of rising edge ticks of encoder
#define GEAR_RATIO 75.81    //gear ratio of motor
#define PT ((2*PI*WHEEL_R)/(GEAR_RATIO*2))    //distance wheel travels per tick
#define PHI atan2(PT, BASE_L)   //angle change of robot per tick (rads)
#define MAX_RPM 180           // maximum RPM of the motor
#define RPM_TO_PWM 1.417      // ratio of max RPM to max PWM
#define PERC_TO_RPM 1.8       // ration of max percentage to max RPM 
#define PERC_TO_PWM 2.55    // ration of max percentage to max PWM
#define TURN_SPEED 255      // speed (in terms of PWM) a wheel will turn to change theta
#define KP 10                  // proportional gain for the closed-loop system for speed of robot 
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
// initializing structs
struct Velocity a;
// variables for UDP
unsigned int recvPort = 5000;      // local port to listen on
unsigned int sendPort = 5005;
char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP RecvUdp;
WiFiUDP SendUdp;
IPAddress sendIP;
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
volatile float cur_enc_theta = 0;    //current z angle via encoder odometery acos((TGR[1][4]))
// PWM variables for setting the motors' speeds
volatile int Mr = 0;
volatile int Ml = 0;
volatile int correct_theta = 0;
volatile int has_turned = 0;       // flips to 1 once the robot has fully stopped when facing correct direction.
// variables for finding current speed of robot, found via IR interrupts
// do not need two sets of variables for each wheel since this speed
// is only used when angle is already correct and now moving forward
volatile float cur_speed = 0;  // current forward speed of robot (rad/ms)
volatile float tprev = 0;     // times used for knowing current speed
volatile float tcur = 0;
// distance and angle the robot must move for the test, with state variables
#define GO_DIST 2000     // supposed to be 20cm(200mm), but the interrupts for the IR
#define FIRST_ANGLE 0    // sensors are occuring more than expected no matter what sensity
#define SECOND_ANGLE 270 // the are at.
#define GO_SPEED 100     // percentage of max speed of PWM that can be applied to the motors
#define STOP_SPEED 0
volatile int first_done = 0;
volatile int turn_done = 0;
volatile int second_done = 0;

Matrix<4,4> testmat = TGR * right_full_matrix;


void setup() {
  APMode();  // Settup uC as an access point for the pi
  RecvUdp.begin(recvPort);  // Open the ports needed for UDP messages
  SendUdp.begin(sendPort);
  InitPins();  // Settup the pins needed for each component
  InitStructs();  // Initialize data structures
  Serial.print("Setup Complete!\n");
  /*Serial.println(PT);
  Serial.println(PHI);
  Serial.println("right matrix = ");
  Serial.print(right_full_matrix(0,0));
  Serial.print("  ");
  Serial.print(right_full_matrix(0,1));
  Serial.print("  ");
  Serial.print(right_full_matrix(0,2));
  Serial.print("  ");
  Serial.print(right_full_matrix(0,3));
  Serial.print("\n");
  Serial.print(right_full_matrix(1,0));
  Serial.print("  ");
  Serial.print(right_full_matrix(1,1));
  Serial.print("  ");
  Serial.print(right_full_matrix(1,2));
  Serial.print("  ");
  Serial.print(right_full_matrix(1,3));
  Serial.print("\n");
  Serial.print(right_full_matrix(2,0));
  Serial.print("  ");
  Serial.print(right_full_matrix(2,1));
  Serial.print("  ");
  Serial.print(right_full_matrix(2,2));
  Serial.print("  ");
  Serial.print(right_full_matrix(2,3));
  Serial.print("\n");
  Serial.print(right_full_matrix(3,0));
  Serial.print("  ");
  Serial.print(right_full_matrix(3,1));
  Serial.print("  ");
  Serial.print(right_full_matrix(3,2));
  Serial.print("  ");
  Serial.print(right_full_matrix(3,3));
  Serial.print("\n");
  Serial.println("testmat = ");
  Serial.print(testmat(0,0));
  Serial.print("  ");
  Serial.print(testmat(0,1));
  Serial.print("  ");
  Serial.print(testmat(0,2));
  Serial.print("  ");
  Serial.print(testmat(0,3));
  Serial.print("\n");
  Serial.print(testmat(1,0));
  Serial.print("  ");
  Serial.print(testmat(1,1));
  Serial.print("  ");
  Serial.print(testmat(1,2));
  Serial.print("  ");
  Serial.print(testmat(1,3));
  Serial.print("\n");
  Serial.print(testmat(2,0));
  Serial.print("  ");
  Serial.print(testmat(2,1));
  Serial.print("  ");
  Serial.print(testmat(2,2));
  Serial.print("  ");
  Serial.print(testmat(2,3));
  Serial.print("\n");
  Serial.print(testmat(3,0));
  Serial.print("  ");
  Serial.print(testmat(3,1));
  Serial.print("  ");
  Serial.print(testmat(3,2));
  Serial.print("  ");
  Serial.print(testmat(3,3));
  Serial.print("\n");*/
}

void loop() {
  CheckAP();   // See if device connects
  CheckUDP();  // Get data structure from UDP (v, theta)
  correct_theta = SetDir();
  // if facing the right way, stop the motors and wait a couple seconds before moving forward or turning again
  if ((1==correct_theta) && (0==has_turned) && (0==second_done)){
    Mr = 0;
    Ml = 0;
    UpdateMotors();
    int turntick = millis();
    int turntock = turntick;
    while (2000 > (turntock - turntick)){
      turntock = millis();
    }
    has_turned = 1;
  }
  // facing the right was and has stopped for awhile, move forward now
  else if(1==has_turned){
    SetSpeed();
  }
  if (1==second_done){
    Mr = 0;
    Ml = 0;
  }
  UpdateMotors();
  matrix_delta_x = TGR(0,3);
  matrix_delta_y = TGR(1,3);
  if ((0==first_done) && (GO_DIST<matrix_delta_x)){
    first_done = 1;
  }
  else if((1==first_done) && ((GO_DIST<matrix_delta_y)||(-GO_DIST>matrix_delta_y))){
    second_done = 1;
  }
  Serial.print(matrix_delta_x);
  Serial.print("  ");
  Serial.print(matrix_delta_y);
  Serial.print("  ");
  Serial.print(cur_enc_theta);
  Serial.print("  ");
  Serial.print(first_done);
  Serial.print("  ");
  Serial.println(second_done);
}


// make the microcontroller an access point
void APMode(){
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/
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


// getting data from UDP if there is any
/* without having to change the entire way the microcontroller gets messages from the Pi,
 *  communication remains the same, but the desired speed and angle of the robot are overriden based on
 *  where it currently is in the odometery test.
 */
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
    int afound = 0;
    int bfound = 0;
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
      else if (0 == afound){
        vString += curch;
      }
      else{
        thetaString += curch;
      }
      i++;
    }
    if (1==qfound){
      String xString = String(matrix_delta_x);
      String yString = String(matrix_delta_y);
      String zString = String(cur_enc_theta);
      String outString = "x: "+xString+"  y: "+yString+"  phi: "+zString+"\n";
      SendUdp.beginPacket(sendIP, sendPort);
      SendUdp.print(outString);
      SendUdp.endPacket();
    }
  }
  // first 50cm, move forward, do not turn
  if (0==first_done){
    a.v = GO_SPEED;
    a.theta = FIRST_ANGLE;
  }
  // second 50cm, turn to -90 degrees then move 50cm
  else if ((0==second_done) && (1==first_done)){
    a.v = GO_SPEED;
    a.theta = SECOND_ANGLE;
  }
  // final destination reached, stop moving
  else{
    a.v = STOP_SPEED;
  }
}


// setting z angle of robot
// outputs 1 when looking in correct direction
int SetDir(){
  cur_enc_theta = (acos(TGR(0,0))*(180/PI));  // show current angle from encoders in degrees
  // convert +/- 180 degrees --> 0 to 360 degrees
  if(0 > cur_enc_theta){
    cur_enc_theta = 360 + cur_enc_theta;
  }
  if(cur_enc_theta < ((float)a.theta - 2)){  // +/-2 a small range of acceptable angles
    Mr = TURN_SPEED;                     // for the robot to be facing, this way it system
    Ml = 0;                              // isn't constantly currecting its z-angle and never
        tcur = 0;
    tprev = 0;
    return 0;                            // not allowing the robot to move forward.
  }
  if(cur_enc_theta > ((float)a.theta + 2)){
    Mr = 0;
    Ml = TURN_SPEED;
    tcur = 0;
    tprev = 0;
    return 0;                           // not allowing the robot to move forward.
  }
  tprev = millis(); // now that we are facing the right dirrect, start moving forward
  return 1;         // timer for calculating speed
}


// setting the speed of the motors when moving forward
// only occurs once facing the correct direction
void SetSpeed(){
  // current speed is in rad/ms. /1000 to make it rad/s, /60 to rad/m, then ratio to be percentage
  float speed_perc = (float)cur_speed/(60*1000*(PERC_TO_RPM));
  int speed_delta = (int)(a.v - speed_perc);
  Mr = Mr + (speed_delta * KP * PERC_TO_PWM);  // remember, analogWrite input is 0-255
  Ml = (Ml + (speed_delta * KP * PERC_TO_PWM))*0.25; // the right motor started running at reduced speeds
  if (Mr > 255){                                     // in an attempt to make the robot go straight
    Mr = 255;                                        // the speed of the left motor is reduced
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
  analogWrite(MOTOR_R_1, 0);
  analogWrite(MOTOR_R_2, Mr);
  // left motor
  analogWrite(MOTOR_L_1, Ml);
  analogWrite(MOTOR_L_2, 0);
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

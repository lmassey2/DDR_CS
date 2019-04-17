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
#define MOTOR_L_1 12  // left motor
#define MOTOR_L_2 13
#define IRPIN_R 10
#define IRPIN_L 11
//constants for robot dimensions
#define BASE_L 80   //distance between wheels(mm)
#define WHEEL_R 20  //radius of wheels(mm)
#define TICKS 2     //number of rising edge ticks of encoder
#define GEAR_RATIO 75.81    //gear ratio of motor
#define DELTA_THETA_R ((2*PI/TICKS)/GEAR_RATIO)   //radians wheel travels per tick
#define PT (WHEEL_R*DELTA_THETA_R)    //distance wheel travels per tick
#define PHI atan2(PT, BASE_L)   //angle change of robot per tick (radians)
#define MAX_RPM 180           // maximum RPM of the motor
#define RPM_TO_PWM 1.417      // ratio of max RPM to max PWM
#define PERC_TO_RPM 1.8       // ration of max percentage to max RPM 
#define PERC_TO_PWM 2.55    // ration of max percentage to max PWM
#define TURN_SPEED 127      // speed (in terms of PWM) a wheel will turn to change theta
#define KP 5                  // proportional gain for the closed-loop system for speed of robot 
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
// variables for IMU
LSM303 compass;
int pickedup = 0;
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
volatile double matrix_delta_x = 0;   //delta x prime found using the matrix method (TGR[0][4])
volatile double matrix_delta_y = 0;   //delta y prime found using the matrix method (TGR[1][4])
volatile double matrix_z_angle = 0;   //current z angle after using inverse trig of a value in the rotation matrix of TGR (radians)
// PWM variables for setting the motors' speeds
volatile int Mr = 0;
volatile int Ml = 0;
volatile int correct_theta = 0;
// variables for finding current speed of robot, found via IR interrupts
// do not need two sets of variables for each wheel since this speed
// is only used when angle is already correct and now moving forward
volatile unsigned long cur_speed = 0;  // current forward speed of robot (rad/ms)
volatile unsigned long tprev = 0;
volatile unsigned long tcur = 0;



void setup() {
  APMode();  // Settup uC as an access point for the pi
  Udp.begin(localPort);  // Open the ports needed for UDP messages
  InitStructs();  // Initialize data structures
  InitPins();  // Settup the pins needed for each component
  SetIMU();  // Establish connection with IMU
}

void loop() {
  CheckIMU();  // Check for data from IMU
  CheckUDP();  // Get data structure from UDP (v, theta)
  correct_theta = SetDir();
  if (correct_theta){
    SetSpeed();
  }
  if (pickedup){  // If CheckIMU() found that the robot was
    Mr = 0;               // picked up, stop the motors
    Ml = 0;
  }
  UpdateMotors();
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


// initialize structures
void InitStructs(){
  a.v = 0;
  a.theta = 0;
}


// initialize the pins for every component being used
void InitPins(){
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(IRPIN_R, INPUT);
  pinMode(IRPIN_L, INPUT);
  attachInterrupt(IRPIN_R, rightISR, RISING);
  attachInterrupt(IRPIN_L, leftISR, RISING);
}


// initialize what is needed for IMU
void SetIMU(){
  Wire.begin();
  compass.init();
  compass.enableDefault();
}



// checking data from IMU to see if robot is picked up
void CheckIMU(){
  compass.read();
  if (-8 < compass.a.z){
    pickedup = 1;
  }
}


// getting data from UDP if there is any
void CheckUDP(){
  int packetSize = Udp.parsePacket();
  if (packetSize){
    int len = Udp.read(packetBuffer, 1023);
    if (len > 0) packetBuffer[len] = 0;
    // a = (Velocity)packetBuffer;
  }
}


// setting z angle of robot
int SetDir(){
  int cur_theta = acos(TGR(0,0));
  if(cur_theta < a.theta - 1){  // +/-1 a small range of acceptable angles
    Mr = TURN_SPEED;            // for the robot to be facing, this way it system
    Ml = 0;                     // isn't constantly currecting its z-angle and never
    return 0;                   // allowing the robot to move forward.
  }
  if(cur_theta > a.theta + 1){
    Mr = 0;
    Ml = TURN_SPEED;
    return 0;
  }
  tprev = millis(); // now that we are facing the right dirrect, start
  return 1;         // timer for calculating speed
}


// setting the speed of the motors when moving forward
void SetSpeed(){
  // current speed is in rad/ms. /1000 to make it rad/s, /60 to rad/m, then ratio to be percentage
  int speed_perc = cur_speed/(60*1000*(PERC_TO_RPM));
  int speed_delta = a.v - speed_perc;
  Mr = Mr + (speed_delta * KP * PERC_TO_PWM);  // remember, analogWrite input is 0-255
  Ml = Ml + (speed_delta * KP * PERC_TO_PWM);
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
  analogWrite(MOTOR_R_1, Ml);
  analogWrite(MOTOR_R_2, 0);
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

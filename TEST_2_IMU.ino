/* An edited version of the DDR_CS.ino.  This version is dedicated to
 *  the second test:  robot rotates to what it thinks is North, then rotates
 *  to face South, then East, then West, and North again.  5 second
 *  interval between each turn.
 *  NOTE: this only uses the IMU, no odometery, as requested. No IR interrupts needed.
 *  NOTE: as this is an automated test, WiFi and UDP features are removed.
 */


#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <math.h>
#include <time.h>
#include <Wire.h>
#include <LSM303.h>
#include <SPI.h>

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
#define DELTA_THETA_R ((2*PI/TICKS)/GEAR_RATIO)   //radians wheel travels per tick
#define PT (WHEEL_R*DELTA_THETA_R)    //distance wheel travels per tick
#define PHI atan2(PT, BASE_L)   //angle change of robot per tick (rads)
#define MAX_RPM 180           // maximum RPM of the motor
#define RPM_TO_PWM 1.417      // ratio of max RPM to max PWM
#define PERC_TO_RPM 1.8       // ration of max percentage to max RPM 
#define PERC_TO_PWM 2.55    // ration of max percentage to max PWM
#define TURN_SPEED 255      // speed (in terms of PWM) a wheel will turn to change theta
#define KP 10                  // proportional gain for the closed-loop system for speed of robot 
// structure for speed and angle of robot
struct Velocity {
  int v;
  int theta;
};
// initializing structs
struct Velocity a;

// variables for IMU
#define MAG_RATIO 0.160  // magnetometer ratio in spec sheet
LSM303 compass;
volatile float imu_angle = 0;

// PWM variables for setting the motors' speeds
volatile int Mr = 0;
volatile int Ml = 0;
volatile int correct_theta = 0;

// stage variable to determine which direction to face
// 1 and 5 = North, 2 = South, 3 = East, 4 = West
volatile int facing = 1;
// no more turning after stage 5
volatile int all_done = 0;


void setup() {
  Serial.begin(9600);
  InitPins();  // Settup the pins needed for each component
  SetIMU();  // Establish connection with IMU
  InitStructs();  // Initialize data structures
  Serial.print("Setup Complete!\n");

}

void loop() {
  CheckIMU();  // Check for data from IMU
  
  a.v = 0;  // for this turning test we are never moving straight
  if ((1==facing) || (5==facing)){  // face north
    a.theta = 0;
  }
  else if (2==facing){         // face south
    a.theta = 180;
  }
  else if (3==facing){         // face east
    a.theta = 90;
  }
  else if (4==facing){         // face west
    a.theta = 270;
  }
  else{
    all_done = 1;
    Mr = 0;
    Ml = 0;
  }
  if(0==all_done){
    correct_theta = SetDir();
  }
  if (correct_theta){
    //SetSpeed();  never moving forward, so don't call this
    Mr = 0;
    Ml = 0;
    UpdateMotors();
    int tick = millis();
    int tock = tick;
    // facing correct angle, wait 5 seconds (5000 ms)
    while(5000 > (tock - tick)){
      tock = millis();
    }
    // after facing the correct direction for 5 seconds, go to
    // next state.
    facing++;
  }
  UpdateMotors();
}


// initialize structures and variables
void InitStructs(){
  Serial.print("Initializing Structs...\n");
  a.v = 0;
  a.theta = 0;
  compass.read();
  imu_angle = compass.heading();
  Serial.print("Starting Angle = ");
  Serial.print(imu_angle);
  Serial.print("\n");
}


// initialize the pins for every component being used
void InitPins(){
  Serial.print("Initializing Pins...\n");
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
}


// initialize what is needed for IMU
void SetIMU(){
  Serial.print("Initializing IMU...\n");
  Wire.begin();
  compass.init();
  compass.enableDefault();
  // values found via running the Calibration example in the library
  compass.m_min = (LSM303::vector<int16_t>){-2317, -5867, -3964};
  compass.m_max = (LSM303::vector<int16_t>){+3814, +626, +4179};
}


// checking data from IMU to see if robot is picked up
void CheckIMU(){
  compass.read();
  imu_angle = compass.heading();
  Serial.print(imu_angle);
  Serial.print("  ");
  Serial.print(a.theta);
  Serial.print("  ");
  Serial.println(facing);
}



// setting z angle of robot
// outputs 1 when looking in correct direction
int SetDir(){
  if(imu_angle < ((float)a.theta - 2)){  // +/-2 a small range of acceptable angles
    Mr = 0;                     // for the robot to be facing, this way it system
    Ml = TURN_SPEED;                              // isn't constantly currecting its z-angle and never
    return 0;                            // allowing the robot to move forward.
  }
  if(imu_angle > ((float)a.theta + 2)){
    Mr = TURN_SPEED;
    Ml = 0;
    return 0;
  }
  return 1;  
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

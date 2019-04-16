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

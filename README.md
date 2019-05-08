# DDR_CS
Differential Drive Robot Control Structure for ECE 370.
Works under a new name of Wireless UDP Transport Bot, otherwise known as W.U.T. Bot.

# Setup
1. Pull the git onto your Raspberry Pi.
2. Run the installwutbot.sh bash script
3. Upload the DDR_CS.ino onto the microcontroller
4. Make sure your Pi can now connect to the Access Point with the ssid of "FeatherThyme" (or your own if you alter the .ino)
5. Run the wutbotpi script now located in your /usr/bin directory.
NOTE: Using the robot it different areas/buildings will require the IMU to be recalibrated. Please upload the "Calibrate" example from the IMU's Arduino library to the microcontroller and follow its instructions.  Plug its output into the SetIMU() of DDR_CS.ino

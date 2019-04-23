import socket
import time
import sys

UDP_HOME = "127.0.0.1"
UDP_IP = "192.168.1.1"
UDP_PORT = 5000

MIN_V = 0
MAX_V = 100
#Theta does not need min-max values as it can only be 1 of 4 values below

v = 0
theta = 0
time_to_send = 0
get_data = 0
endloop = 0

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

while(0==endloop):
	keypress = raw_input("Press ONE key!: ")

	if ("w" == keypress):		# w increases speed, e decreases speed.  both in increments of 5%
		v = v+5
		time_to_send = 1
	if ("e" == keypress):
		v = v-5
		time_to_send = 1
	if (" " == keypress):       # SPACE should stop the robot
		v = MIN_V
		time_to_send = 1
	if ( MIN_V > v):
		v = MIN_V
	if ( MAX_V < v):
		v = MAX_V
	if ("a" == keypress):       # a, s, d, e make the robot face North, East, South, and West respectively
		theta = 0               # which is 0, 270, 180, 90 degrees in the robot's frame
		time_to_send = 1
	if ("s" == keypress):
		theta = 270
		time_to_send = 1
	if ("d" == keypress):
		theta = 180
		time_to_send = 1
	if ("f" == keypress):
		theta = 90
		time_to_send = 1
	if ("q" == keypress):		# q requests odometry data
		time_to_send = 1
		get_data =1
	if ("p"== keypress):		# p ends python script
		endloop = 1
	
	# turn the integers into a string to be send to the robot
	# characters a and b are parsing and stoping characters for the microcontroler
	# to know which integer goes to which variable in the c struct it has
	vString = str(v)
	thetaString = str(theta)
	if (1 == get_data):
		MESSAGE = vString + 'a' + thetaString + 'q'
	MESSAGE = vString + 'a' + thetaString + 'b'

	# only send when the desired values are changed by user
	if (1 == time_to_send):
		print "Sending..."
		sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
		sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
		sock.close()
		time_to_send = 0;
	if (1 == get_data):
		sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
		sock.bind((UDP_HOME, UDP_PORT))
		data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
		print data
		sock.close()
		get_data = 0

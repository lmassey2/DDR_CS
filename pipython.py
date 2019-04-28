import socket
import time
import sys

UDP_HOME = ""
UDP_IP = "192.168.1.1"
SEND_UDP_PORT = 5000
RECV_UDP_PORT = 5005

MIN_V = 0
MAX_V = 100
#Theta does not need min-max values as it can only be 1 of 4 values below

v = 0
theta = 0
time_to_send = 0
get_data = 0
endloop = 0

sendsock = socket.socket(socket.AF_INET, # Internet
			socket.SOCK_DGRAM) # UDP

recvsock = socket.socket(socket.AF_INET, # Internet
			socket.SOCK_DGRAM) # UDP
recvsock.bind((UDP_HOME, RECV_UDP_PORT))

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
	if ("r" == keypress):		# r resets robot, useful after being picked ip
		time_to_send = 1
		time_to_reset = 1
		v = 0
		theta = 0
	if ("p" == keypress):		# p ends python script
		endloop = 1
	
	# turn the integers into a string to be send to the robot
	# characters a and b,q,r are parsing and stoping characters for the microcontroler
	# to know which integer goes to which variable in the c struct it has
	vString = str(v)
	thetaString = str(theta)
	MESSAGE = vString + 'a' + thetaString + 'b'
	if (1 == get_data):
		MESSAGE = vString + 'a' + thetaString + 'q'
	if (1 == time_to_reset):
		v = 0
		theta = 0;
		MESSAGE = vString + 'a' + thetaString + 'r'
		time_to_reset = 0


	# only send when the desired values are changed by user
	if (1 == time_to_send):
		print "Sending..."
		sendsock.sendto(MESSAGE, (UDP_IP, SEND_UDP_PORT))
		time_to_send = 0;
	if (1 == get_data):
		data, addr = recvsock.recvfrom(256) # buffer size is 1024 bytes
		print data
		get_data = 0

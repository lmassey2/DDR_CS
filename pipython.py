import socket
import time
import sys

UDP_IP = "192.168.1.1"
UDP_PORT = 5000

v = 0
theta = 0
time_to_send = 0
endloop = 0

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

while(0==endloop):
	keypress = raw_input("Press ONE key!: ")

	if ("w" == keypress):
		v = v+5
		time_to_send = 1
	if ("e" == keypress):
		v = v-5
		time_to_send = 1
	if ("a" == keypress):
		theta = 0
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
	if ("p"== keypress):
		endloop = 1
		
	vString = str(v)
	thetaString = str(theta)
	MESSAGE = vString + 'a' + thetaString + 'b'

	if (1 == time_to_send):
		print "Sending..."
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
		sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
		time_to_send = 0
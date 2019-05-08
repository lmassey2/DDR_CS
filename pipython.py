#Copyright <2019> <LIAM R. MASSEY>
#
#Redistribution and use in source and binary forms, with or without modification, are permitted 
#provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this list of conditions 
#and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, this list of 
#conditions and the following disclaimer in the documentation and/or other materials provided with 
#the distribution.
#
#3. Neither the name of the copyright holder nor the names of its contributors may be used to 
#endorse or promote products derived from this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
#IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
#FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
#IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
#OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import socket
import time
import sys
import curses

UDP_HOME = ""
UDP_IP = "192.168.1.1"
SEND_UDP_PORT = 5000
RECV_UDP_PORT = 4242

MIN_V = 0
MAX_V = 100
MIN_THETA = 0
MAX_THETA = 360

v = 0
theta = 0
time_to_send = 0
time_to_reset = 0
get_data = 0
endloop = 0

sendsock = socket.socket(socket.AF_INET, # Internet
			socket.SOCK_DGRAM) # UDP

recvsock = socket.socket(socket.AF_INET, # Internet
			socket.SOCK_DGRAM) # UDP
recvsock.bind((UDP_HOME, RECV_UDP_PORT))

# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

print "///////////////////////////////////////////////////////////////////////////////////\n"
print "Hello!  Welcome to the Wireless UDP Transport Robot, otherwise known as W.U.T. Bot.\n"
print "///////////////////////////////////////////////////////////////////////////////////\n\n"
print "Controls : \n"
print "UP_ARROW - speed up\n"
print "DOWN_ARROW - slow down\n"
print "SPACE_BAR - stop\n"
print "LEFT_ARROW - turn left 15 degrees\n"
print "RIGHT_ARROW - turn right 15 degrees\n"
print "w - face North\n"
print "a - face West\n"
print "s - face South\n"
print "s - face East\n"
print "q - display odometry data\n"
print "r - reset after pickup\n"
print "p - close this program\n\n"


while(0==endloop):
	keypress = screen.getch()

	if (curses.KEY_UP == keypress):
		v = v+10
		time_to_send = 1
	elif (curses.KEY_DOWN == keypress):
		v = v-10
		time_to_send = 1
	elif (curses.KEY_LEFT == keypress):
		theta = theta - 15
		time_to_send = 1
	elif (curses.KEY_RIGHT == keypress):
		theta = theta + 15
		time_to_send = 1
	elif (ord(' ') == keypress):
		v = MIN_V
		time_to_send = 1
	elif (ord('w') == keypress):
		theta = 0 
		time_to_send = 1
	elif (ord('d') == keypress):
		theta = 90
		time_to_send = 1
	elif (ord('s') == keypress):
		theta = 180
		time_to_send = 1
	elif (ord('a') == keypress):
		theta = 270
		time_to_send = 1
	elif (ord('q') == keypress):
		time_to_send = 1
		get_data = 1
	elif (ord('r') == keypress):
		time_to_send = 1
		time_to_reset = 1
		v = 0
		theta = 0
	elif (ord('p') == keypress):
		endloop = 1
		
	if ( MIN_V > v):
		v = MIN_V
	elif ( MAX_V < v):
		v = MAX_V
	if ( MIN_THETA > theta):
		theta = MIN_THETA + theta
	elif ( MAX_THETA < theta):
		theta = theta - MAX_THETA
		
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
		data, addr = recvsock.recvfrom(1024) # buffer size is 1024 bytes
		print data
		get_data = 0
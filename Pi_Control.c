/*  Make sure the wpa_supplicant.conf has the correct hostname and password
	to connect to the microcontroller access point!							*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#define BUFSIZE 1024

#define MIN_V 0
#define MAX_V 100
#define MIN_THETA 0
#define MAX_THETA 360


// structure containing speed and angle of robot
struct Velocity {
	int v;
	int theta;
};


// initialize global structure for speed and angle to 0
volatile struct Velocity a;



/* 
 * error - wrapper for perror
 */
void error(char *msg) {
    perror(msg);
    exit(0);
}




int main(){
	

	a.v = 0;
	a.theta = 0;
	///// NETWORKING /////
	int sockfd, portno, n;
    int serverlen;
    struct sockaddr_in serveraddr;
    struct hostent *server;
    char *hostname;
	
	hostname = "192.168.1.1";  // make sure hostname and password match with uC access point
    portno = "5000";
	printf("hostname: %s    portnum: %s\n", hostname, portno);
	
	/* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");

    /* gethostbyname: get the server's DNS entry */
    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        exit(0);
    }

    /* build the server's Internet address */
    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
	  (char *)&serveraddr.sin_addr.s_addr, server->h_length);
    serveraddr.sin_port = htons(portno);
	/////////////////////////
	
	
	
	///// GET USER/SEND DATA LOOP /////
	char x = 0;
	int time_to_send = 0;
	char outString[100];
	char vString[5];
	char thetaString[5];
    while(1){
        scanf("%c", &x);
		printf("x = %c\n", x);

        switch(x)					// similar to video games, W/S will put the speed of the robot up and down
        {                           // A/D will turn the robot left and right
            case 'W':               // each input changes the global desired values of the robot, meaning 
            case 'w':               // only holding down W won't necessarily make the robot move forware, but
                a.v = a.v + 5;      // after several presses of the W key the robot will move at a constant speed
		time_to_send = 1;
                break;
	    case 'E':
	    case 'e':
		a.v = a.v - 5;
		time_to_send = 1;
		break;
            case 'A':
            case 'a':
                a.theta = 0;  // north
		time_to_send = 1;
                break;
            case 's':
            case 'S':
                a.theta = 270; // east
		time_to_send = 1;
                break;
            case 'D':
            case 'd':
                a.theta = 180; // south
		time_to_send = 1;
                break;
	    case 'F':
	    case 'f':
		a.theta = 90;  // west
		time_to_send = 1;
		break;

        }
		if (MIN_V > a.v){
			a.v = MIN_V;
		}
		if (MAX_V < a.v){
			a.v = MAX_V;
		}
		if (MIN_THETA > a.theta){
			a.theta = MIN_THETA;
		}
		if (MAX_THETA < a.theta){
			a.theta = MAX_THETA;
		}
		if (1 == time_to_send){
			sprintf(vString, "%d", a.v);
			sprintf(thetaString, "%d", a.theta);
			printf("a.v = %s\n", vString);
			printf("a.theta = %s\n", thetaString);
			strcpy(outString, vString);
			strcat(outString, "a");
			strcat(outString, thetaString);
			strcat(outString, "b");
			
			// now send the struct over UDP
			serverlen = sizeof(serveraddr);
			n = sendto(sockfd, outString, sizeof(outString), 0, &serveraddr, serverlen);
			if (n < 0){
				error("ERROR in sendto");
			}
			time_to_send = 0;
			memset(outString,0,sizeof(outString));
			memset(vString,0,sizeof(vString));
			memset(thetaString,0,sizeof(thetaString));
		}
    }
	
}

//////////// OLD SKELETON CODE //////////////

// ConnectToPort();  connect to the correct UDP port to contact the microcontroller
// InitStruct();  initialize data structures
// while(1){
// 		x = UserInput();  get WASD input from user
//  	v, theta = Parse(x);  gather desired speed and angle direction from user input
// 		a.v = v;
// 		a.theta = theta;
//		Send(a);			put speed and angle into a struct and send over UDP
// }						|||can also use netcat to make sure data is being sent|||
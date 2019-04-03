#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#define BUF_SIZE 512
#define PACKET_SIZE 35
#define PACKETS_TO_STORE 100000
#define PORT 9930

//#define SERVER_IP "10.0.0.13" //Home PC
//#define SERVER_IP "192.168.43.119" //Manuel's Laptop on Brock's Hotspot
#define SERVER_IP "68.44.201.181" //Home PC seperate network
//#define SERVER_IP "174.231.25.85" //Manuel's laptop

#define CONFIRM_TIMEOUT 15 //seconds
#define SIGNAL_THRESHOLD -57.5 //dbFS


int set_interface_attribs(int fd, int speed, int parity);
void set_blocking(int fd, int should_block);
int init_gps_read();

float get_rf_strength();
void get_GPS_data(int fd, char * lat, char * lng, float * speed);
time_t get_time();
int get_signal_weight(float sig_strength, float speed);

int main(void)
{
	//RF VARIABLES
	float sig_strength = 0;
	//float old_sig_strength = 0;
	int sig_weight;
	//GPS VARIABLES
	char lat[10];
	char lng[10];
	float speed;
	int fd;
	//UDP VARIABLES
	char packet[PACKET_SIZE];
	char buf[BUF_SIZE];
	struct sockaddr_in si_other;
	int sock;
	int slen = sizeof(si_other);
	int msg_len;
	socklen_t len;
	//TIME VARIABLES
	time_t epoch_time;
	struct timeval tv;
	tv.tv_sec = CONFIRM_TIMEOUT;
	tv.tv_usec = 0;
	//FILE VARIABLES
	FILE * fptr;
	int file_size;

	//Bind to a socket for data transfer
	if((sock = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP)) == -1)
	{
		perror("socket");
		exit(1);
	}
	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	if(inet_aton(SERVER_IP, &si_other.sin_addr) == 0)
	{
		fprintf(stderr,"inet_aton() failed!\n");
		exit(1);
	}
	
	//Check if there are any stored packets needing sent
	if((fptr = fopen("packet_storage", "r")) == NULL)
	{
		printf("Error Opening Packet Storage File!");
		exit(1);
	}

	//Get file size, if 0 there are no packets to offload
	fseek(fptr, 0L, SEEK_END);
	file_size = ftell(fptr);
	
	//If packets are stored, offload them	
	if(file_size != 0)
	{
		//Go back to start of file
		fseek(fptr,0, SEEK_SET);
		while(ftell(fptr) < file_size)
		{
			fscanf(fptr,"%s\n",packet);
			printf("sending packet. . .\n");
			printf("%s\n",packet);
			if(sendto(sock, packet, PACKET_SIZE, 0, (struct sockaddr *) &si_other, slen) == -1)
			{
				perror("sendto() failed!, is hotspot on?");
				//failed to connect to hotspot, reboot and try again
				system("sudo reboot");
			}
			printf("sent packet\n");
			//set timeout
			setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

			//clear buffer
			memset(buf, '\0', BUF_SIZE);

			//Receive confirmation message
			//if timeout is met, reboot to re-attempt offload
			len = sizeof si_other;
			msg_len = recvfrom(sock, (char *)buf, BUF_SIZE, MSG_WAITALL, (struct sockaddr *) &si_other,&len);
			if(msg_len == -1)
			{
				printf("confirmation timeout, %s\n",strerror(errno));
				//reboot to attempt to reconnect
				system("sudo reboot");
			}
			buf[msg_len] = '\0';
			printf("%s\n",buf);

			//Throttle packet offload to avoid overload on server end
			sleep(.5);

		}
		//clear packet storage when finished
		fclose(fopen("packet_storage","w"));
		
	}
	fclose(fptr);
	



	//Initialize serial read port for GPS data
	fd = init_gps_read();

	//clear initial screen
	system("clear");
	
	//hide cursor
	system("tput civis");

	//Loop that runs the main process for the duration of the power being on
	while(1)
	{
		//Check the RF signal strength
		sig_strength = get_rf_strength();

		//Display RF signal strength in console for debugging
		system("tput cup 1 0");
		printf("Signal Strength: %.1f\n", sig_strength);
		
		//Check for abnormal "noise" level in signal strength,
		//Trigger packet creation and attempt to send if abnormal signal
		if(sig_strength > SIGNAL_THRESHOLD)
		{
			//Set up screen for new data
			
			//clear potential connection timeout message
			system("tput cup 14 0");
			printf("                                                      \n");
			printf("              \n");
			system("tput cup 3 0");
			printf("LAST CAPTURE\n");
			printf("=====================================\n");

			//get the gps data and display in console for debugging
			get_GPS_data(fd,lat, lng, &speed);
			printf("Location saved: lat:%s,lng:%s,mph:%f\n", lat,lng,speed);
			
			//Calculate signal weight
			sig_weight = get_signal_weight(sig_strength, speed);
			printf("Signal weight: %d\n",sig_weight);

			//Get and print current time in epoch
			epoch_time = get_time();
			printf("Current epoch time: %ld\n",(long) epoch_time);

			//Create packet to send to server and
			//print to console for debugging
			sprintf(packet,"%ld,%d,%s,%s",epoch_time,sig_weight,lat,lng);
			printf("Packet: %s\n",packet);

			printf("destination IP: %s\n", SERVER_IP);
			//Alert console of packet sending for debugging
			printf("sending packet. . .\n");
			if(sendto(sock, packet, PACKET_SIZE, 0, (struct sockaddr *) &si_other, slen) == -1)
			{
				perror("sendto() failed! Is hotspot on?");
				//failed to connect to hotspot, reboot and try again
				system("sudo reboot");
			}
			printf("sent packet\n");
			
			//set timeout
			setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

			//clear buffer
			memset(buf, '\0', BUF_SIZE);

			//Recieve confirmation message
			//if timeout is met, store the packet until max packets are stored
			//once max packets are stored, reboot system to try to solve the issue and offload
			len = sizeof si_other;
			msg_len = recvfrom(sock, (char *)buf, BUF_SIZE, MSG_WAITALL, (struct sockaddr *) &si_other,&len);
			if(msg_len == -1)
			{
				printf("confirmation timeout, %s\n",strerror(errno));
				//Open packet storage file in append mode
				if((fptr = fopen("packet_storage", "a")) == NULL)
				{
					printf("Error Opening Packet Storage File for append!");
					exit(1);
				}
				
				//store up to max packets, then reboot to attempt to reconnect
				//and offload packets stored
				if(ftell(fptr)/PACKET_SIZE < PACKETS_TO_STORE)
				{
					fprintf(fptr,"%s\n",packet);
					fclose(fptr);
					printf("packet stored!\n");
				}
				else
				{
					//REBOOT
					fclose(fptr);
					printf("Storage Full!\n");
					system("sudo reboot");
				}

			}
			else
			{
			
				printf("packet received\n");
			}
			buf[msg_len] = '\0';
			printf("%s\n",buf);

		}
		
		//Save the signal strength for comparein demo version
		//old_sig_strength = sig_strength;
	}
}

float get_rf_strength()
{
	char buffer[6];
	FILE *fptr;
	float sig_strength;

	system("sh ./RF_Detect/rfdetect");
	
	if((fptr = fopen("rf_strength", "r")) == NULL)
	{
		printf("Error Opening File!");
		exit(1);
	}
	
	fscanf(fptr, "%[^\n]", buffer);
	fclose(fptr);
	
	sig_strength = atof(buffer);
	return sig_strength;
}
int init_gps_read()
{
	char *portname = "/dev/ttyACM0";

	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
        	printf("error opening GPS serial port\n");
			exit(1);
	}

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking
	
	return fd;
}
void get_GPS_data(int fd, char * lat, char * lng, float * speed)
{
	char buf[66];
	char temp[6];
	char DD[3];
	char SS[9];
	int DDint;
	float coord = 0;
	int done = 0;
	
	while(done == 0)
	{
		read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
		buf[sizeof buf] = '\0';
		
		//For debugging GPS data
		//printf("%s\n",buf);
		
		strncpy(temp,buf+1,5);
		
		//Check to make sure it is GPRMC formated
		//if not read again
		if(strcmp("GPRMC",temp) == 0)
		{
			//Get latitude
			strncpy(DD,buf+19,2);
			strncpy(SS,buf+21,8);
			DDint = atoi(DD);
			coord = atof(SS);
			coord = coord/60 + DDint;
			if(buf[30] == 'S')
			{
				coord = coord *-1;
			}
			sprintf(lat,"%f",coord);
			
			//Get longitude
			strncpy(DD,buf+33,2);
			strncpy(SS,buf+35,8);
			DDint = atoi(DD);
			coord = atof(SS);
			coord = coord/60 + DDint;
			if(buf[44] == 'W')
			{
				coord = coord *-1;
			}
			sprintf(lng,"%f",coord);
			
			strncpy(temp, buf+46, 5);
			*speed = atof(temp) * 1.15078;
	
			done = 1;
		}
	}
}

time_t get_time()
{
	time_t	now;
	struct	tm ts;
	char	buf[80];
	
	//get time
	time(&now);
	return now;
}

int get_signal_weight(float sig_strength, float speed)
{
	int sig_weight;
	
	printf("Signal Strength: %.1f dbFS\n",sig_strength);
	printf("Speed: %f mph\n",speed);

	sig_weight = (50 * (-30/sig_strength)) + (10 * (1 - (speed / 120)));
	
	if(sig_weight > 99 || sig_weight < 0)
	{
		sig_weight = 0;
	}

	return sig_weight;
}

int set_interface_attribs(int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                //error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking(int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error setting term attributes");//error_message ("error %d setting term attributes", errno);
}


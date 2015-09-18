#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

#ifndef __UART_H__
#define __UART_H__

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop );
int open_port(int fd,int comport);

#endif

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
  struct termios newtio,oldtio;
  if ( tcgetattr( fd,&oldtio) != 0) { 
    perror("SetupSerial 1");
    return -1;
  }
  bzero( &newtio, sizeof( newtio ) );
  newtio.c_cflag |= CLOCAL | CREAD; 
  newtio.c_cflag &= ~CSIZE; 

  switch( nBits )
  {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }

  switch( nEvent )
  {
  case 'O':
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'E': 
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'N': 
    newtio.c_cflag &= ~PARENB;
    break;
  }

switch( nSpeed )
  {
  case 2400:
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case 4800:
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case 9600:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case 115200:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }
  if( nStop == 1 )
    newtio.c_cflag &= ~CSTOPB;
  else if ( nStop == 2 )
  newtio.c_cflag |= CSTOPB;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcflush(fd,TCIFLUSH);
    
  if((tcsetattr(fd,TCSANOW,&newtio))!=0)
  {
    perror("com set error");
    return -1;
  }
  printf("set done!\n");
  return 0;
}

int open_port(int fd,int comport)
{
  char *dev[]={"/dev/ttyUSB0","/dev/s3c2410_serial0","/dev/s3c2410_serial1","/dev/s3c2410_serial2"};
  long vdisable;
  if (comport==1)
  { 
    fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
    if (-1 == fd){
      perror("Can't Open Serial Port");
      return(-1);
    }
    else 
      printf("open ttyUSB0 .....\n");
  }
  else if (comport==2)
  { fd = open( "/dev/s3c2410_serial0", O_RDWR|O_NOCTTY|O_NDELAY);
    if (-1 == fd){
      perror("Can't Open Serial Port");
      return(-1);
    }
    else 
      printf("open ttyS0 .....\n");
  }
  else if(comport==3)
  { fd = open( "/dev/s3c2410_serial1", O_RDWR|O_NOCTTY|O_NDELAY);
    if (-1 == fd){
      perror("Can't Open Serial Port");
      return(-1);
    }
    else 
      printf("open ttyS1 .....\n");
  }
  else if (comport==3)
  {
    fd = open( "/dev/s3c2410_serial2", O_RDWR|O_NOCTTY|O_NDELAY);
    if (-1 == fd){
      perror("Can't Open Serial Port");
      return(-1);
    }
    else 
      printf("open ttyS2 .....\n");
  }
  
  if(fcntl(fd, F_SETFL, 0)<0)
    printf("fcntl failed!\n");
  else
    printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
  if(isatty(STDIN_FILENO)==0)
    printf("standard input is not a terminal device\n");
  else
    printf("isatty success!\n");
  printf("fd-open=%d\n",fd);
  return fd;
}

void Send_Message(void)
{
	int num;
	int fd = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
 	unsigned char cmd1[] = "AT+CMGF=1\r\n";
	unsigned char cmd2[] = "AT+CSCS=\"UCS2\"\r\n";
	unsigned char cmd3[] = "AT+CSMP=17,167,2,25\r\n";
	unsigned char cmd4[] = "AT+CMGS=\"00310033003600370038003000350034003200310035\"\r\n";
	unsigned char cmd5[] = "8BF76CE8610FFF0C6709964C751F4EBA8FDB5165FF01\r\n";
	unsigned char cmd6[] = "0x1A";
	
	write(fd,cmd1, 11);//10
	printf("send %s success!,size = %d \n",cmd1,sizeof(cmd1));
	sleep(1);
	
	write(fd,cmd2, 16);//15
	printf("send %s success!,size = %d \n",cmd2,sizeof(cmd2));
	sleep(1);
	
	write(fd,cmd3, 21);//20
	printf("send %s success!,size = %d \n",cmd3,sizeof(cmd3));
	sleep(1);

	write(fd,cmd4, 56);//55
	printf("send %s success!,size = %d \n",cmd4,sizeof(cmd4));
	sleep(5);

    write(fd,cmd5, 46);//45
	printf("send %s success!,size = %d \n",cmd5,sizeof(cmd5));
	sleep(1);
	
	write(fd,cmd6, 1);//2
	printf("send %s success!\n",cmd6);
	sleep(10);

	//sleep(5);
}

int main(void)
{
    int fd,i;
	
    if((fd=open_port(fd,1))<0)
	{
        perror("open_port error");
        return;
    }
	  
    if((i=set_opt(fd,9600,8,'N',1))<0)
	{
        perror("set_opt error");
        return;
    }
    
    Send_Message();
    
    return 0;
}

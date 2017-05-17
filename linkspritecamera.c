
/*
read image from linksprite serial camera from serial port
*/

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss


unsigned char serialport[] = "/dev/ttyUSB2";
unsigned char smallbuffer[99]="";

unsigned int a=0x0000,i=0, j=0, k=0, count=0;   //Read Starting address and indexes        
unsigned char MH,ML;				//length of image return data
unsigned int jpegsize;				//size of jpeg image
unsigned int fd, n;				// file description for the serial port and counter
unsigned char jpegstop=1, lastchar=0;		//end of jpeg marker

       FILE *jpegfile_ptr;

struct termios  config;
 
void SendResetCmd();
void SendResizeCmd();
void SendTakePhotoCmd();
void SendReadDataCmd();
void StopTakePhotoCmd();
void open_serial_port() ;
void SendData();
void ReadData();
void GetjpegFileSize();
void ReadImageData();
void StopTakePhotoCmd();



void main() 
{

  jpegfile_ptr = fopen("imagetest.jpg","wb");			//FILE TO SAVE JPEG

  if(jpegfile_ptr !=NULL)					//test if file is open and ready
	{
	printf("imagetest.jpg created.\n\n\n");
	} else
	{
	printf("could not create imatetest.jpg.\n\n\n");
	return;
	}

     open_serial_port();					// open the serial port for linksprite

     printf("open_port %s main program starting.\n",serialport);

     printf("Trying to send camera command data.\n");		// send camera control commands
     SendResetCmd();
     SendResizeCmd();
//     SendResetCmd();
     SendTakePhotoCmd();
     GetjpegFileSize();
     sleep(1);
     ReadImageData();
     StopTakePhotoCmd();


  fclose(jpegfile_ptr);						//all done, so close file
  close(fd);							//close serial port
} //main


 

//Send Reset command
void SendResetCmd()
{
 char resets[]={0x56,0x00,0x26,0x00};				//command string
 write(fd,&resets,4);						//send command string to serial port
 usleep(1000000); 						// delay to get return data
 printf("Trying to read reset return data.\n");
 ReadData();							// return data from camera
}


//Send Resize command
void SendResizeCmd()
{
 char resizel[]={0x56,0x00,0x54,0x01,0x00};
 char resizem[]={0x56,0x00,0x54,0x01,0x11};
 char resizes[]={0x56,0x00,0x54,0x01,0x22};
 write(fd,&resizel,5);
 usleep(1000000); // delay to get return data
 printf("Trying to read resize return data.\n");
 ReadData();
}

 
//Send take picture command
void SendTakePhotoCmd()
{
 char take[] = {0x56,0x00,0x36,0x01,0x00};
 write(fd,&take,5);
 usleep(1000000); // delay to get return data
 printf("Trying to read take picture return data.\n");
 ReadData();
}

//Send get jpeg file size command
void GetjpegFileSize()
{
 unsigned char get[] = {0x56,0x00,0x34,0x01,0x00};
 write(fd,&get,5);
 usleep(1000000); // delay to get return data
 printf("Trying to read get jpeg file size return data.\n");
 ReadData();
 jpegsize = smallbuffer[7]*256+smallbuffer[8];
 printf("The picture size is %d bytes.\n", jpegsize);
}

 
//Read image data
void ReadImageData()
{
 printf("read image starting\n");

 unsigned int MM=0, MMH=0, KK=32, KKH=0,zero=0;
 unsigned char readdataf[] = {0x56,0x00,0x32,0x0c,0x00,0x0a,0x00,0x00};
 unsigned char readdatal[] = {0x00,0x0a};
 unsigned char stop = 1;
 
 while(jpegstop)
 {
 write(fd,&readdataf,8);
 MMH=MM>>8;
 write(fd,&MMH,1);
 write(fd,&MM,1);
 write(fd,&zero,2);
 write(fd,&KKH,1);
 write(fd,&KK,1);
 write(fd,&readdatal,2);
 
 usleep(30000); // delay to get return data

 printf("Trying to read readimagedata data.\n");
 ReadData();

  for(k=5;k<37;k++)					//extract substring from return data
   {
   fputc(smallbuffer[k],jpegfile_ptr);			//save substring to disk file
	if((lastchar==0xff) && (smallbuffer[k]==0xd9))	//test for jpeg end marker
          {
          jpegstop=0;
	  }
	lastchar=smallbuffer[k];
   }

 printf("MM is %d.\n",MM);
 printf("jpegstop %d.\n\n\n", jpegstop);
 MM+=32;                            		//address increases set according to buffer size
 }
}

 
void StopTakePhotoCmd(void)
{
 unsigned char stop[] = {0x56,0x00,0x36,0x01,0x03};        
 write(fd,&stop,5);
 usleep(100000); // delay to get return data
 printf("Trying to read stop photo return data.\n");
 ReadData();
 printf("\n\n\n");
}



void ReadData(void)
{
	int index=0;
	n = read(fd,&smallbuffer,90);
	printf("%d  ",n);
	
	for(index=0;index<n;index++)
        {
	printf("%x ", smallbuffer[index] );
	}
 printf("\n\n");
}



void open_serial_port(void) {

fd = open(serialport, O_RDWR | O_NOCTTY ); //| O_NDELAY);
if(fd == -1) // if open is unsucessful
 {
  printf("open_port: Unable to open %s.\n",serialport);
  } else {
  fcntl(fd, F_SETFL, 0);
  printf("port %s is open.\n",serialport);
 }

 if(!isatty(fd)) {
 //... error handling ... 
 printf("open_port %s is not a serial port.\n",serialport);
 } else {
 printf("open_port %s is a serial port.\n",serialport);
 }

 /*
 * Get the current options for the port...
 */

 tcgetattr(fd, &config);

 //
 // Input flags - Turn off input processing
 //
 // convert break to null byte, no CR to NL translation,
 // no NL to CR translation, don't mark parity errors or breaks
 // no input parity check, don't strip high bit off,
 // no XON/XOFF software flow control
 //
 config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR |
 		     PARMRK | INPCK | ISTRIP | IXOFF | IXON);

 //
 // Output flags - Turn off output processing
 //
 // no CR to NL translation, no NL to CR-NL translation,
 // no NL to CR translation, no column 0 CR suppression,
 // no Ctrl-D suppression, no fill characters, no case mapping,
 // no local output processing
 //

 
 //config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
 //                     ONOCR | OFILL | OLCUC | OPOST);
 config.c_oflag &= ~OPOST;

 //
 // No line processing
 //
 // echo off, echo newline off, canonical mode off, 
 // extended input processing off, signal chars off
 //

 config.c_lflag &= ~(ECHO | ECHONL | ICANON | XCASE |
		     ECHOE | ECHOK | ECHONL | IEXTEN | ISIG);

 //	
 // Turn off character processing
 //
 // clear current char size mask, no parity checking,
 // no output processing, force 8 bit input
 //
 config.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
 config.c_cflag |= (CS8 | CSTOPB | CREAD | CLOCAL);

 //
 // One input byte is enough to return from read()
 // Inter-character timer off
 //
 config.c_cc[VMIN]  = 1;
 config.c_cc[VTIME] = 5;
 //
 // Communication speed (simple version, using the predefined
 // constants)
 //
 if(cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0) {
 // ... error handling ...
 printf("open_port %s could not set baud rate to 38400.\n",serialport);
 }

 //
 // Finally, apply the configuration
 //
 if(tcsetattr(fd, TCSAFLUSH, &config) < 0) {
 //... error handling ... 
 printf("open_port %s applied the configuration failed.\n",serialport);
 }else {
 printf("open_port %s applied the configuration suceeded.\n",serialport);
 }

} //open_serial_port



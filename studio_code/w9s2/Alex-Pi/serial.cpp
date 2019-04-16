#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "serial.h"


static int _fd;
static struct termios _serOptions;

void startSerial(const char *portName, int baudRate, int byteSize, char parity, int stopBits, int maxAttempts)
{
	_fd = -1;

	int attempt=0;
	while(_fd < 0 && attempt < maxAttempts)
	{
	  printf("ATTEMPTING TO CONNECT TO SERIAL. ATTEMPT # %d of %d.\n", ++attempt, maxAttempts);
   	_fd=open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

		if(_fd < 0)
		{
		  printf("FAILED. TRYING AGAIN IN 5 SECONDS\n");
		  sleep(5);
		}
	}
    
    if(_fd < 0)
		{
      perror("GIVING UP. Unable to open serial port.");
		}
    else
    {
        fcntl(_fd, F_SETFL, 0);
 
        tcgetattr(_fd, &_serOptions);
				cfmakeraw(&_serOptions);
        cfsetispeed(&_serOptions, baudRate);
        cfsetospeed(&_serOptions, baudRate);

        _serOptions.c_cflag|=(CLOCAL | CREAD);

        switch(parity)
        {
                case 'o':
                case 'O':
                _serOptions.c_cflag|=PARENB;
                _serOptions.c_cflag|=PARODD;
                _serOptions.c_iflag |= (INPCK | ISTRIP);
                break;

                case 'e':
                case 'E':
                _serOptions.c_cflag |= PARENB;
                _serOptions.c_cflag &= ~PARODD;
                _serOptions.c_iflag |= (INPCK | ISTRIP);
                break;
                
            default:
                _serOptions.c_cflag&= ~ PARENB;
                break;
            
        }
        
        if(stopBits==2)
            _serOptions.c_cflag |= CSTOPB;
        else
            _serOptions.c_cflag &= ~CSTOPB;
        
        _serOptions.c_cflag &= ~CSIZE;
        
        
        switch(byteSize)
        {
            case 5:
                _serOptions.c_cflag |= CS5;
                break;
                
            case 6:
                _serOptions.c_cflag|=CS6;
                break;
                
            case 7:
                _serOptions.c_cflag|=CS7;
                break;
                
            default:
                _serOptions.c_cflag|=CS8;
        }
    }
    
    // Disable hw flow control
    _serOptions.c_cflag &= ~CRTSCTS;
    
    // Disable sw flow control
    _serOptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    
    // Clear canonical input mode
    _serOptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
		/*
	// Set canonical input mode
	_serOptions.c_lflag |= ICANON; */

    // Set canonical output mode
   //_serOptions.c_oflag |= OPOST;

	//_serOptions.c_oflag &= ~OPOST;
	_serOptions.c_oflag &= OPOST;

    // Set the attributes
    tcsetattr(_fd, TCSANOW, &_serOptions);
}

int serialRead(char *buffer)
{
	ssize_t n=0;

	if(_fd >= 0)
		n = read(_fd, buffer, MAX_BUFFER_LEN);
	
	return n;
}

void serialWrite(char *buffer, int len)
{
	ssize_t n=-1;
	if(_fd >= 0)
	{
		n = write(_fd, (void *) buffer, len);
	}
}

void endSerial()
{
	if(_fd > 0)
		close(_fd);
}

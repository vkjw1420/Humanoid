#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int main()
{
	int fd;
	int key=0;


	if((fd = serialOpen("/dev/ttyAMA0", 4800)) < 0)
	{
		fprintf(stderr, "Unable to open: %s\n", strerror (errno));
		return 1;
	}

	if(wiringPiSetup () == -1)
	{
		fprintf(stdout, "Unable to start wiringPi : %s\n", strerror (errno));
		return 1;
	}

	while(key != -1){
		fscanf(stdin, "%d%*c", &key);
		printf("%d : inputted\n", key);
		fflush(stdout);
		serialPutchar(fd, key);
		delay(300);
		
		while(serialDataAvail(fd))
		{
			printf("-> %d\n", serialGetchar(fd));
			fflush(stdout);
			delay (300);
		} 
	}

	printf("\n");
	return 0;
}


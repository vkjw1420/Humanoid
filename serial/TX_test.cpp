#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
 
#include <wiringPi.h>
#include <wiringSerial.h>
 
int main ()
{
	int fd ;
	int count ;
	char order ;
	unsigned int nextTime ;
 
	if ((fd = serialOpen ("/dev/ttyAMA0", 4800)) < 0)  // 두번째 인자값이 보레이트 설정
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;                   
		return 1 ;
	}
 
	if (wiringPiSetup () == -1)
	{
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
		return 1 ;
	}
 
	nextTime = millis () + 300 ;
 
	for (count = 0 ; count < 256 ; )
	{
		if (millis () > nextTime)
		{
			printf ("\nOut: %d: ", count) ;
			fflush (stdout) ;

			order = getchar();
			while( getchar() != '\n' );

			serialPutchar (fd, atoi(&order)) ; // 데이터 전송해주는 함수

			nextTime += 300 ;
			++count ;
		}

		delay (3) ;
	}
 
	printf ("\n") ;
	return 0 ;
}


//when you want to compail, typing like this. "gcc -o ' ' ' ' -lwiringPi"
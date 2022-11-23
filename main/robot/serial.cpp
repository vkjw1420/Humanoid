#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int sendCommand(int fd, int key) //명령을 보내기 위해 사용
{
   int isEnd=0;
   int received=0;

   printf("%d : sending \n", key);
   fflush(stdout);
   serialPutchar(fd, key);
   delay(100);// 상황에 따라 변경

   while(isEnd != 1){
      while(serialDataAvail(fd))
      {
         received = serialGetchar(fd)
         printf("%d : received \n", received);
         if(received == 38) isEnd=1;
         
         fflush(stdout);
         delay (100);
      } 
   }
   return 0;
}

int initSerial(){ //시리얼 초기화
   int fd;

   if((fd = serialOpen("/dev/ttyAMA0", 4800)) < 0)
   {
      fprintf(stderr, "Unable to open: %s\n", strerror(errno));
      return -1;
   }

   if(wiringPiSetup() == -1)
   {
      fprintf(stdout, "Unable to start wiringPi : %s\n", strerror(errno));
      return -1;
   }

   return fd;
}
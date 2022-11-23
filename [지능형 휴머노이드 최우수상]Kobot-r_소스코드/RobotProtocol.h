#ifndef _ROBOT_PROTOCOL_
#define _ROBOT_PROTOCOL_
#include <iostream>
#include <cstdlib>  
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdio>
#include<fstream>

void Uart_open(void);
void Init_console(void);
void Motion(unsigned char move_value);
void ProcessMotion(unsigned char move_value) ;
unsigned char Check_Read();
unsigned char Check_remote(); 
void Uart_close(void);
void Motion2(unsigned char move_value);
unsigned char Check_Read2();

#endif //_ASDFROBOT_PROTOCOL


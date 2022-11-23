#include "kobot.h"
///
///
int main(int argc, char *argv[])
{
	Uart_open() ; // 통신 준비
	Init_console() ; 

	usleep(1000000) ;
	Check_remote() ;


	
	std::cout << " start " << std::endl ;

///////각 경기 실행 코드///////////////----------------------------------------------------------
//	hockey_passer();
//	hockey_shooter();
	Koo();
//	Curling();
///////////////////////////////////

	Uart_close() ;
	return 0;
}

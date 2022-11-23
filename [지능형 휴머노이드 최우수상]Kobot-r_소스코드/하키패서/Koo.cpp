#include "RobotProtocol.h"
#include "Koo.h"
////////
using namespace std;
using namespace cv;
int temp_j=-1,temp_i=-1;
cv::Mat Frame;
cv::Point p1, p2;

cv::Point start,end;
float MAX_LEN=0.0;
cv::Vec4i LINE;
std::vector<cv::Vec4i> lines;
std::vector<cv::Vec4i> lines2;

cv::Vec3b V_PURPLE = cv::Vec3b(255, 255, 255);

cv::Mat image;
cv::Mat image2;

pthread_mutex_t frameLocker;
pthread_t updateThread;
cv::VideoCapture capture;

void *updateFrame(void* arg){
	while(1){
		cv::Mat tempframe;
		capture >> tempframe;
		pthread_mutex_lock(&frameLocker);
		Frame = tempframe;
		pthread_mutex_unlock(&frameLocker);
		cv::resize(Frame, Frame, cv::Size(320, 240), 0, 0, CV_INTER_NN);
	}
	pthread_exit((void *)0);
}

int flag = 1;
int hit_flag = 0;
int firsthit=0;
int firstturn=1;
int whereball=0;
int koominjun=0;
int hitcount=0;

int zeroonetwo=1;
int loopcount =0;
void Koo(){
	cv::Mat currentFrame, currentFrame2, currentFrame3;
	cv::Mat img_hsv, img_binary;
	cv::Mat img_hsv2, img_binary2;
	cv::Mat img_hsv3, img_binary3;

	//int findleftpuckI, findleftpuckJ;
	int dx, dy;
	int dx2, dy2;
	double degree;
	double degree2;
	int findleftpuck_flag=0; //&&&&& 0
	int findyellow_flag=0;

	int findpuckhead_flag= 0;

	uchar *WW, *EE, *NN, *SS, *CC, *WN, *WS, *EN, *ES;

	capture.open(0);
	pthread_mutex_init(&frameLocker, NULL);
	pthread_create(&updateThread, NULL, updateFrame, NULL);

	if(flag==0){
		Motion2(159);
		Motion2(191); //숙이는 건데 나중에 확인하기
 	}

	int nofindfirstleftpuck=0;
	int nofindfirstunderpuck=0;
	int nofindyellow01=0;
	int nofindyellow02=0;
	int findleftpuckI=0, findleftpuckJ = 0;
	int findleftpuckII=0, findleftpuckJJ=0;

				int findyellowlineI1=0, findyellowlineI2=0, findyellowlineJ1=0, findyellowlineJ2=0;
			int findleftpuckI01=0, findleftpuckJ01=0;

	float distance;
	//int nolook=0;
	//int hello = 0;
	while(true){
		
		pthread_mutex_lock(&frameLocker);
		currentFrame = Frame;
		pthread_mutex_unlock(&frameLocker);

		if(currentFrame.empty()) continue;

		if(flag==0){ //퍽들 중 제일 왼쪽 것 찾기
			findleftpuckI=0, findleftpuckJ = 0;
			cvtColor(currentFrame, img_hsv, cv::COLOR_BGR2HSV);

//			inRange(img_hsv, cv::Scalar(100,120,65), cv::Scalar(120, 255, 255), img_binary); //학교파랑
			inRange(img_hsv, cv::Scalar(85,50,25), cv::Scalar(120, 255, 255), img_binary); //오후 2시 10분

			cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
			cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

			cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
			cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
			
			for(int i=2; i<318 ; i++){
				int j;
				for(j=2 ; j<238 ; j++){
					WW = img_binary.data + j*img_binary.step + (i-1)*img_binary.elemSize() + 0;
					EE = img_binary.data + j*img_binary.step + (i+1)*img_binary.elemSize() + 0;
					NN = img_binary.data + (j-1)*img_binary.step + i*img_binary.elemSize() + 0;
					SS = img_binary.data + (j+1)*img_binary.step + i*img_binary.elemSize() + 0;
					CC = img_binary.data + j*img_binary.step + i*img_binary.elemSize() + 0;
					WN = img_binary.data + (j-1)*img_binary.step + (i-1)*img_binary.elemSize() + 0;
					WS = img_binary.data + (j+1)*img_binary.step + (i-1)*img_binary.elemSize() + 0;
					EN = img_binary.data + (j-1)*img_binary.step + (i+1)*img_binary.elemSize() + 0;
					ES = img_binary.data + (j+1)*img_binary.step + (i+1)*img_binary.elemSize() + 0;
//					std::cout <<"여기서안오는건가??????"<<std::endl;
					if((int)*WW == 255 && (int)*EE ==255 && (int)*NN == 255 && (int)*SS == 255 && (int)*CC == 255
						&& (int)*WN == 255 && (int)*WS == 255 && (int)*EN == 255 && (int)*ES == 255){
						circle(currentFrame, cv::Point(i,j), 2, cv::Scalar(0,255,0), 3, cv::LINE_AA);
						//std::cout << "I : " << i << ", J : " << j << std::endl;
						findleftpuckI = i; findleftpuckJ = j;
						i = 320; j = 240;
					}
				}if(i==317 && j==237) nofindfirstleftpuck=1;
			}

			if(nofindfirstleftpuck==1){
				nofindfirstleftpuck=0;
				continue;
			}
			if(findleftpuckI < 120){
				
				 /////////////////////////////////////////////////////			
				Motion2(146);
			}
			else if(findleftpuckI < 164)
				 Motion2(103); //@@@ 수치조정
			else if(findleftpuckJ < 197) Motion2(135); //@@@ 수치조정 앞으로 한걸음 걷기

			if(findleftpuckI >= 164 && findleftpuckJ >=197){
				Motion2(113);
				Motion2(113);
				Motion2(113);
				Motion2(113); //@@@@@ 이거 아니면 129번임
				Motion2(113);
				Motion2(106); usleep(1000000);
				Motion2(106); 
				//Motion2(106);
				flag = 1;
				hit_flag=101;
		//		//Motion2(163); //숙이기 164번은 더숙임 내일 확인
			}
		}
		else if(flag==1){ //이제 앞으로 가면서 퍽을 확인할 것
			findleftpuckI01=0, findleftpuckJ01=0;
			if(findpuckhead_flag==0){
				Motion2(159);
				Motion2(163);
				usleep(1000000);
				findpuckhead_flag=1;
				continue;
			}else if(findpuckhead_flag==2){
				Motion2(159);
				Motion2(192); //정면 60도 더 앞으로봄
				usleep(1000000);
				findpuckhead_flag=3;
				continue;
			}else if(findpuckhead_flag==4){
				Motion2(163); //전방하향 40
				Motion2(155); //머리 오른쪽 30도
				usleep(1000000);
				findpuckhead_flag=5;
				continue;
			}else if(findpuckhead_flag==6){
				Motion2(163);
				Motion2(151);//머리 왼쪽 30도
				usleep(1000000);
				findpuckhead_flag=7;
				continue;
			}
			if(hit_flag==0){
				//Motion2(159);
				//Motion2(163);
				cvtColor(currentFrame, img_hsv, cv::COLOR_BGR2HSV);
//				img_hsv3 = img_hsv.clone();
				inRange(img_hsv, cv::Scalar(85,50,25), cv::Scalar(120, 255, 255), img_binary);
//				inRange(img_hsv, cv::Scalar(100,120,65), cv::Scalar(120, 255, 255), img_binary); //학교파랑

				cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
				cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

				cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
				cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

				//머리 왼쪽에 있을 땐 옐로라인왼쪽부분 자르기 시작 #@@@@@@@@@@@@@@@@@@@@@@
				if(findpuckhead_flag==7 || findpuckhead_flag==1 || findpuckhead_flag==3){
					std::cout << "오류예상1" << std::endl;
					cvtColor(currentFrame, img_hsv3, cv::COLOR_BGR2HSV);
					std::cout << "오류예상2" << std::endl;
	
					//inRange(img_hsv3, cv::Scalar(25, 30, 30), cv::Scalar(32, 255, 255), img_binary3); //노란색
					inRange(img_hsv3, cv::Scalar(18,50,42), cv::Scalar(50, 200, 155), img_binary3); //노란색 오후 2시 10분
				//	inRange(img_hsv3, cv::Scalar(25,120,200), cv::Scalar(32,255,255), img_binary3); //학교
					
					cv::erode(img_binary3, img_binary3, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
					cv::dilate(img_binary3, img_binary3, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
	
					cv::dilate(img_binary3, img_binary3, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
					cv::erode(img_binary3, img_binary3, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
				
					//img_binary : 파랑, img_binary3 : 노랑
					
					yellow_cut(img_binary3 , img_binary, 1);
					
				}
				//머리 왼쪽에 잇을 땐 옐로라인왼쪽부분 자르기 끝 # @@@@@@@@@@@@@@@@@@@@@@@@@

				for(int j = 235 ; j>2 ; --j){
					int i;
					for(i = 315 ; i>2 ; --i){
						WW = img_binary.data + j*img_binary.step + (i-1)*img_binary.elemSize() + 0;
						EE = img_binary.data + j*img_binary.step + (i+1)*img_binary.elemSize() + 0;
						NN = img_binary.data + (j-1)*img_binary.step + i*img_binary.elemSize() + 0;
						SS = img_binary.data + (j+1)*img_binary.step + i*img_binary.elemSize() + 0;
						CC = img_binary.data + j*img_binary.step + i*img_binary.elemSize() + 0;
						WN = img_binary.data + (j-1)*img_binary.step + (i-1)*img_binary.elemSize() + 0;
						WS = img_binary.data + (j+1)*img_binary.step + (i-1)*img_binary.elemSize() + 0;
						EN = img_binary.data + (j-1)*img_binary.step + (i+1)*img_binary.elemSize() + 0;
						ES = img_binary.data + (j+1)*img_binary.step + (i+1)*img_binary.elemSize() + 0;

						if((int)*WW == 255 && (int)*EE ==255 && (int)*NN == 255 && (int)*SS == 255 && (int)*CC == 255
							&& (int)*WN == 255 && (int)*WS == 255 && (int)*EN == 255 && (int)*ES == 255){
							//circle(currentFrame, cv::Point(i,j), 2, cv::Scalar(0,255,0), 3, cv::LINE_AA);
							findleftpuckI01 = i; findleftpuckJ01 = j;
							//std::cout << "I : " << i << ", J : " << j << std::endl;
							i = 1; j = 1;
						}
					}
					if(j==3 && i==3) {nofindfirstunderpuck=1; std::cout << "세그멘테이션 오류가 어디서날까?"<<std::endl;}
				}

//				findleftpuckI01 = temp_i; findleftpuckJ01=temp_j;
				if(nofindfirstunderpuck==1){
					nofindfirstunderpuck=0;
					continue;
				} 

				
				if(findleftpuckI01==0 && findpuckhead_flag==7){ //왼쪽에도 없다면 고개를 좀 앞으로 들것
					//Motion2(135);
					findpuckhead_flag=2;
					continue;
				}else if(findleftpuckI01==0 && findpuckhead_flag==1){ //밑에 엎다면 고개를 오른쪽으로 할것
					findpuckhead_flag=4;
					continue;
				}else if(findleftpuckI01==0 && findpuckhead_flag==5){ //오른쪽에도 없다면 고개를 왼쪽으로 할것
					findpuckhead_flag=6;
					continue;
				}else if(findleftpuckI01==0 && findpuckhead_flag==3){ //앞에 좀 들어도 없으면 뒤로 3걸음
					Motion2(136); Motion2(136); Motion2(136);
					findpuckhead_flag=0;
					hit_flag=101; findyellow_flag=0;whereball=1;
					continue;
				}
				if(findleftpuckI01 < 139 && (findpuckhead_flag==1 || findpuckhead_flag==3)){Motion2(146); usleep(800000); } //좌걷기
				else if(findleftpuckI01 < 159 && (findpuckhead_flag==1 || findpuckhead_flag==3)){Motion2(103); usleep(800000); } //좌걷기
				else if(findleftpuckI01 > 208 && (findpuckhead_flag==1 || findpuckhead_flag==3)){Motion2(147); usleep(800000);} //우걷기
				else if(findleftpuckI01 > 188 && (findpuckhead_flag==1 || findpuckhead_flag==3)){Motion2(104); usleep(800000);} //우걷기
				else if(findleftpuckJ01 < 150 && (findpuckhead_flag==1 || findpuckhead_flag==3)){Motion2(135); usleep(1000000); } //멀리서는 조금 크게 걷기
				else if(findleftpuckJ01 < 190 && (findpuckhead_flag==1 || findpuckhead_flag==3)){Motion2(134); usleep(1000000);} //앞으로 걷기인데 만약에 넘 작으면 135
				else if(findleftpuckJ01 > 230 && (findpuckhead_flag==1 || findpuckhead_flag==3)){Motion2(136); usleep(1000000);} //후진

				if(findleftpuckI01 < 159 && findpuckhead_flag==5){Motion2(146); Motion2(146); usleep(800000);} //우걷기
				else if(findleftpuckI01 > 200 && findpuckhead_flag==5){Motion2(147); Motion2(147); usleep(800000);} //우걷기
				
				if(findpuckhead_flag==7){std::cout <<"I01 : " << findleftpuckI01 << std::endl;}
				if(findleftpuckI01 < 159 && findpuckhead_flag==7){Motion2(146); Motion2(146); usleep(800000);}
				else if(findleftpuckI01 > 200 && findpuckhead_flag==7){Motion2(147); Motion2(147);  usleep(800000);}

				if(findpuckhead_flag==5 || findpuckhead_flag==3 || findpuckhead_flag==7){ findpuckhead_flag=0;} //&&&&&&&&& flag=0;
				
				if(findleftpuckI01 >= 159  && findleftpuckI01 <= 188 && findleftpuckJ01 >= 190  && findleftpuckJ01 <= 230 && hit_flag==0 && findpuckhead_flag==1){
					//std::cout << "왼쪽아래보냐" << std::endl;
					findyellow_flag=0;
					hit_flag = 101; //&^&&&&7 hit_flag=101;
				}
			}
			else if(hit_flag == 101){//노란선으로 각도를 맞출 것이야.
				findyellowlineI1=0, findyellowlineI2=0, findyellowlineJ1=0, findyellowlineJ2=0;

				if(findyellow_flag==0){
					if(zeroonetwo==0)
						Motion2(164);//30도
					else if(zeroonetwo==1)
						Motion2(163); //163 40도
					else if(zeroonetwo==2)
						Motion2(191); //55도
					Motion2(153);
					usleep(1000000);
					findyellow_flag=1;
				}

				cvtColor(currentFrame, img_hsv, cv::COLOR_BGR2HSV);

				//inRange(img_hsv, cv::Scalar(25, 30, 30), cv::Scalar(32, 255, 255), img_binary); //노란색
				inRange(img_hsv, cv::Scalar(18,50,42), cv::Scalar(50, 200, 155), img_binary); //노란색 오후 2시 10분
	//			inRange(img_hsv, cv::Scalar(25,120,200), cv::Scalar(32,255,255), img_binary); //학교
				
				cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
				cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

				cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
				cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
				

				MAX_LEN=0.0;
				if(img_binary.empty()){ findyellow_flag=0;  continue; }
				image = img_binary.clone();

				//cv::cvtColor(image, image, cv::CV_RGB2GRAY);

				cv::Canny(image ,image, 30, 90, 3);

				cv::HoughLinesP(image, lines, 1, PI / 100, 30, 30, 3);
				if(lines.size()==0){
					std::cout <<"없다 라인이" <<std::endl;
					loopcount+=1;
					if(zeroonetwo==0){
						Motion2(147);
					}
					if(loopcount==3){
						loopcount=0;
						zeroonetwo += 1;
						zeroonetwo = zeroonetwo % 3;
						Motion2(147);
						Motion2(147);
						Motion2(147);
						usleep(1000000);
					} 
					findyellow_flag=0; 
					
					continue;
				}
				else{
					zeroonetwo = 1;
					for(int i=0; i<lines.size(); i++){
						cv::Vec4i L = lines[i];
						p1 = cv::Point(L[0], L[1]);
						p2 = cv::Point(L[2], L[3]);
						
						cv::line(currentFrame, p1, p2, cv::Scalar(0,0,255), 1);

						if(p1.y > p2.y){
							cv::Point temp = p1;
							p1 = p2;
							p2 = temp;
						}
						float len;
						if(p2.x == p1.x)
							len = p2.y-p1.y;
						else if(p2.y == p1.y)
							len = p2.x-p1.x;
						else{
							float a = (float)(p2.y-p1.y) / (float)(p2.x-p1.x);
							int b =p1.y - a*p1.x;
							len = (p2.y - p1.y)*(p2.y-p1.y) + (p2.x- p1.x)*(p2.x-p1.x);
						}
						if(len > MAX_LEN){
							MAX_LEN = len;
							LINE = lines[i];
						}
					}
					p1 = cv::Point(LINE[0], LINE[1]);
					p2 = cv::Point(LINE[2], LINE[3]);
					if(p1.x==p2.x){
						start = cv::Point(p1.x, 2);
						end = cv::Point(p1.x, 238);
					}
					else if(p1.y == p2.y){
						start = cv::Point(2, p1.y);
						end = cv::Point(318, p1.y);
					}
					else{
						float a = (float)(p2.y-p1.y)/(float)(p2.x-p1.x);
						int b= p1.y - a*p1.x;
						int x= (0-b) / a;
						int y= 0;
						if(x<0){
							y=b;
							x=0;
						}
						else if(x>320){
							x=320;
							y=a*x + b;
						}
						start = cv::Point(x, y);
						y=240;
						x=(y-b) / a;
						if(x<0){
							y=b;
							x=0;
						}
						else if(x>320){
							x=320;
							y=a*x + b;
						}
						end = cv::Point(x, y);
					}
					cv::line(currentFrame2, start, end, cv::Scalar(255,0,0), 1);
					//imshow("cccc", currentFrame2);
				}
				dx = end.x - start.x;
				dy = end.y - start.y;
				if(dy==0. && dx==0.){findyellow_flag=0;  continue;}
				degree = atan2(dy, dx) * (180.0 / PI) ;
				std::cout << "degree : " << degree << std::endl;

				if(firsthit==0){
					if(degree < 180. && degree > 160. || degree > 0. && degree <50.){
						Motion2(106); //139
 						usleep(1000000);
					}else if(90. < degree && degree  < 154.) {
						Motion2(105);
					//	std::cout << " 왼쪽으로 회전하는데 : " << degree << std::endl;
						usleep(1000000);
					}
					else if(degree <= 160. && degree >= 154.){
						//std::cout << "startx : " << start.x << ", endx : " << end.x << std::endl;
						//std::cout << "starty : " << start.y << ", endy : " << end.y << std::endl;
						float  aa = (float)(start.y-end.y) / (float)(start.x-end.x) ;
						float bb = (float)start.y - (float)(aa * start.x);
						distance = aa * 160.0 + bb;
						std::cout << "distance : " << distance << std::endl;
						if(whereball==1){
							hit_flag=0;
							findpuckhead_flag=0;
							whereball=0;
							continue;
						}
						hit_flag = 102;
						findpuckhead_flag=0;
						
					}

				}
				else if(firsthit==1){
					if(degree <180. && degree > 160. || degree >0. && degree < 50.){
						Motion2(106);
						usleep(1000000);
					}else if(90. <degree && degree < 154.){
						Motion2(105);
						usleep(1000000);
					}
					else if(degree <= 160. && degree >= 154.){
						float aa = (float)(start.y-end.y) / (float)(start.x-end.x);
						float bb = (float)start.y - (float)(aa * start.x);
						distance = aa * 160.0 +  bb;
						std::cout << "distance : " << distance << std::endl;
						
						if(distance==0) continue;
						if(firstturn==1){
							flag=1;
							hit_flag=0;
							firstturn=0;
							findpuckhead_flag=0;
							continue;
						}
						if(whereball==1){
							hit_flag=0;
							findpuckhead_flag=0;
							whereball=0;
							continue;
						}
						Motion2(106);usleep(1000000); 

						hit_flag=102;
						findpuckhead_flag=0;
						firsthit=0;
					}
				}
			}
			else if(hit_flag==102){
				//std::cout <<"지금 102번안들어가는거같은데"<<std::endl;
				cvtColor(currentFrame, img_hsv, cv::COLOR_BGR2HSV);
			//	inRange(img_hsv, cv::Scalar(100,120,65), cv::Scalar(120, 255, 255), img_binary); //학교파랑

				inRange(img_hsv, cv::Scalar(80,50,25), cv::Scalar(120, 255, 255), img_binary); // 2시 10분
//				inRange(img_hsv, cv::Scalar(100, 120, 65), cv::Scalar(120, 255, 255), img_binary);

				cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
				cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

				cv::dilate(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
				cv::erode(img_binary, img_binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

				for(int jj = 238 ; jj>2 ; --jj){
					int ii;
					for(ii = 315 ; ii>2 ; --ii){
						WW = img_binary.data + jj*img_binary.step + (ii-1)*img_binary.elemSize() + 0;
						EE = img_binary.data + jj*img_binary.step + (ii+1)*img_binary.elemSize() + 0;
						NN = img_binary.data + (jj-1)*img_binary.step + ii*img_binary.elemSize() + 0;
						SS = img_binary.data + (jj+1)*img_binary.step + ii*img_binary.elemSize() + 0;
						CC = img_binary.data + jj*img_binary.step + ii*img_binary.elemSize() + 0;
						WN = img_binary.data + (jj-1)*img_binary.step + (ii-1)*img_binary.elemSize() + 0;
						WS = img_binary.data + (jj+1)*img_binary.step + (ii-1)*img_binary.elemSize() + 0;
						EN = img_binary.data + (jj-1)*img_binary.step + (ii+1)*img_binary.elemSize() + 0;
						ES = img_binary.data + (jj+1)*img_binary.step + (ii+1)*img_binary.elemSize() + 0;
						//std::cout <<"여기안들어가지지금!!!!!!!!"<<std::endl;
						if((int)*WW == 255 && (int)*EE ==255 && (int)*NN == 255 && (int)*SS == 255 && (int)*CC == 255
							&& (int)*WN == 255 && (int)*WS == 255 && (int)*EN == 255 && (int)*ES == 255){
							circle(currentFrame, cv::Point(ii,jj), 2, cv::Scalar(0,255,0), 3, cv::LINE_AA);
							findleftpuckII = ii; findleftpuckJJ = jj;
					//		std::cout << "IIIIIIII : " <<ii << ", JJJJJJJJJJ : " << jj << std::endl;
							ii = 1; jj = 1;
						}
					}
					if(jj==3 && ii==3) {nofindfirstunderpuck=1;}				}
				if(nofindfirstunderpuck==1){
					//std::cout <<" dfldsafadsfdsfdsafsaf " << std::endl;
					nofindfirstunderpuck=0;
					continue;
				}

				//std::cout << " findleftpuckII, JJ  ::::: " << findleftpuckII << ", " << findleftpuckJJ << std::endl;
				if(findleftpuckII < 196){Motion2(103); usleep(1000000); } //좌걷기
				else if(findleftpuckII > 219){Motion2(104); usleep(1000000);} //우걷기
				else if(findleftpuckJJ < 205){Motion2(134); usleep(1000000);} //앞으로 걷기인데 만약에 넘 작으면 135
				else if(findleftpuckJJ > 235){Motion2(133); usleep(1000000);} //후진

				if(findleftpuckII >= 196  && findleftpuckII <= 219  && findleftpuckJJ >= 205  && findleftpuckJJ <= 235 && hit_flag==102){
					//std::cout << "왼쪽아래보냐" << std::endl;
					hit_flag=1;
					//hello = 1;
					//nolook=2;
					findyellow_flag=0;
		
				}
			}
			else if(hit_flag == 1){
				if(distance < 50){

					Motion2(141);
					hitcount += 1;
					findyellow_flag=0;
					//Motion2(141); //친다.
					findpuckhead_flag=0;
					flag=2;
				}else if(distance >= 50 && distance < 100){
					Motion2(141);
					hitcount += 1;
					findyellow_flag=0;
					findpuckhead_flag=0;
					flag=2;
				}else if(distance >= 100){
					Motion2(140);
					hitcount+=1;
					findyellow_flag=0;
					findpuckhead_flag=0;
					flag=2;
				}
			}
		}
		else if(flag==2){
//			Motion2(136); //뒤로 한걸음
			findpuckhead_flag=0;
			findyellow_flag=0;
			flag = 1;
			hit_flag=0;
			usleep(1000000);
		}
		//imshow("currentFrame", currentFrame);
		//imshow("img_binary", img_binary);
//		imshow("curr2", currentFrame2);
//
		if(cv::waitKey(10)==27) break;
	}
}



void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G, int* R){
	uchar *data = image.data;
	uchar *blue, *green, *red;
 
	int nRow = image.rows;
	int nCol = image.cols;
 
	if (i > nCol - 1 || j > nRow - 1) {
		//cout << "Selected pixel is out of range." << endl;
		return;
	}
 
	blue = image.data + j*image.step + i*image.elemSize() + 0;
	if (is_color) {
		green = image.data + j*image.step + i*image.elemSize() + 1;
		red = image.data + j*image.step + i*image.elemSize() + 2;
	}
	*B = (int)*blue;
	if (is_color) { *G = (int)*green; *R = (int)*red; }
}
 

void SetColor(bool is_color, cv::Mat& image, int j, int i, int B, int G, int R)
{
	uchar *data = image.data;
	uchar *blue, *green, *red;
 
 
	int nRow = image.rows;
	int nCol = image.cols;
 
	if (i > nCol - 1 || j > nRow - 1) {
		//cout << "Selected pixel is out of range." << endl;
		return;
	}
 
	blue = image.data + j*image.step + i*image.elemSize() + 0;
	if (is_color) {
		green = image.data + j*image.step + i*image.elemSize() + 1;
		red = image.data + j*image.step + i*image.elemSize() + 2;
	}
	*blue = B;
	if (is_color) { *green = G;	*red = R; }
 
}
 
bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x) {
	int R, G, B;
 
	GetColor(0, image, y, x, &B, &G, &R);
	//std::cout<<B<<" "<<G<<" "<<R<<std::endl;
	return (B == color[0]);
}








int START,END,FIND_LINE;

void yellow_cut(Mat inImage , Mat& blue_image, bool is_cut){
	Mat image = inImage.clone();

//	cvtColor(image, image, CV_RGB2GRAY);

	Canny(image,image,30,90,3);

	vector<Vec4i> lines;

	HoughLinesP(image, lines, 1, CV_PI /100,30,30,3);

	float MAX_LEN=0.0, MAX_DEGREE=-360,degree[lines.size()];
	Vec4i LINE,LINE2;
	if(lines.size() != 0){
		for(int i =0; i<lines.size();i++){
			Vec4i L = lines[i];
			Point p1 = Point(L[0],L[1]);
			Point p2 = Point(L[2],L[3]);

			line(inImage, p1, p2, Scalar(0,0,255),1);

			if(p1.y>p2.y){
				Point temp = p1;
				p1 = p2;
				p2 = temp;
			}

			float len;

			if(p2.x == p1.x){
				len = p2.y-p1.y;
				degree[i] = 90;
			}
			else if (p2.y== p1.y){
				len = p2.x-p1.x;
				degree[i] = 0;
			}
			else{
				float a = (float)(p2.y-p1.y)/(float)(p2.x-p1.x);
				int b = p1.y - a*p1.x;
				len = (p2.y-p1.y)*(p2.y-p1.y) + (p2.x-p1.x)*(p2.x-p1.x);
				degree[i] = atan2((float)(p2.y-p1.y),(float)(p2.x-p1.x)) *(180.0/ 3.14);
			}
			
			
			if(len>MAX_LEN) {
		//		cout<<"lenFIND"<<endl;
				MAX_LEN = len;
				MAX_DEGREE = degree[i];
				LINE = lines[i];
			}
			

		}
		//Imshow("all",inImage);

		FIND_LINE = 1;
			Point p1 = Point(LINE[0],LINE[1]);
			Point p2 = Point(LINE[2],LINE[3]);
		draw_LINE(inImage, p1, p2);
			//P_START = start; P_END = end;
		
		cut(inImage,blue_image,START,END);
	
	/*	for(int i=318; i>2;i--){
			for( int j = 238;j>2;j--){
				if(COLOR(blue_image,V_PURPLE,j,i)) {temp_i = i; temp_j = j; i = 0; j=0;}
			}
		}
*/
		//imshow("cut",blue_image);

		//return true;
	}
	else {
		FIND_LINE = 0;

		//return false;
	}

}


void cut(Mat& image, Mat& blue_image, int P1, int P2){
	int state;

	int down_before[6][2]={{0,3},{0,7},{3,5},{3,4}, {4,7}, {4,5}}; // 위에서 내려오며 만날때까지 지운다.
	int up_after[4][2] = {{1,3}, {1,7}, {2,3}, {2,7} }; // 아래에서 위로 가며 만난뒤로 지운다.
	int left_after[8][2] = {{0,1},{0,2},{0,5},{1,5},{1,4} , {2,5} , {2,4},{5,7}}; //왼쪽으로 진행하며 만난 뒤로 지운다,
	int left_before[4][2] = {{0,6}, {1,6},{2,6},{6,4}} ;// 왼쪽으로 진행하며 만날때까지 지운다
	int right_after[2][2] = {{3,6},{7,6}}; // 오른쪽으로 진행하며 만난뒤로 지운다.

	cout<<P1<<"/"<<P2<<endl;
	for(int i =0; i<6; i++){
		if((P1==down_before[i][0] && P2==down_before[i][1]) || (P1==down_before[i][1] && P2==down_before[i][0]) ){ state = 0; break;}
	}

	for(int i =0; i<4; i++){
		if((P1==up_after[i][0] && P2==up_after[i][1]) || (P1==up_after[i][1] && P2==up_after[i][0]) ){ state = 1; break;}
	}

	for(int i =0; i<8; i++){
		if((P1==left_after[i][0] && P2==left_after[i][1]) || (P1==left_after[i][1] && P2==left_after[i][0]) ){ state = 2; break;}
	}

	for(int i =0; i<4; i++){
		if((P1==left_before[i][0] && P2==left_before[i][1]) || (P1==left_before[i][1] && P2==left_before[i][0]) ){ state = 3; break;}
	}

	for(int i =0; i<2; i++){
		if((P1==right_after[i][0] && P2==right_after[i][1]) || (P1==right_after[i][1] && P2==right_after[i][0]) ){ state = 4; break;}
	}
	cout<<"%%%%%"<<state<<endl;
	switch(state){
		case 0:		//			cout<<"//위에서 내려오며 만날때까지 지운다."<<endl;
			for(int i = 0; i<320; i++){
				for(int j =0; j<240; j++){
					if(COLOR(image,V_PURPLE, j, i)) j=240;
					SetColor(0,blue_image,j,i,0,0,0);
				}
			}
		break;
		case 1:		//			cout<<"// 아래에서 위로 가며 만난뒤로 지운다."<<endl;
			for(int i = 0; i<320; i++){
				bool find = false;
				for(int j =240; j>0; j--){
					if(COLOR(image,V_PURPLE, j, i)) find = true;
					if(find) SetColor(0,blue_image,j,i,0,0,0);
				}
			}
		break;
		case 2:		//			cout<<"//왼쪽으로 진행하며 만난 뒤로 지운다,"<<endl;
			for(int j =0; j<240; j++){
				bool find = false;
				for(int i = 320 ; i>0; i--){
					if(COLOR(image,V_PURPLE,j, i)) find = true;
					if(find) SetColor(0,blue_image,j,i,0,0,0);
				}
			}
		break;
		case 3:		//			cout<<"// 왼쪽으로 진행하며 만날때까지 지운다"<<endl;
			for(int j =0; j<240; j++){
				for(int i = 320 ; i>0; i--){
					if(COLOR(image,V_PURPLE, j, i)) i=0;
					SetColor(0,blue_image,j,i,0,0,0);
				}
			}
		break;
		case 4:		//			cout<<"// 오른쪽으로 진행하며 만난뒤로 지운다."<<endl;
			for(int j =0; j<240; j++){
				bool find = false;
				for(int i = 0; i<320; i++){
					if(COLOR(image,V_PURPLE, j, i)) find = true;
					if(find) SetColor(0,blue_image,j,i,0,0,0);
				}
			}
		break;
	}

}


void draw_LINE(cv::Mat& inImage, Point p1, Point p2){

	if(p1.x ==p2.x){
		start = Point(p1.x,0);
		end = Point(p1.x,240);
	}
	else if (p1.y ==p2.y){
		start = Point(0, p1.y);
		end = Point(320, p1.y);
	}
	else {
		float a = (float)(p2.y - p1.y) /(float)(p2.x-p1.x);
		int b = p1.y - a *p1.x;
		int x = (0-b) / a;
		int y = 0;
		if(x<0) {
			y = b;
			x = 0;
		}
		else if(x>320){
			x = 320;
			y = a*x +b;
		}
		start = Point(x, y);
		y = 240;
		x = (y-b) /a ;
		if(x<0) {
			y = b;
			x = 0;
		}
		else if(x>320){
			x = 320;
			y = a*x +b;
		}
		end = Point(x,y);
	}
//	cout<< start.x <<" " <<start.y<<" " <<end.x << " " <<end.y<<endl;
	line(inImage,start,end,Scalar(255,0,255),3);
	//imshow("line",inImage);

	if(start.x == 0){
		if(start.y <120) START = 0;
		else 			 START = 4 ;
	}
	else if ( start.x < 160){
		if(start.y <120) START = 1;
		else 			 START = 5 ;
	}
	else if (start.x<320){
		if(start.y < 120) START = 2;
		else			  START = 6;
	}
	else if (start.x ==320){
		if(start.y < 120) START = 3;
		else			  START = 7;
	}

	if(end.x == 0){
		if(end.y <120) 	END = 0;
		else 			END = 4 ;
	}
	else if ( end.x < 160){
		if(end.y <120) 	END = 1;
		else 			END = 5 ;
	}
	else if (end.x<320){
		if(end.y < 120) END = 2;
		else			END = 6;
	}
	else if (end.x ==320){
		if(end.y < 120) END = 3;
		else			END = 7;
	}
}


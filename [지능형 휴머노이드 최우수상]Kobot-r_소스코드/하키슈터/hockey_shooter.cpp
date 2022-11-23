#include "hockey_shooter.h"
#include "RobotProtocol.h"

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;


/*---------------Motion_num-----------------*/
#define BASE               99

#define W_forward_run         100
#define W_forward_repeat          101
#define W_backward_repeat         102
#define W_left_s            103
#define W_right_s            104
#define W_forward_slow_good      134
#define W_forward_slow_2         135
#define W_backward_slow_good         136
#define W_back_good          133
#define W_backward_good_1         132
#define W_left_m            110
#define W_right_m            111
#define W_left_large         146
#define W_right_large         147

#define T_left_20             105
#define T_right_20            106
#define T_left_90         114
#define T_left_large         112
#define T_right_large         113

#define T_left_middle         116
#define T_right_middle          117

#define T_left_good_1         115
#define T_left_good_2         138
#define T_right_good_1         118

#define A_L_start         114
#define A_L_walk         115
#define A_L_end            116
#define A_R_start         117
#define A_R_walk         118
#define A_R_end            119

#define A_back             130
#define A_forward          131

#define Hit                   120

#define T_left_set         122
#define T_right_set         123
#define T_left_3           124
#define T_right_3         125

#define H_base                  150
#define H_left_30             151
#define H_left_45             152
#define H_left_60             153
#define H_left_90             154
#define H_right_30            155
#define H_right_45            156
#define H_right_60            157
#define H_right_90              158
#define H_forward_LR          159
#define H_down_80             160
#define H_down_60             161
#define H_forward_UD          162
#define H_down_40             163
#define H_down_30             164
#define H_50                  190
#define H_55                  191
#define H_60                  192
#define H_65                  193
#define H_70                  194
#define H_80                  195
/*------------------------------------------*/


#define FIND_GOAL          119
#define LINE_COLOR         4

#define C_BLUE 1
#define C_GREEN 2
#define C_YELLOW 3
#define C_WHITE 10

cv::Mat Frame,FRAME,currentFrame, currentFrame2,Binaryframe, small_Binaryframe;
cv::Vec3b V_RED = cv::Vec3b(0, 0, 255);
cv::Vec3b V_BLUE = cv::Vec3b(0,255,255);
cv::Vec3b V_GREEN = cv::Vec3b(0, 255, 0);
cv::Vec3b V_YELLOW = cv::Vec3b(0, 255, 255);
cv::Vec3b V_PURPLE = cv::Vec3b(255, 0, 255);
cv::Vec3b V_WHITE = cv::Vec3b(255, 255, 255);
cv::Vec3b V_BLACK = cv::Vec3b(0, 0, 0);
cv::VideoCapture capture;

Vec3b V_LINE =V_YELLOW;

int list_H_UP[] = {H_down_30,H_down_60,H_down_80 };
int list_H_LR[] = {H_forward_LR, H_left_45,H_forward_LR, H_right_45};

int Blue_H_1 = 85,  Blue_S_1 = 30,  Blue_V_1 = 30;
int Blue_H_2 = 120,  Blue_S_2 = 255,  Blue_V_2 = 255;
int Green_H_1 = 40,  Green_S_1 = 70, Green_V_1 = 50;
int Green_H_2 = 65,  Green_S_2 = 255, Green_V_2 = 255;
int Yellow_H_1 = 18,  Yellow_S_1 = 220,  Yellow_V_1 = 60;
int Yellow_H_2 = 32,  Yellow_S_2 = 255,  Yellow_V_2 = 200;
int CENTER_Y = -1;
int T_cnt = 0, TracingIDX = 112;
int current_color, COLOR_NUM =0;
int INDEX = 0;
int FIRST_X = -1,FIRST_Y = -1,G_UP,G_DOWN,  G_LEFT, G_RIGHT,FIND = 0;
int center_X = 0, center_Y = 0,in = 159,walk_count = 0;
int R_H_UD = H_forward_UD, R_H_LR= H_forward_LR;
int START, END;
int green_approx_size = 0;
int DEGREE = -1;
int nofindfirstrightpuck = 0;
int nofindfirstunderpuck = 0;
int nofindyellow01 = 0;
int nofindyellow02 = 0;
int findrightpuckI = 0, findrightpuckJ = 0;
int findyellowlineI1 = 0, findyellowlineI2 = 0, findyellowlineJ1 = 0, findyellowlineJ2 = 0;
int findrightpuckI01 = 0, findrightpuckJ01 = 0;
int flag = 0;

bool GARO = false;
bool TRACING_GARO = true;
bool FIND_LINE;

pthread_mutex_t frameLocker;
pthread_t updateThread;

Point P_START,P_END,P2_START,P2_END;
Point start,end;


// 출력 확인 함수(한번에 주석 처리 가능)
void Cout(string str1, int int1, string str2, int int2){
   cout<< str1<<"  "<<int1<<"/"<<str2<<"  "<<int2<<endl;
}
// imshow 함수(한번에 주석 처리 가능)
void Imshow(string str, Mat image){
   imshow(str,image);
}


void *updateFrame(void* arg)
{
   while (1)
   {
      cv::Mat tempframe;
      capture >> tempframe;
      pthread_mutex_lock(&frameLocker);
      Frame = tempframe;
      pthread_mutex_unlock(&frameLocker);
   }
   pthread_exit((void *)0);
}

void getFrame(int COLOR1,int COLOR2, bool is_bin){
   cv::Mat b_image, y_image;
   current_color = COLOR1;
   while(1){
      cv::Mat image1,image;
      pthread_mutex_lock(&frameLocker);
      image = Frame;
      pthread_mutex_unlock(&frameLocker);

      if(image.empty()) continue;

      cv::resize(image,image,cv::Size(320,240),0,0,CV_INTER_NN);
      medianBlur(image,image1,3);
      if(is_bin){
         BinaryColors(image1,COLOR1,COLOR2,2);
         b_image = small_Binaryframe.clone();
         if(COLOR1 == C_BLUE &&COLOR2 != 10){
            BinaryColors(image1,C_YELLOW,0,1);
            y_image = small_Binaryframe.clone();
            if(R_H_LR == H_left_45 ||R_H_LR == H_left_90) GARO = true;
            green_cut(y_image,b_image);
            if(R_H_LR ==H_right_45||R_H_LR ==H_forward_LR)    yellow_cut(y_image , b_image);
         }
      }
      else
         b_image = image1;

      break;
   }
   if(INDEX >= 250 && INDEX <= 252){ Imshow("look",b_image);
      int b_cnt =0;
      for(int j=0; j<240;j++){
         for(int i = 0;i<320;i++){
            if(COLOR(b_image,V_GREEN,j,i))b_cnt++;
         }
      }
   }
   FRAME = b_image.clone();
}

int head_detect_count = 0;
int head_detect(){
   for(int i = 0; i < 4; i++){
      Motion2(list_H_LR[i]);
      R_H_LR = list_H_LR[i];
      for(int j = 0; j < 3; j++){
         Motion2(list_H_UP[j]);
         R_H_UD = list_H_UP[j];
         usleep(400000);
         INDEX = find_ball(INDEX);

         if(FIND_LINE) {
            if( i == 3) {
               j = 3;
               i = 4;//0
            }
            FIND_LINE = false;
         }
         if(INDEX == 6){
            head_detect_count = 0;
            return 6;
         }
      }
   }
   head_detect_count++;
   return INDEX;
}

int goal_idx;
int hit_count = 0;
int dir = 0;
void hockey_shooter() {
   Motion2(BASE);
   usleep(500000);
   Motion2(H_down_60);
   R_H_UD = H_down_60;
   R_H_LR = H_forward_LR;
   capture.open(0);

   pthread_mutex_init(&frameLocker, NULL);
   pthread_create(&updateThread, NULL, updateFrame, NULL);

   if( !capture.isOpened() ) {
      std::cerr << "Could not open camera" << std::endl;
      exit(1) ;
   }
   Cout("cameraOpen");

   int position = 0;
   int LRcnt = 0;

   while (1) {
      Motion2(R_H_UD);
      Motion2(R_H_UD);
      Motion2(R_H_LR);
      Motion2(R_H_LR);
      if (cv::waitKey(10) == 27)
         break;

      pthread_mutex_lock(&frameLocker);
      currentFrame2 = Frame;
      pthread_mutex_unlock(&frameLocker);

      if (currentFrame2.empty()) {
         continue;
      }
      Cout(">>>>>>>>>>>>>>>>>>>>>>>----------------------------" ,INDEX);

      currentFrame = currentFrame2;

      cv::resize(currentFrame2, currentFrame2, cv::Size(320, 240), 0, 0, CV_INTER_NN);
      cv::resize(currentFrame, currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

      Imshow("main",currentFrame);
      if (INDEX == 0 ) { INDEX = first_head_detect();   }
      else if (INDEX == 1) { INDEX = first_location(); }
      else if (INDEX == 2) { INDEX = head_detect(); }
      else if (INDEX == 6)  { INDEX = goto_start_position();}
      else if (INDEX == 7)  {
         INDEX = find_ball(INDEX);
         Motion2(H_forward_LR);
         R_H_LR = H_forward_LR;
         INDEX = 21;
      }
      else if (INDEX == 21) { INDEX = goto_ball_LR(); }
      else if (INDEX == 23) { INDEX = before_hit(); }
      else if (INDEX == 24) {
         Motion2(H_forward_LR);
         Motion2(H_down_30);
         R_H_UD = H_down_30;
         R_H_LR = H_forward_LR;
         INDEX = 25;
      }
      else if (INDEX == 25) { INDEX = find_puck(); }
      else if (INDEX == 26) { Motion2(Hit); usleep(1000000); hit_count++; head_detect_count = 0; INDEX++; }
      else if (INDEX == 27) {
         for(int i = 0; i < walk_count * 3 / 2;  i++){
            Motion2(W_backward_good_1);
            usleep(500000);
         }
         Motion2(H_down_40);
         Motion2(H_right_60);
         Motion2(H_down_40);
         Motion2(H_right_60);
         walk_count = 0;
         R_H_LR = H_right_60;
         R_H_UD = H_down_40;

         TRACING_GARO = false;
         TracingIDX = 27;
         INDEX = 112;
      }
      else if (INDEX == 28)  { Motion2(H_forward_UD); Motion2(H_forward_LR); usleep(1000000); INDEX = 2; }
      else if (INDEX == 119) { INDEX = find_goal_test(); }
      else if (INDEX == 112 ){ INDEX = lineTracing();}
      else if (INDEX == 111) {
         getFrame(C_YELLOW,C_BLUE);
         Mat y_img = FRAME.clone();
         Mat b_img = FRAME.clone();
         get_distance_line_puck(y_img,b_img);
      }
      else if (INDEX == IDX_SCAN) {
         SCAN();
         INDEX++;
      }
      else if(INDEX == IDX_SCAN+1){
            getFrame(C_BLUE,0);
            Imshow("blue",FRAME);
            getFrame(C_GREEN,0);
            Imshow("green",FRAME);
            getFrame(C_YELLOW,0);
            Imshow("yellow",FRAME);
      }
      else if (INDEX == L_BLUE){
         getFrame(C_BLUE, 10);
      }
      else if (INDEX == L_GREEN){
         getFrame(C_GREEN,0);
      }
      else if (INDEX == L_YELLOW){
         getFrame(C_YELLOW,0);
      }
      if (cv::waitKey(10) == 27){
         if(INDEX == IDX_SCAN + 1) INDEX = IDX_SCAN;
         else break;
      }
   }
   return;
}

int first_head_detect(){
   int first_head[2] = { H_down_60, H_forward_UD };

   for(int i = 0; i < 2; i++){
      Motion2(first_head[i]);
      INDEX = first_find_puck(INDEX);
      if(INDEX == 1)
         return 1;
   }
   return INDEX;
}


int first_find_puck(int INDEX){
   getFrame(C_BLUE,0);
   int blue_puck_count = 0;

   cv::Mat b_image = FRAME;

   for (int y = 0; y< b_image.rows; y++) {
      for (int x = 0; x < b_image.cols; x++) {
         if (COLOR(b_image, V_BLUE, y, x)) {
            blue_puck_count++;
         }
      }
   }

   Cout("blue_puck_count<<<",blue_puck_count);

   if(blue_puck_count < 50){
      Motion2(T_right_large);
      return INDEX;
   }
   else{
      Motion2(T_right_middle);
      Motion2(T_right_middle);
      return 1;
   }
   return INDEX;
}

bool lineDONE=false;
bool first_loc = true;
int lineTracing(){
   if(!TRACING_GARO) {
      R_H_UD = H_down_60; R_H_LR = H_right_60;
   }
   else
      Cout("GARO");

   Motion2(R_H_UD);
   Motion2(R_H_LR);
   usleep(500000);
   Mat image1,image2;
   GARO = true;
   getFrame(C_YELLOW,0);
   image1 = FRAME;
   image2 = image1.clone();

   yellow_cut(image1 , image2,0);

   if(FIND_LINE) Cout("FIND");
   else Cout("UNFIND");

   Imshow("trai",image1);
   if(FIND_LINE && R_H_UD == H_down_30 ){
      Motion2(W_backward_good_1);
      if(CENTER_Y < 80){
         Motion2(H_down_60);
         R_H_UD =H_down_60;
      }
      return INDEX;
   }
   if(!FIND_LINE && R_H_UD == H_down_30){
      Motion2(W_forward_slow_2);
      return INDEX;
   }
   if(!FIND_LINE && R_H_UD == H_down_60&&TRACING_GARO){
      R_H_UD = H_down_30;
      return INDEX;
   }
   if(!FIND_LINE){
      if(TRACING_GARO)
         Motion2(W_forward_slow_2);
      else
         Motion2(W_right_m);
      Cout("notfind");
      return INDEX;
   }
   else Cout("find");
   Cout(" *******Y",CENTER_Y);
   if(FIND_LINE){
      if(TRACING_GARO){
         if(CENTER_Y < 60) Motion2(W_forward_slow_2);
         if(CENTER_Y < 120) Motion2(W_forward_slow_2);
         if(CENTER_Y > 220) Motion2(W_backward_good_1);
      }
      else {
         if(CENTER_Y < 80) Motion2(W_right_m);
         if(CENTER_Y > 150) Motion2(W_left_m);
      }
   }
   if(TRACING_GARO){
      if(DEGREE > 170 && DEGREE < 175) {Motion2(T_left_middle); }
      else if(DEGREE > 90 && DEGREE < 170) {Motion2(T_left_large); }
      else if(DEGREE > 5 && DEGREE < 10) {Motion2(T_right_middle);}
      else if(DEGREE > 10 && DEGREE < 90) {Motion2(T_right_large); }
      else{
         if(lineDONE){
            lineDONE = false;
            Motion2(T_left_large);
            Motion2(T_left_large);
            Motion2(T_left_large);
            Motion2(T_left_large);
            Motion2(T_left_20);
            usleep(500000);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            first_loc = true;
            TRACING_GARO = false;

            return 112;
         }
         return TracingIDX;
      }
   }
   else{
      if(DEGREE < 20 && DEGREE > 0 ||( DEGREE < 180 && DEGREE > 120)){
         Motion2(105);
         usleep(500000);
      }
      else if (DEGREE > 32  && DEGREE < 120){
         Motion2(106);
         usleep(500000);
      }
      else {
         if(TracingIDX == 1){
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
            Motion2(W_backward_good_1);
         }
         return ++TracingIDX;
      }
   }

   DEGREE = -1;
   return INDEX;
}

//처음 위치로 이동
int first_location(){
   Motion2(H_forward_UD);
   cv::Mat img_hsv, img_binary;
   cv::Mat currentFrame01;

   uchar *WW, *EE, *NN, *SS, *CC, *WN, *WS, *EN, *ES;

   findrightpuckI = 0, findrightpuckJ = 0;
   if(currentFrame.empty()) return INDEX;
   cvtColor(currentFrame, img_hsv, cv::COLOR_BGR2HSV);

   inRange(img_hsv, cv::Scalar(Blue_H_1, Blue_S_1, Blue_V_1), cv::Scalar(Blue_H_2, Blue_S_2, Blue_V_2),currentFrame01);

   cv::erode(currentFrame01, currentFrame01, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
   cv::dilate(currentFrame01 , currentFrame01, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

   cv::dilate(currentFrame01 , currentFrame01, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
   cv::erode(currentFrame01, currentFrame01, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

   Imshow("DD", currentFrame01);TRACING_GARO = true;
   if(first_loc) {
      TracingIDX = INDEX;
      first_loc = false;
      R_H_UD = H_down_60;
      return 112;
   }
   TracingIDX= INDEX;
   R_H_UD = H_down_60;
   int a = lineTracing();
   if(a == TracingIDX+1) return ++INDEX;
   if(DEGREE > 170 && DEGREE < 175) {Motion2(T_left_20); return INDEX;}
   else if(DEGREE > 90 && DEGREE < 170) {Motion2(T_left_middle); return INDEX;}
   else if(DEGREE > 5 && DEGREE < 10) {Motion2(T_right_20); return INDEX;}
   else if(DEGREE > 10 && DEGREE < 90) {Motion2(T_right_middle); return INDEX;}
   DEGREE =-1;

   for(int i = 318; i > 2; i--){
      int j;
      for(j = 238; j > 2; j--){
         WW = currentFrame01.data + j*currentFrame01.step + (i-1)*currentFrame01.elemSize() + 0;
         EE = currentFrame01.data + j*currentFrame01.step + (i+1)*currentFrame01.elemSize() + 0;
         NN = currentFrame01.data + (j-1)*currentFrame01.step + i*currentFrame01.elemSize() + 0;
         SS = currentFrame01.data + (j+1)*currentFrame01.step + i*currentFrame01.elemSize() + 0;
         CC = currentFrame01.data + j*currentFrame01.step + i*currentFrame01.elemSize() + 0;
         WN = currentFrame01.data + (j-1)*currentFrame01.step + (i-1)*currentFrame01.elemSize() + 0;
         WS = currentFrame01.data + (j+1)*currentFrame01.step + (i-1)*currentFrame01.elemSize() + 0;
         EN = currentFrame01.data + (j-1)*currentFrame01.step + (i+1)*currentFrame01.elemSize() + 0;
         ES = currentFrame01.data + (j+1)*currentFrame01.step + (i+1)*currentFrame01.elemSize() + 0;

         if((int)*WW == 255 && (int)*EE == 255 && (int)*NN == 255 && (int)*SS == 255 && (int)*CC == 255
            && (int)*WN == 255 && (int)*WS == 255 && (int)*EN == 255 && (int)*ES == 255){
            circle(currentFrame, cv::Point(i,j), 2, cv::Scalar(0,255,0), 3, cv::LINE_AA);
            Cout("I : " , i ,", J : " ,j );
            findrightpuckI = i; findrightpuckJ = j;
            i = 1; j = 1;
         }
      }
      if(i == 1 && j == 1)
         nofindfirstrightpuck = 1;
   }

   if(nofindfirstrightpuck == 1){
      nofindfirstrightpuck = 0;
      Motion2(W_right_s);
      return INDEX;
   }
   if(findrightpuckI > 240){
      Motion2(W_right_m);
      Motion2(W_right_m);
      Motion2(W_right_m);
   }
   else if(findrightpuckI > 200){
      Motion2(W_right_m);
      Motion2(W_right_m);
   }
   else if(findrightpuckI > 150)
      Motion2(W_right_m);
   else if(findrightpuckJ != 0 && findrightpuckJ < 20)
      Motion2(W_forward_slow_2);

   if(findrightpuckI <= 150 && findrightpuckJ >= 20){
      TracingIDX= INDEX;
      lineDONE = true;
      return 112;
   }
   return INDEX;
}

int find_ball(int INDEX) {
   getFrame(C_BLUE, 0);
   cv::Mat temp_image = FRAME;
   Cout("find_ball_CNT",head_detect_count);
   DrawShapeDetection_Control(temp_image, &center_X, &center_Y);

   if((center_X == 0 || center_Y == 0) && INDEX == 7){
      INDEX = 2;
      return INDEX;
   }
   else if(center_X != 0 && center_Y != 0){
      return 6;
   }
   else if(head_detect_count > 2 && hit_count > 1){
      Motion2(W_backward_good_1);
      Motion2(W_backward_good_1);
      Motion2(W_backward_good_1);
      head_detect_count = 0;
      return INDEX;
   }
   else if(head_detect_count > 2){
      head_detect_count = 0;
      return INDEX;
   }
   return INDEX;

}

int goto_start_position() {
   getFrame(C_BLUE,0);
   cv::Mat b_image = FRAME;

   DrawShapeDetection_Control(b_image, &center_X, &center_Y);

   if(R_H_UD == H_down_30 && (R_H_LR == H_left_90 || R_H_LR == H_right_90)){
      if(center_X <= 300 && center_X != 0)
         Motion2(W_backward_good_1);
      else{
         if(R_H_LR == H_left_90 && center_Y <= 200 && center_Y != 0)
            Motion2(W_left_m);
         else if(R_H_LR == H_right_90 && center_Y >= 40 && center_Y != 0){
            dir = 1;
            Motion2(W_right_m);
         }
         else
            return ++INDEX;
      }
   }
   if(R_H_UD == H_down_60 && (R_H_LR == H_left_90 || R_H_LR == H_right_90)){
      if(center_X <= 250 && center_X != 0)
         Motion2(W_backward_good_1);
      else{
         if(R_H_LR == H_left_90 && center_Y <= 230 && center_Y != 0)
            Motion2(W_left_m);
         else if(R_H_LR == H_right_90 && center_Y >= 20 && center_Y != 0){
            dir = 1;
            Motion2(W_right_m);
         }
         else
            return ++INDEX;
      }
   }
   if(R_H_UD == H_down_80 && (R_H_LR == H_left_90 || R_H_LR == H_right_90)){
      if(center_X <= 300 && center_X != 0)
         Motion2(W_backward_good_1);
      else{
         if(R_H_LR == H_left_90 && center_Y <= 140 && center_Y != 0)
            Motion2(W_left_m);
         else if(R_H_LR == H_right_90 && center_Y >= 100 && center_Y != 0){
            dir = 1;
            Motion2(W_right_m);
         }
         else
            return ++INDEX;
      }
   }

   if(R_H_UD == H_down_30 && (R_H_LR == H_left_45 || R_H_LR == H_right_45)){
      if(center_Y >= 300 && center_Y != 0)
         Motion2(W_backward_good_1);
      else{
         if(R_H_LR == H_left_45 && center_X <= 200 && center_X != 0)
            Motion2(W_left_m);
         else if(R_H_LR == H_right_45 && center_X >= 40 && center_X != 0){
            dir = 1;
            Motion2(W_right_m);
         }
         else
            return ++INDEX;
      }
   }
   if(R_H_UD == H_down_60 && (R_H_LR == H_left_45 || R_H_LR == H_right_45)){
      if(center_Y >= 300 && center_Y != 0)
         Motion2(W_backward_good_1);
      else{
         if(R_H_LR == H_left_45 && center_X <= 260 && center_X != 0)
            Motion2(W_left_m);
         else if(R_H_LR == H_right_45 && center_X >= 80 && center_X != 0){
            dir = 1;
            Motion2(W_right_m);
         }
         else
            return ++INDEX;
      }
   }
   if(R_H_UD == H_down_80 && (R_H_LR == H_left_45 || R_H_LR == H_right_45)){
      if(center_Y >= 300 && center_Y != 0)
         Motion2(W_backward_good_1);
      else{
         if(R_H_LR == H_left_45 && center_X <= 280 && center_X != 0)
            Motion2(W_left_m);
         else if(R_H_LR == H_right_45 && center_X >= 80 && center_X != 0){
            dir = 1;
            Motion2(W_right_m);
         }
         else
            return ++INDEX;
      }
   }
   if(R_H_LR == H_forward_LR && (R_H_UD == H_down_30 || R_H_UD == H_down_60 || R_H_UD == H_down_80)){
      if(center_Y < 160 && R_H_UD == H_down_80)
         Motion2(W_forward_slow_2);
      if(center_Y < 120 && R_H_UD == H_down_60)
         Motion2(W_forward_slow_2);
      if(center_Y < 80 && R_H_UD == H_down_30)
         Motion2(W_forward_slow_good);
      return ++INDEX;
   }

   return INDEX;
}

bool LR_usleep =false;
int nothing = 0;
int where_is_the_ball = 0;
//찾은 방향으로 이동
int goto_ball_LR() {
   if(LR_usleep) {Cout("LRsleep"); usleep(500000);}
   getFrame(C_BLUE,0);
   cv::Mat b_image = FRAME;

   DrawShapeDetection_Control(b_image, &center_X, &center_Y);
   Cout("LR centerX:" ,center_X , "center Y : " ,center_Y);

   Cout("RHUD값 : ",R_H_UD);

   if(center_X == 0 || center_Y == 0){
      if(nothing == 0){
         Motion2(W_forward_slow_2);
         nothing = 1;
      }
      else{
         Motion2(W_backward_good_1);
         nothing = 0;
      }
      INDEX = 2;
      return INDEX;
   }

   if(R_H_UD == H_down_80){
      Motion2(H_down_60);
      R_H_UD = H_down_60;
   }

   if(center_X == 0 && where_is_the_ball <= 3 && R_H_UD != H_down_30){
      if(dir == 0)
         Motion2(W_left_m);
      else if(dir == 1)
         Motion2(W_right_m);
      where_is_the_ball++;
      return INDEX;
   }
   if(center_Y == 0 && where_is_the_ball == 4){
      Motion2(W_forward_slow_2);
      where_is_the_ball = 0;
      walk_count++;
      return INDEX;
   }
   if(center_Y == 0 && R_H_UD == H_down_30){
      Motion2(W_forward_slow_2);
      walk_count++;
      return INDEX;
   }

   for(int i = 0; i < 2; i++){
      if (center_X < 40 && center_X != 0 && R_H_UD == H_down_60)   Motion2(W_left_m);
      else if (center_X > 280 && center_X != 0 && R_H_UD == H_down_60) Motion2(W_right_m);
      else if (center_X < 80 && center_X != 0 && R_H_UD == H_50)   Motion2(W_left_s);
      else if (center_X > 240 && center_X != 0 && R_H_UD == H_50)   Motion2(W_right_s);
      else if (center_X < 148 && center_X != 0) Motion2(W_left_s);
      else if (center_X > 170 && center_X != 0) Motion2(W_right_s);
      else{
         Motion2(H_forward_LR);
         R_H_LR = H_forward_LR;
      }
   }
   if(center_Y < 120 && center_Y != 0 && R_H_UD == H_down_60) {
      Motion2(W_forward_slow_2);
      Motion2(W_forward_slow_2);
      Motion2(W_forward_slow_2);
      walk_count++;
   }
   else if(center_Y < 140 && center_Y != 0 && R_H_UD == H_50){
      Motion2(W_forward_slow_2);
      Motion2(W_forward_slow_2);
      walk_count++;
   }
   else if(center_Y < 170 && center_Y != 0 && R_H_UD == H_50) {
      usleep(500000);
      Motion2(W_forward_slow_2);
      walk_count++;
   }
   else if(center_Y > 230 && center_Y != 0 && R_H_UD == H_50) {
      Motion2(W_back_good);
      walk_count--;
   }
   else if(R_H_UD == H_down_30 && center_Y < 110) {
      Motion2(W_forward_slow_good);
   }
   else if(R_H_UD == H_down_60) {
      Motion2(H_50);
      Motion2(W_forward_slow_2);
      R_H_UD = H_50;
   }
   else if(R_H_UD == H_50){
      Motion2(H_down_30);
      Motion2(W_forward_slow_good);
      R_H_UD = H_down_30;
   }
   else if(R_H_UD == H_down_30){
      if(LR_usleep){
         Motion2(H_left_90);
         Motion2(H_forward_UD);
         R_H_UD = H_forward_UD;
         R_H_LR = H_left_90;
         LR_usleep = false;
         return 23;
      }else if(!LR_usleep){
      LR_usleep = true;
      }

   }

   dir = 0;
   return INDEX;
}

int before_hit() {
   usleep(500000);
   getFrame(C_GREEN,0);
   cv::Mat g_image = FRAME;

   std::vector<cv::Point2f> approx = DrawShapeDetection_Control(g_image, &center_X, &center_Y);
   Get_approx_shape(approx);

   if(green_approx_size < 200){
      goal_idx = INDEX;
      return 119;
   }
   green_approx_size = 0;
   center_X = (G_RIGHT + G_LEFT)/2;

   Imshow("before hit", g_image);

   if(center_X > 140 && center_X < 180){
      return ++INDEX;
   }
   else if (center_X > 240 && center_X != 319) { Motion2(T_right_large); return INDEX; }
   else if (center_X > 180 && center_X != 319){ Motion2(T_right_20); }
   else if (center_X < 80 && center_X != 0) { Motion2(T_left_large); return INDEX; }
   else if (center_X < 140 && center_X != 0){ Motion2(T_left_20); }
   else {
      goal_idx = INDEX;
      return 119;
   }
   return INDEX;
}


int blue_puck_count = 0;
int find_puck_back_cnt=0;
int find_puck() {
   if(find_puck_back_cnt > 6) {find_puck_back_cnt=0; return 2;}
   usleep(500000);
   getFrame(C_BLUE,0);
   cv::Mat b_image = FRAME;

   DrawShapeDetection_Control(b_image, &center_X, &center_Y);
   Cout("cnt",find_puck_back_cnt);
   if(center_X == 0){
      if(blue_puck_count > 1500){
         Cout("?");
         Motion2(W_left_s);
         Motion2(W_backward_good_1);
         Motion2(W_backward_good_1);
      }
      else{
         find_puck_back_cnt++;
         Motion2(W_backward_good_1);
         Motion2(W_backward_good_1);
      }
      return 24;
   }
   find_puck_back_cnt =0;

   if (center_X < 200 && center_X != 0 ){
      if(center_Y>130  && center_Y!=0) Motion2(W_back_good);
      Motion2(W_left_s);
      return INDEX;
   }
   else if (center_X > 240 && center_X != 0){
      if(center_Y>130  && center_Y!=0) Motion2(W_back_good);
      Motion2(W_right_s);
      return INDEX;
   }

   usleep(500000);
   getFrame(C_BLUE, 0);
   b_image = FRAME;

   DrawShapeDetection_Control(b_image, &center_X, &center_Y);

   if ( center_Y < 95 && center_Y != 0) { Motion2(W_forward_slow_good); }
   else if( center_Y > 130 &&center_Y != 0) { Motion2(W_back_good); }
   else return 26;

   return INDEX;

}

int find_goal_test(){
   int F_90_green_count = green_blue(H_left_90, 0);

   int F_30_green_count = green_blue(H_left_30, 0);

   Cout("90_Green : ", F_90_green_count," 30_Green : ",F_30_green_count);

   if(F_90_green_count + F_30_green_count < 500){
      int F_0_green_count = green_blue(H_forward_LR, 0);
      if(F_0_green_count > 1000){
         Motion2(W_backward_good_1);
         usleep(500000);
         Motion2(T_right_large);
      }else{
         Motion2(W_backward_good_1);
         usleep(500000);
         Motion2(T_left_large);
      }
      F_0_green_count = 0;
   }
   else if(F_90_green_count > F_30_green_count){
      Cout("left");

      Motion2(W_backward_good_1);
      usleep(500000);
      Motion2(T_left_large);
   }
   else if(F_90_green_count < F_30_green_count){

      Cout("right");
      Motion2(W_backward_good_1);
      usleep(500000);
      Motion2(T_right_large);
   }
   else{
      Cout("else");
      Motion2(W_backward_good_1);
      usleep(500000);
      Motion2(T_left_large);
   }

   F_90_green_count = 0;
   F_30_green_count = 0;
   Motion2(H_left_90);
   R_H_LR = H_left_90;
   usleep(500000);
   return goal_idx;
}


int green_blue(int head_position, int gorb){
   Motion2(head_position);

   usleep(500000);

   if(gorb == 0){
      getFrame(C_GREEN,0);
      int green_goal_count = 0;

      cv::Mat g_image = FRAME;

      for (int y = 0; y< g_image.rows; y++) {
         for (int x = 0; x < g_image.cols; x++) {
            if (COLOR(g_image, V_GREEN, y, x)) {
               green_goal_count++;
            }
         }
      }
      return green_goal_count;
   }else{
      getFrame(C_BLUE,0);

      cv::Mat b_image = FRAME;

      for (int y = 0; y< b_image.rows; y++) {
         for (int x = 0; x < b_image.cols; x++) {
            if (COLOR(b_image, V_BLUE, y, x)) {
               blue_puck_count++;
            }
         }
      }//y
   }
   return 0;
}

int find_line(int INDEX) {
   int cnt_YELLOW = 0;

   getFrame(C_YELLOW,0);
   cv::Mat temp_image = FRAME;
   int start_x = 0, start_y = 0;
   int finish_x = temp_image.cols, finish_y = temp_image.rows;

   for (int y = 60; y < 180; y++) {
      for (int x = 120; x < 200; x++) {
         if (COLOR(temp_image, V_YELLOW, y, x)) {
            cnt_YELLOW++;
         }
      }
   }
   Imshow("find_line",temp_image);

   Cout("------------------COUNTYELLOW",cnt_YELLOW,"  index::   ",INDEX);
   if (R_H_UD == H_down_30){
      if (cnt_YELLOW > 300){
         if(R_H_LR == H_left_45){
            Motion2(T_right_middle);
         }
         else if(R_H_LR == H_left_90){
            Motion2(T_left_large);
         }
         else if(R_H_LR == H_right_45){
            Motion2(T_left_middle);
         }
         else if(R_H_LR == H_right_90){
            Motion2(T_left_large);
         }
         return 6;
      }
      else
         return INDEX;
   }
   else if(R_H_UD == H_down_60){
      if (cnt_YELLOW > 50){
         if(R_H_LR == H_left_45){
            Motion2(T_right_middle);
         }
         else if(R_H_LR == H_left_90){
            Motion2(T_left_large);

         }
         else if(R_H_LR == H_right_45){
            Motion2(T_left_middle);
         }
         else if(R_H_LR == H_right_90){
            Motion2(T_left_large);
         }
         return 6;
      }
      else
         return INDEX;
   }
   else if(R_H_UD==H_forward_UD){
      if (cnt_YELLOW > 50){
         if(R_H_LR == H_left_45){
            Motion2(T_right_middle);
         }
         else if(R_H_LR == H_left_90){
            Motion2(T_left_large);

         }
         else if(R_H_LR == H_right_45){
            Motion2(T_left_middle);
         }
         else if(R_H_LR == H_right_90){
            Motion2(T_left_large);

         }
         return 6;
      }
      else
         return INDEX;
   }
   return INDEX;
}

int get_distance_line_puck(cv::Mat& image,cv::Mat& blue_image){
  int x,y, Y_1,Y_2, distance=-1,Y=-10000;

  DrawShapeDetection_Control(blue_image, &center_X, &center_Y);

   getFrame(C_YELLOW,0);
    cv::Mat b_image = FRAME;
     Imshow("line",b_image);

   Y_1=center_Y-20;
     Y_2=center_Y+20;

   if(Y_1<=0) Y_1=1;
    if(Y_2>=240) Y_2=239;

   Cout("",Y_1,"/",Y_2);
    for (y = Y_1; y < Y_2; y++)
     {
       bool find_line = false;
          for(x = 1; x<319 ; x++){
             if( (COLOR(b_image, V_LINE, y-1,x-1) ? 1 : 0) + (COLOR(b_image, V_LINE, y-1,x) ? 1 : 0)
            + (COLOR(b_image, V_LINE, y-1,x+1) ? 1 : 0) + (COLOR(b_image, V_LINE, y,x-1) ? 1 : 0 )
            + (COLOR(b_image, V_LINE, y,x) ? 1 : 0) + (COLOR(b_image, V_LINE, y,x+1) ? 1 : 0)
            + (COLOR(b_image, V_LINE, y+1,x-1) ? 1 : 0 )+ (COLOR(b_image, V_LINE, y+1,x) ? 1 : 0 )+ (COLOR(b_image, V_LINE, y+1,x+1) ? 1 : 0 ) > 3 )
            {     int temp = x-center_X;
               if(abs(center_Y-Y)>abs(center_Y-y)) {Y = y; distance=temp;}
           }
      }
   }

   Cout("DISTANCE:",distance);
   return distance;

}


void PreProcess(cv::Mat &frame) {
   //스레드로 돌고있는 영상중 프레임을 하나 가져옴
   currentFrame2.copyTo(frame);
   cv::resize(frame, frame, cv::Size(320, 240), 0, 0, CV_INTER_NN);
}

void BinaryColors(cv::Mat &frame, int color1,int color2, int num) {
   int cnt_BLACK = 0;
   // hsv로 바꾸고
   cv::Mat Temp, temp, temp2, F_red, F_black, F_blue, F_green, F_yellow, F_white, sum, x;
   cv::Mat a, b;
   if(color2==0)num=1;
   else num =2;
   int color[2];
   color[0]=color1;
   color[1]=color2;
   cv::Mat small(2, 2, CV_8U, cv::Scalar(1));
   cv::Mat element(4, 4, CV_8U, cv::Scalar(1));

   cvtColor(frame, Temp, CV_BGR2HSV);
   cvtColor(frame, x, CV_BGR2HSV);

   for (int i = 0; i<num; i++) {
      switch (color[i]) {
       case 0: break;
      case 1: // BLUE
         cvtColor(frame, Temp, CV_BGR2HSV);
         inRange(Temp, cv::Scalar(Blue_H_1 ,Blue_S_1, Blue_V_1), cv::Scalar(Blue_H_2, Blue_S_2, Blue_V_2), temp);

         a = x;
         a.setTo(0);
         a.setTo(V_BLUE, temp);
         F_blue = a;
         cv::erode(F_blue, F_blue, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
         cv::dilate(F_blue, F_blue, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));


         cv::dilate(F_blue, F_blue, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
         cv::erode(F_blue, F_blue, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));

         sum = F_blue;
         break;

      case 2: //GREEN
         cvtColor(frame, Temp, CV_BGR2HSV);
         inRange(Temp, cv::Scalar(Green_H_1 ,Green_S_1, Green_V_1), cv::Scalar(Green_H_2, Green_S_2, Green_V_2), temp);
         //inRange(Temp, cv::Scalar(150, 140, 0), cv::Scalar(190, 180, 20), temp);
         a = x;
         a.setTo(0);
         a.setTo(V_GREEN, temp);
         F_green = a;
         cv::erode(F_green, F_green, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
         cv::dilate(F_green, F_green, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));


         cv::dilate(F_green, F_green, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
         cv::erode(F_green, F_green, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));

         sum = F_green;
         break;

      case 3: //YELLOW
      inRange(Temp, cv::Scalar(Yellow_H_1 ,Yellow_S_1, Yellow_V_1), cv::Scalar(Yellow_H_2, Yellow_S_2, Yellow_V_2), temp);
         a = x;
         a.setTo(0);
         a.setTo(V_YELLOW, temp);

         F_yellow = a;

         cv::erode(F_yellow, F_yellow, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
         cv::dilate(F_yellow, F_yellow, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));


         cv::dilate(F_yellow, F_yellow, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
         cv::erode(F_yellow, F_yellow, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));

         sum = F_yellow;
         break;

      }//switch

      if (i == 0) Binaryframe = sum;
      else Binaryframe = sum | Binaryframe;

   }//for
   for (int y = 0; y < Binaryframe.rows; y++) {
      for (int x = 0; x < Binaryframe.cols; x++) {
         if (y == 0 || y == Binaryframe.rows - 1 || x == 0 || x == Binaryframe.cols - 1
            || y == 1 || y == Binaryframe.rows - 2 || x == 1 || x == Binaryframe.cols - 2) {
            SetColor(1, Binaryframe, y, x, 0, 0, 0);
         }
      }
   }
   cv::resize(Binaryframe, small_Binaryframe, cv::Size(320, 240), 0, 0, CV_INTER_NN);
}


void draw_garo(cv::Mat& image, int y){
  for(int i = 0; i < 320; i++){
    SetColor(1,image,y,i,255,0,255);}
}

void draw_sero(cv::Mat& image, int x){
  for(int i = 0; i < 240; i++){
    SetColor(1,image,i,x,255,0,255);}
}

void draw_rectangle(cv::Mat& image, int a,int b,int c, int d){
   if(a < 0) a = 0;
   if(b > 240) b = 240;
   if(c < 0) c = 0;
   if(d > 320) d = 320;

   for(int j = a; j < b; j++){
      for(int i = c; i < d; i++){
         SetColor(1,image,j,i,0,0,0);
      }
    }
}


bool MASK(Mat image, Vec3b color, int y, int x, int size ,int num){
   int y_cnt=0;

   for(int j=0;j<240;j++){
      if (j < 0)j =0;
      else if (j==240) break;
      for(int i=0;i<320;i++){
         if( i<0) i=0;
         else if (i==320){ j =240; i=320;}
         if(COLOR(image, color, y, x)) y_cnt++;
      }
   }
   if(y_cnt>num) return true;
   else return false;
}

void Get_approx_shape(std::vector<cv::Point2f> approx){
   G_UP = 999, G_DOWN=-1, G_RIGHT = -1, G_LEFT = 999;
   for(int i = 0; i < approx.size(); i++){
      if(approx[i].y < G_UP)
      {
         G_UP = approx[i].y;
      }
      if(approx[i].y > G_DOWN)
      {
         G_DOWN = approx[i].y;
      }
      if(approx[i].x > G_RIGHT)
      {
         G_RIGHT = approx[i].x;
      }
      if(approx[i].x < G_LEFT)
      {
         G_LEFT = approx[i].x;
      }
   }

   if(G_UP < 0)
      G_UP = 0;
   else if(G_UP >= 240)
      G_UP = 239;

   if(G_DOWN < 0)
      G_DOWN = 0;
   else if(G_DOWN >= 240)
      G_DOWN = 239;

   if(G_RIGHT < 0)
      G_RIGHT = 0;
   else if(G_RIGHT >= 320)
      G_RIGHT = 319;

   if(G_LEFT < 0)
      G_LEFT = 0;
   else if(G_LEFT >= 320)
      G_LEFT = 319;

   if(approx.size()==0) G_RIGHT = -100;
}


int GetAngleABC(cv::Point a, cv::Point b, cv::Point c)
{
   cv::Point ab(b.x - a.x, b.y - a.y);
   cv::Point cb(b.x - c.x, b.y - c.y);

   float dot = (ab.x * cb.x + ab.y * cb.y); // dot product
   float cross = (ab.x * cb.y - ab.y * cb.x); // cross product

   float alpha = atan2(cross, dot);

   return (int)floor(alpha * 180.0 / CV_PI + 0.5);
}

void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G, int* R)
{
   uchar *data = image.data;
   uchar *blue, *green, *red;

   int nRow = image.rows;
   int nCol = image.cols;

   if (i > nCol - 1 || j > nRow - 1) {
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

   if (i > nCol - 1 || j > nRow - 1) {return;}

   blue = image.data + j*image.step + i*image.elemSize() + 0;
   if (is_color) {
      green = image.data + j*image.step + i*image.elemSize() + 1;
      red = image.data + j*image.step + i*image.elemSize() + 2;
   }
   *blue = B;
   if (is_color) { *green = G;   *red = R; }
}

bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x) {
   int R, G, B;
   GetColor(1, image, y, x, &B, &G, &R);
   return (B == color[0] && G == color[1] && R == color[2]);
}

std::vector<cv::Point2f> DrawShapeDetection_Control(cv::Mat &m_image, int* x, int* y)
{
   cv::Mat img_result, img_gray = m_image, a = m_image;

   int X = 0, Y = 0, IDX;

   //그레이스케일 이미지로 변환
   if (!m_image.empty()) {
      cvtColor(m_image, img_gray, CV_RGB2GRAY);
   }

   //이진화 이미지로 변환
   cv::Mat binary_image;
   if (!m_image.empty()) {
      threshold(img_gray, img_gray, 180, 200, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
   }
   //contour를 찾는다.
   std::vector<std::vector<cv::Point> > contours;
   findContours(img_gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

   //contour를 근사화한다.(윤곽선)
   std::vector<cv::Point2f> approx;

   img_result = m_image.clone();

   int MAX_size = -1;
   for (size_t i = 0; i < contours.size(); i++)
   {
      //가장 바깥에 있는 외곽선들 좌표
      cv::Point first(1, 1),second(1, 238),third(318, 238),fourth(318, 1);

      if ((contours[i][0] != first) && (contours[i][1] != second) && (contours[i][2] != third) && (contours[i][3] != fourth)) //가장 바깥에 있는 외곽선 안잡게
         cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
      else
         continue;

      //모든 코너의 각도를 구해서 더한다.
      int ang_sum = 0;
      int size = approx.size();

      for (int k = 0; k < size; k++) {
         int ang = GetAngleABC(approx[k], approx[(k + 1) % size], approx[(k + 2) % size]);
         ang_sum += abs(ang);
      }
      int ang_threshold = 8;
      int ang_sum_min = (180 - ang_threshold) * (size - 2);
      int ang_sum_max = (180 + ang_threshold) * (size - 2);
      ang_sum = abs(ang_sum);
      if (1)
      {
         if (fabs(cv::contourArea(cv::Mat(approx))) > 100) { //200
            FIND= 1;
            cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);

            if (current_color!=C_BLUE&&MAX_size<fabs(cv::contourArea(cv::Mat(approx)))) {
               MAX_size = fabs(cv::contourArea(cv::Mat(approx)));
               IDX = i;
               for (int z = 0; z < approx.size(); z++)
               {
                  X += approx[z].x;
                  Y += approx[z].y;
               }
               X /= approx.size();
               Y /= approx.size();

            }
            else if(current_color == C_BLUE && approx[0].y > MAX_size){
               MAX_size = approx[0].y;
               IDX = i;
               for( int z =0;z<approx.size(); z++){
                  X+= approx[z].x;
                  Y += approx[z].y;
               }
               X /= approx.size();
               Y /= approx.size();
            }
         }
      }
      green_approx_size = MAX_size;
   }
   if (FIND) {
      FIND = 0;
      Cout("APPROX find");
      cv::approxPolyDP(cv::Mat(contours[IDX]), approx, arcLength(cv::Mat(contours[IDX]), true)*0.02, true);

      draw_approx(img_result, approx);

      if (!Frame.empty())
      {
         Imshow("Shape Detection", img_result);
      }
      *x = X;
      *y = Y;
      return approx;
   }
   *x = 0; *y=0;
   return approx;
}

void draw_approx(cv::Mat& image, std::vector<cv::Point2f> approx) {

   int size = approx.size();

   //Contour를 근사화한 직선을 그린다.
   if (size % 2 == 0) {
      cv::line(image, approx[0], approx[approx.size() - 1], cv::Scalar(255, 255, 255), 3);

      for (int k = 0; k < size - 1; k++)
         cv::line(image, approx[k], approx[k + 1], cv::Scalar(255, 255, 255), 3);

      for (int k = 0; k < size; k++)
         cv::circle(image, approx[k], 3, cv::Scalar(255, 255, 255));
   }
   else {
      cv::line(image, approx[0], approx[approx.size() - 1], cv::Scalar(255, 255, 255), 3);

      for (int k = 0; k < size - 1; k++)
         cv::line(image, approx[k], approx[k + 1], cv::Scalar(255, 255, 255), 3);

      for (int k = 0; k < size; k++)
         cv::circle(image, approx[k], 3, cv::Scalar(0, 0, 0));
   }
}


void green_cut(Mat image, Mat& blue_image){
   getFrame(C_GREEN,0);
   cv::Mat g_image = FRAME;
   Imshow("dddddd",blue_image);
   std::vector<cv::Point2f> approx = DrawShapeDetection_Control(g_image, &center_X, &center_Y);
   Cout("green_approx",green_approx_size);
   if(center_X !=0 && center_Y !=0 && green_approx_size>5000){
      Get_approx_shape(approx);

      if(G_DOWN >150){
         blue_image.setTo(0);
      }
      else {
         int L=G_LEFT-30,R =G_RIGHT+30;
         if(30<G_LEFT-30) L=30;
         if(290>R) R= 290;
         draw_rectangle(blue_image,0,G_DOWN+90,L,R);

      }
   }
   if(R_H_LR == H_left_45){
      for( int j = 0; j<240; j++){
         bool find_Y=false;
         for(int i = 320; i>0; i--){
            if(COLOR(image,V_YELLOW,j,i)){ find_Y = true;
               if(CUTCUT) {for(int k = i ; k<i+10; k ++) {SetColor(1,blue_image,j,k,0,0,0);}}
            }
            if(find_Y) SetColor(1,blue_image,j,i,0,0,0);
         }
      }
   }
   else if (R_H_LR == H_forward_LR){
      for(int j =0; j<240 ; j++){
         bool find_Y = false;
         for(int i = 160; i<320;i++){
            if(COLOR(image,V_YELLOW,j,i)){ find_Y = true;
            }
            if(find_Y) SetColor(1, blue_image, j,i,0,0,0);
         }
         find_Y =false;
         for(int i = 160; i>0; i--){
            if(COLOR(image,V_YELLOW,j,i)) find_Y = true;
            if(find_Y) SetColor(1,blue_image,j,i,0,0,0);
         }
      }
   }
   else if (R_H_LR == H_left_90){
      for(int i=0;i<160;i++){
         bool find_Y = false;
         for(int j =240;j>0;j--){
            if(COLOR(image,V_YELLOW,j,i)){ find_Y = true;
               if(CUTCUT){ for(int k = j; k<j+10;k++) {SetColor(1,blue_image,k,i,0,0,0);}  }
            }
            if(find_Y) SetColor(1,blue_image,j,i,0,0,0);
         }
      }
   }
   Imshow("GREENCUT",blue_image);

}


void cut(Mat& image, Mat& blue_image, int P1, int P2){
   int state;

   int down_before[6][2]={{0,3},{0,7},{3,5},{3,4}, {4,7}, {4,5}}; // 위에서 내려오며 만날때까지 지운다.
   int up_after[4][2] = {{1,3}, {1,7}, {2,3}, {2,7} }; // 아래에서 위로 가며 만난뒤로 지운다.
   int left_after[8][2] = {{0,1},{0,2},{0,5},{1,5},{1,4} , {2,5} , {2,4},{5,7}}; //왼쪽으로 진행하며 만난 뒤로 지운다,
   int left_before[4][2] = {{0,6}, {1,6},{2,6},{6,4}} ;// 왼쪽으로 진행하며 만날때까지 지운다
   int right_after[2][2] = {{3,6},{7,6}}; // 오른쪽으로 진행하며 만난뒤로 지운다.

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

   switch(state){
      case 0:      //         cout<<"//위에서 내려오며 만날때까지 지운다."<<endl;
         for(int i = 0; i<320; i++){
            for(int j =0; j<240; j++){
               if(COLOR(image,V_PURPLE, j, i)) j=240;
               SetColor(1,blue_image,j,i,0,0,0);
            }
         }
      break;
      case 1:      //         cout<<"// 아래에서 위로 가며 만난뒤로 지운다."<<endl;
         for(int i = 0; i<320; i++){
            bool find = false;
            for(int j =240; j>0; j--){
               if(COLOR(image,V_PURPLE, j, i)) find = true;
               if(find) SetColor(1,blue_image,j,i,0,0,0);
            }
         }
      break;
      case 2:      //         cout<<"//왼쪽으로 진행하며 만난 뒤로 지운다,"<<endl;
         for(int j =0; j<240; j++){
            bool find = false;
            for(int i = 320 ; i>0; i--){
               if(COLOR(image,V_PURPLE,j, i)) find = true;
               if(find) SetColor(1,blue_image,j,i,0,0,0);
            }
         }
      break;
      case 3:      //         cout<<"// 왼쪽으로 진행하며 만날때까지 지운다"<<endl;
         for(int j =0; j<240; j++){
            for(int i = 320 ; i>0; i--){
               if(COLOR(image,V_PURPLE, j, i)) i=0;
               SetColor(1,blue_image,j,i,0,0,0);
            }
         }
      break;
      case 4:      //         cout<<"// 오른쪽으로 진행하며 만난뒤로 지운다."<<endl;
         for(int j =0; j<240; j++){
            bool find = false;
            for(int i = 0; i<320; i++){
               if(COLOR(image,V_PURPLE, j, i)) find = true;
               if(find) SetColor(1,blue_image,j,i,0,0,0);
            }
         }
      break;
   }

}

bool yellow_cut(Mat inImage , Mat& blue_image, bool is_cut){
   Mat image = inImage.clone();
   cvtColor(image, image, CV_RGB2GRAY);
   Canny(image,image,30,90,3);
   vector<Vec4i> lines;
   HoughLinesP(image, lines, 1, CV_PI /100,30,30,3);

   int CENTER_Y_GARO;
   float MAX_LEN=0.0,MAX_LEN_GARO=0.0 , MAX_DEGREE=-360,MAX_DEGREE_GARO=-360,degree[lines.size()];
   Vec4i LINE, LINE_GARO,LINE2;
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

         if(GARO){
            if(len>MAX_LEN_GARO && ((degree[i]>0 && degree[i]<45 )|| (degree[i]>135 && degree[i]<180))) {
               MAX_LEN_GARO = len;
               MAX_DEGREE_GARO = degree[i];
               LINE_GARO = lines[i];
               CENTER_Y_GARO = (p2.y+p1.y)/2;
            }
         }
         if(len>MAX_LEN) {
            MAX_LEN = len;
            MAX_DEGREE = degree[i];
            LINE = lines[i];
            CENTER_Y = (p2.y+p1.y)/2;
         }
         if(MAX_LEN_GARO>5 && GARO){
            Cout("LEN",(int)MAX_LEN_GARO,"DEGREE",(int)MAX_DEGREE_GARO);
            MAX_LEN = MAX_LEN_GARO;
            MAX_DEGREE = MAX_DEGREE_GARO;
            LINE = LINE_GARO;
            CENTER_Y = CENTER_Y_GARO;
          }
      }
      Imshow("all",inImage);

      FIND_LINE = 1;

      Cout("DEGREE",MAX_DEGREE);
      Point p1 = Point(LINE[0],LINE[1]);
      Point p2 = Point(LINE[2],LINE[3]);
      draw_LINE(inImage, p1, p2);
      P_START = start; P_END = end;
      if(is_cut){
         cut(inImage,blue_image,START,END);
      }
      DEGREE = MAX_DEGREE;

      Imshow("cut",blue_image);
      GARO=false;
      return true;
   }
   else {
      FIND_LINE = 0;
      GARO = false;
      return false;
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

   line(inImage,start,end,Scalar(255,0,255),3);
   Imshow("line",inImage);

   if(start.x == 0){
      if(start.y <120) START = 0;
      else           START = 4 ;
   }
   else if ( start.x < 160){
      if(start.y <120) START = 1;
      else           START = 5 ;
   }
   else if (start.x<320){
      if(start.y < 120) START = 2;
      else           START = 6;
   }
   else if (start.x ==320){
      if(start.y < 120) START = 3;
      else           START = 7;
   }

   if(end.x == 0){
      if(end.y <120)    END = 0;
      else          END = 4 ;
   }
   else if ( end.x < 160){
      if(end.y <120)    END = 1;
      else          END = 5 ;
   }
   else if (end.x<320){
      if(end.y < 120) END = 2;
      else         END = 6;
   }
   else if (end.x ==320){
      if(end.y < 120) END = 3;
      else         END = 7;
   }
}
void SCAN(){
   int COLOR_NUM =0, H1,S1,V1,H2,S2,V2;
   Cout("B:1 / G:2 / Y:3");
   cin>> COLOR_NUM;
   cin >>H1>>S1>>V1>>H2>>S2>>V2;
   switch(COLOR_NUM){
      case C_BLUE:
         Blue_H_1 = H1;  Blue_S_1 = S1;  Blue_V_1 = V1;
         Blue_H_2 = H2;  Blue_S_2 = S2;  Blue_V_2 = V2;
      break;
      case C_YELLOW:
         Yellow_H_1 = H1;  Yellow_S_1 = S1;  Yellow_V_1 = V1;
         Yellow_H_2 = H2;  Yellow_S_2 = S2;  Yellow_V_2 = V2;
      break;
      case C_GREEN:
         Green_H_1 = H1;  Green_S_1 = S1;  Green_V_1 = V1;
         Green_H_2 = H2;  Green_S_2 = S2;  Green_V_2 = V2;
      break;
   }

}

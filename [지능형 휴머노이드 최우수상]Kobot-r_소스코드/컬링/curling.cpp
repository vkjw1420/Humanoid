#include "curling.h"
#include "RobotProtocol.h"

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

cv::Mat Frame;
cv::Mat currentFrame, currentFrame2;

int robot_head_state = BASE;
int robot_body_state = BASE;
bool going_puck = true;

cv::Point Puck_center, Goal_center;
int radius, Goal_radius;

/////////////////////////////////////////////////
pthread_mutex_t frameLocker; //뮤텍스 잠금
pthread_t updateThread; //thread ID는 updateThread
cv::VideoCapture capture;

void *updateFrame(void* arg)
{
	while(1)
	{
		cv::Mat tempframe;
		capture >> tempframe;
		pthread_mutex_lock(&frameLocker);
		Frame = tempframe;
		pthread_mutex_unlock(&frameLocker);
	}
	pthread_exit((void *)0);
}

//영상을 받고 이미지 다운 샘플링
void PreProcess(cv::Mat &frame)
{
	//스레드로 돌고있는 영상중 프레임을 하나 가져옴
	currentFrame2.copyTo(frame);
	cv::resize(frame, frame, cv::Size(320,240), 0,0, CV_INTER_NN);
}
////////////////////////////////////////////////////

void Motion2_test(int motion)
{
	Motion2(motion);

	switch(motion)
	{
		case WALK_FORWARD:
			usleep(1000000);
			break;
		case TURN_RIGHT:
			usleep(2000000);
			break;
		case TURN_LEFT:
			usleep(2000000);
			break;
//----------------------------------------------------------------------------------------
		case TURN_LEFT_SMALL:
			usleep(700000);//1000000
			break;
		case TURN_RIGHT_SMALL:
			usleep(700000);//1000000
			break;

		case WALK_LEFT_SMALL:
			usleep(1500000);//2500000
			break;
		case WALK_RIGHT_SMALL:
			usleep(1500000);//2500000
			break;

		case WALK_FORWARD_ONE:
			if(going_puck == false)
				usleep(1000000);//3000000
			break;
		case WALK_FORWARD_ONE_SMALL:
			usleep(800000);
			break;
		case SIT:
			usleep(1000000);
			break;

		case HEAD_DOWN_30:
			if(robot_head_state != HEAD_DOWN_30)
			{
				robot_head_state = HEAD_DOWN_30;
				usleep(1000000);
			}
			break;

//----------------------------------------------------------------------------------------

		case HEAD_DOWN:
			if(robot_head_state != HEAD_DOWN)
			{
				robot_head_state = HEAD_DOWN;
				//Motion2(HEAD_FORWARD);
				usleep(800000);//800000
			}
			break;
		case HEAD_DOWN_LARGE:
			if(robot_head_state != HEAD_DOWN_LARGE)
			{
				robot_head_state = HEAD_DOWN_LARGE;
				usleep(800000);
			}
			break;
		case HEAD_UP:
			if(robot_head_state != HEAD_UP)
			{
				robot_head_state = HEAD_UP;
				usleep(3000000);//2500000
			}
			break;
		case HEAD_LEFT_60:
			if(robot_head_state != HEAD_LEFT_60)
			{
				robot_head_state = HEAD_LEFT_60;
				usleep(1000000);
			}
		case HEAD_RIGHT_60:
			if(robot_head_state != HEAD_RIGHT_60)
			{
				robot_head_state = HEAD_RIGHT_60;
				usleep(1000000);
			}
			break;
		case HEAD_FORWARD:
			if(robot_head_state != HEAD_FORWARD && robot_head_state != HEAD_DOWN)
			{
				robot_head_state = HEAD_FORWARD;
				usleep(1000000);
			}
			break;

		case WALK_LEFT:
			usleep(2000000);
			break;
		case WALK_RIGHT:
			usleep(2000000);
			break;
		case SHOOT_PUCK:
		case SHOOT_PUCK10:
		case SHOOT_PUCK11:
		case SHOOT_PUCK12:
		case SHOOT_PUCK13:
		case SHOOT_PUCK14:
		case SHOOT_PUCK15:
			usleep(1000000);
			break;

		case BASE:
			if(robot_body_state != BASE)
			{
				robot_body_state = BASE;
				usleep(1000000);
			}
			break;
	}
}

bool Detect_Puck_in_goal()
{
	cout<<"I'm watching"<<endl;
        cv::Mat src_HSV;
        cv::Mat tmp1,tmp2;
        cv::Mat src_binary;

        std::vector< cv::Vec3f > circles;

        pthread_mutex_lock(&frameLocker);
        currentFrame = Frame;
        pthread_mutex_unlock(&frameLocker);

        cv::resize(currentFrame, currentFrame, cv::Size(320,240),0,0,CV_INTER_NN);

        cvtColor(currentFrame, src_HSV, CV_BGR2HSV);

        int LowH = 90, LowS = 120, LowV = 65, HighH = 140, HighS = 255, HighV = 255;//파란색
        int LowH2 = 90, LowS2 = 80, LowV2 = 40, HighH2 = 140, HighS2 = 255, HighV2 = 255;//보라색

        inRange(src_HSV, cv::Scalar(LowH,LowS,LowV), cv::Scalar(HighH,HighS,HighV),tmp1);
        inRange(src_HSV, cv::Scalar(LowH2,LowS2,LowV2), cv::Scalar(HighH2,HighS2,HighV2),tmp2);

        bitwise_or(tmp1,tmp2,src_binary);

        erode(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(3,3)) );//5 5
        dilate(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );//5 5

        dilate(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );
        erode(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );

        //-------------------------------------------------------------------

        RNG rng(12345);

        Mat threshold_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        int thresh = 100;
        threshold(src_binary,threshold_output,thresh,255,THRESH_BINARY);
        findContours(threshold_output,contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        vector<vector<Point> > contours_poly(contours.size() );
        vector<Rect> boundRect ( contours.size() );
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );

        for( int i = 0; i < contours.size(); i++ )
        {
                approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
                boundRect[i] = boundingRect(Mat(contours_poly[i]) );
                minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        }

        Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++)
        {
                cv::Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color,2,8,0);
                circle(drawing, center[i], (int)radius[i],color,2,8,0);
        }
//      imshow("puck",drawing);

//	imshow("goal_binary_ball",src_binary);
        if(cv::waitKey(30) == 27)
                cout<<"aa"<<endl;

	int lower_pos = 160;
	int middle_diff = 320;

	bool find = false;
	int ball_count=0;
        if(contours.size())
        {

		Goal_center.x = 0;
		Goal_center.y = 0;

                for( int i = 0; i < contours.size(); i++)
                {
			if(lower_pos < center[i].y)
			{
				Goal_center.x += center[i].x;
				Goal_center.y += center[i].y;
				cout<<center[i]<<endl;
				ball_count++;
			}
                }

		if(ball_count!=0)
		{
			find = true;
			Goal_center.x /= ball_count;
			Goal_center.y /= ball_count;
		}


		usleep(1000000);
                return find;
        }
        else
                return false;

}


bool Detect_goal(int motion)
{
	cv::Mat src_HSV;
	cv::Mat src_binary;
	cv::Mat tmp1, tmp2;

	Motion2_test(motion);

	std::vector<cv::Vec3f> circles;

	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;
	pthread_mutex_unlock(&frameLocker);

	cv::resize(currentFrame, currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

//	imshow("cur",currentFrame);

	cvtColor(currentFrame, src_HSV, CV_BGR2HSV);

//	int LowH = 0, LowS = 30, LowV = 100, HighH = 20 , HighS = 190, HighV = 190;//빨간색 동아리방
//	int LowH2 = 170, LowS2 = 30, LowV2 = 110, HighH2 = 179, HighS2 = 190, HighV2 = 190;

	int LowH = 0, LowS = 40, LowV = 40, HighH = 20, HighS = 200, HighV = 200;//빨간색 임소경 0 30 20 20 200 200
	int LowH2 = 170, LowS2 = 40, LowV2 = 40, HighH2 = 179, HighS2 = 200, HighV2 = 200;//170 30 30 179 200 200

//	int LowH = 20, LowS = 190, LowV = 190, HighH = 20 , HighS = 190, HighV = 190;//빨간색 테스트
//	int LowH2 = 179, LowS2 = 190, LowV2 = 190, HighH2 = 179, HighS2 = 190, HighV2 = 190;

	inRange(src_HSV, cv::Scalar(LowH,LowS,LowV), cv::Scalar(HighH,HighS,HighV), tmp1);
	inRange(src_HSV, cv::Scalar(LowH2,LowS2,LowV2), cv::Scalar(HighH2,HighS2,HighV2), tmp2);

	bitwise_or(tmp1, tmp2, src_binary);

	erode(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(3,3)) );//3 3
	dilate(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );

	dilate(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );
	erode(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(3,3)) );

//-------------------------------------------------------------------

	RNG rng(12345);

	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	int thresh = 100;
	threshold(src_binary,threshold_output,thresh,255,THRESH_BINARY);
	findContours(threshold_output,contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<vector<Point> > contours_poly(contours.size() );
	vector<Rect> boundRect ( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );

	for( int i = 0; i < contours.size(); i++ )
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]) );
		minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
	}

	Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++)
	{
		cv::Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color,2,8,0);
		circle(drawing, center[i], (int)radius[i],color,2,8,0);
	}
//	imshow("goal",drawing);

//	imshow("goal_binary",src_binary);
	if(cv::waitKey(30) == 27)
		cout<<"aa"<<endl;
//------------------------------------------
	int lower_pos = 160;

	bool find = false;
	if(contours.size())
	{
		for( int i = 0; i < contours.size(); i++)
		{
			if(center[i].y > lower_pos)
			{
				Goal_center.x = center[i].x;
				Goal_center.y = center[i].y;
				lower_pos = center[i].y;
				find = true;
			}
		}
		if(lower_pos == 160)
			return Detect_Puck_in_goal();//밑에 있는 빨간색 골 못차음
		return find;
	}
	else
	{
		return Detect_Puck_in_goal();//정말 빨간색이 하나도 없을 떄
//-------------------------------------------
	}

}

bool Detect_Puck(int motion)
{
	cv::Mat src_HSV;
	cv::Mat tmp1,tmp2;
	cv::Mat src_binary;

	Motion2_test(HEAD_FORWARD);
	Motion2_test(motion);

	std::vector< cv::Vec3f > circles;

	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;
	pthread_mutex_unlock(&frameLocker);

	cv::resize(currentFrame, currentFrame, cv::Size(320,240),0,0,CV_INTER_NN);

	cvtColor(currentFrame, src_HSV, CV_BGR2HSV);

	int LowH = 90, LowS = 120, LowV = 65, HighH = 140, HighS = 255, HighV = 255;//파란색
	int LowH2 = 90, LowS2 = 80, LowV2 = 40, HighH2 = 140, HighS2 = 255, HighV2 = 255;//보라색

	inRange(src_HSV, cv::Scalar(LowH,LowS,LowV), cv::Scalar(HighH,HighS,HighV),tmp1);
	inRange(src_HSV, cv::Scalar(LowH2,LowS2,LowV2), cv::Scalar(HighH2,HighS2,HighV2),tmp2);

	bitwise_or(tmp1,tmp2,src_binary);

	erode(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(7,7)) );//5 5
	dilate(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(7,7)) );//5 5

	dilate(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(15,15)) );
	erode(src_binary, src_binary, getStructuringElement(MORPH_ELLIPSE, Size(15,15)) );

	//-------------------------------------------------------------------

	RNG rng(12345);

	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	int thresh = 100;
	threshold(src_binary,threshold_output,thresh,255,THRESH_BINARY);
	findContours(threshold_output,contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<vector<Point> > contours_poly(contours.size() );
	vector<Rect> boundRect ( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );

	for( int i = 0; i < contours.size(); i++ )
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]) );
		minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
	}

	Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++)
	{
		cv::Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color,2,8,0);
		circle(drawing, center[i], (int)radius[i],color,2,8,0);
	}
//	imshow("puck",drawing);

//	imshow("puck_binary",src_binary);
	if(cv::waitKey(30) == 27)
	        cout<<"aa"<<endl;

	int lower_pos = 0;
        if(contours.size())
        {
                for( int i = 0; i < contours.size(); i++)
                {
                        if(center[i].y > lower_pos)
                        {
                                Puck_center.x = center[i].x;
                                Puck_center.y = center[i].y;
                                lower_pos = center[i].y;
                        }
                }
                return true;
        }
        else
                return false;

}

bool Go_front_Puck()
{
	while(true)
	{
		Detect_Puck();

		if(Puck_center.x < 155)//155
			Motion2_test(WALK_LEFT_SMALL);
		else if(Puck_center.x > 175)//175
			Motion2_test(WALK_RIGHT_SMALL);
		else
			break;
	}
}

bool Go_close_Puck(bool isFindPuck)
{
	if(isFindPuck)
	{
		bool isFront = true, isClose = true;
	        if(Puck_center.x < 145)//155
		{
	                Motion2_test(WALK_LEFT_SMALL);//왼쪽 걷기
			isFront = false;
		}
	        else if(Puck_center.x > 160)//165
		{
        	        Motion2_test(WALK_RIGHT_SMALL);//오른쪽 걷기
			isFront = false;
		}

		if(Puck_center.y < 100)//120
		{
			Motion2_test(WALK_FORWARD_ONE);
			isClose = false;
		}
		else if(Puck_center.y >= 100 && Puck_center.y < 128)//110 / 멀리 125 가까이 130 여태까지 본 거 중에서 최적 128
		{
			Motion2_test(WALK_FORWARD_ONE_SMALL);
			Motion2_test(WALK_FORWARD_ONE_SMALL);
			isClose = false;
		}
		if(isFront && isClose)
			return true;
		else
			return false;

	}
}

bool Go_to_Puck(bool isFindPuck)
{
	if(isFindPuck)
	{
		if(Puck_center.x < 140)
		{
			Motion2_test(TURN_LEFT_SMALL);//왼쪽 회전
		}
	    	else if(Puck_center.x > 180)
		{
		        Motion2_test(TURN_RIGHT_SMALL);//오른쪽 회전
		}

	    if(Puck_center.y > 200)
	{
		going_puck = false;
	        return true;//Puck이 가까이 있으니까 고개 숙이고 골이랑 위치 비교
	}
	    else
	        {
	    	Motion2_test(WALK_FORWARD_ONE);//전진
	        return false;//Puck이 아직 멀리 있으니까 계속 찾기
        	}
	}
	else
	{
		Motion2_test(WALK_FORWARD_ONE);
	    return false;
	}
}

void check_last_direction()
{
	Detect_goal();

	if(Goal_center.x < 150)
	{
		Motion2_test(TURN_LEFT_SMALL);
		if(Goal_center.x < 130)
		{
			Motion2_test(TURN_LEFT_SMALL);
			Motion2_test(WALK_RIGHT_SMALL);
		}
//		Go_front_Puck();
		Motion2_test(WALK_RIGHT_SMALL);
	}
	else if(Goal_center.x > 165)//170
	{
		Motion2_test(TURN_RIGHT_SMALL);
		if(Goal_center.x > 190)
		{
			Motion2_test(TURN_RIGHT_SMALL);
			Motion2_test(WALK_LEFT_SMALL);
		}
//		Go_front_Puck();
		Motion2_test(WALK_LEFT_SMALL);
	}
}

void Curling()//main 함수처럼 동작
{
	capture.open(0);

	pthread_mutex_init(&frameLocker, NULL);
	pthread_create(&updateThread, NULL, updateFrame, NULL);

	if( !capture.isOpened() ) {
		std::cerr << "Could not open camera" << std::endl;
		exit(1) ;
	}
	//1)go to ball!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	while(true)
	{
		pthread_mutex_lock(&frameLocker);
		currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
		pthread_mutex_unlock(&frameLocker);

//		Detect_goal();
//		Detect_Puck(HEAD_DOWN);


		if (!currentFrame.empty())
		{
			//1. Find PUCK@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			bool isFindPuck = Detect_Puck();//멀리 있는 Puck 찾기

			bool isClosePuck = Go_to_Puck(isFindPuck);

			if(isClosePuck)//가까이 붙었음 / 고개 숙이고 Puck이랑 골 비교하기 시작
			{
				std::cout<<"close!!"<<std::endl;
				break;
			}

			if (cv::waitKey(30) == 27) break;
		}

	}

	Go_front_Puck();

	//2) close ball!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	while (true)
	{
		pthread_mutex_lock(&frameLocker);
		currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
		pthread_mutex_unlock(&frameLocker);

		if (!currentFrame.empty())
		{
			//1. Find PUCK@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			bool isFindPcuk = Detect_Puck();//Puck 찾기

			if(isFindPcuk)
			{
				std::cout<<"Puck X:"<<Puck_center.x<<" Y:"<<Puck_center.y<<std::endl;

				//2.Find GOAL@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
				while(true)
				{
					//std::cout<<"finding"<<std::endl;
					bool isFindGoal = Detect_goal(HEAD_UP);

					if(isFindGoal)//정면에서 골 찾았음, Puck과 골 비교할 것
						break;

					if (cv::waitKey(30) == 27) break;

				}

				std::cout<<"Goal X:"<<Goal_center.x<<" Y:"<<Goal_center.y<<std::endl;

				//3.Compare them@@@@@@@@@@@@@@@@@@@@@@
				if(Goal_center.x > 165)//163
				{
					Motion2_test(WALK_LEFT_SMALL);//공을 끼고 왼쪽 걷고
					Motion2_test(TURN_RIGHT_SMALL);//오른쪽 회전
					Go_front_Puck();
					std::cout<<"go left"<<std::endl;
				}
				else if(Goal_center.x < 145)//147
				{
					Motion2_test(WALK_RIGHT_SMALL);//공을 끼고 오른쪽 걷고
					Motion2_test(TURN_LEFT_SMALL);//왼쪽 회전
					Go_front_Puck();
					std::cout<<"go right"<<std::endl;
				}
				else
				{
					cout<<"going"<<endl;
					while(true)
					{
						bool IsFindPuck = Detect_Puck(HEAD_DOWN_30);//HEAD_DOWN_LARGE
						bool IsClosePuck = Go_close_Puck(IsFindPuck);

						if(IsClosePuck)
						{
							check_last_direction();
							break;
						}
					}

					Motion2_test(SHOOT_PUCK15);//Puck 치기
					std::cout<<"end in 1"<<std::endl;
					break;//끝
				}
			}

			if (cv::waitKey(30) == 27) break;//imshow로 이미지 보여주려면 반드시 필요
		}

	}
} 

#include <cstdio>
#include <raspicam/raspicam_cv.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <opencv2/imgproc.hpp>


using namespace std;
using namespace cv;

Mat mask_oper(Mat img);

void coordinate(Mat img_mask, Mat img);



Mat mask_oper(Mat img)
{
	int minH, minS, minV, maxH, maxS, maxV;

	Mat img_hsv;
	Mat img_mask;
	cvtColor(img, img_hsv, COLOR_BGR2HSV);								//img객체를 BGR HSV형태로 변환하여 img_hsv에 저장
	
	namedWindow("Control", CV_WINDOW_AUTOSIZE);

	//Create trackbars in "Control" window
	 cvCreateTrackbar("LowH", "Control", &minH, 179); //Hue (0 - 179)
	 cvCreateTrackbar("HighH", "Control", &maxH, 179);
	 
	 cvCreateTrackbar("LowS", "Control", &minS, 255); //Saturation (0 - 255)
	 cvCreateTrackbar("HighS", "Control", &maxS, 255);
	 
	 cvCreateTrackbar("LowV", "Control", &minV, 255); //Value (0 - 255)
	 cvCreateTrackbar("HighV", "Control", &maxV, 255);


	inRange(img_hsv, Scalar(150,50,50), Scalar(180, 255, 255), img_mask);	//img_hsv에서 BGR기준 150,50,50 180,255,255사이의 값을 img_mask에 저장

	inRange(img_mask, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), img_mask);



//opening 작은 점들을 제거 (잡음 제거)
	erode(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE,		//침식
Size(5, 5)));
	dilate(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE,		//팽창
Size(5, 5)));	

//closing 영역의 구멍 메우기 (작은 영역 연결)
	dilate(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE,
 Size(5, 5)));
	erode(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE,
 Size(5, 5)));


	cv::imshow("hsv",img_hsv);
	cv::imshow("frame",img_mask);
	
	

	coordinate(img_mask, img);	
	return img_mask;
}


void coordinate(Mat img_mask, Mat img){

	int x, y, area, left, top, width, height;
	Mat img_labels, stats, centroids;

	//라벨링
	int numOfLables = connectedComponentsWithStats(img_mask, img_labels,
			stats, centroids, 8, CV_32S);

	//영역박스 그리기
	int max = -1, idx = 0;
	for (int j = 1; j < numOfLables; j++) {

		 area = stats.at<int>(j, CC_STAT_AREA);


			if (max < area)
				{
					max = area;
					idx = j;
				}

				x = centroids.at<double>(j,0); //중심좌표
				y = centroids.at<double>(j,1);

		
	}

		 left = stats.at<int>(idx, CC_STAT_LEFT);
		 top = stats.at<int>(idx, CC_STAT_TOP);
		 width = stats.at<int>(idx, CC_STAT_WIDTH);
		 height = stats.at<int>(idx, CC_STAT_HEIGHT);

    //	 circle(img, Point(x, y), 5, Scalar(255, 0, 0), 1);

		 rectangle(img, Point(left, top), Point(left + width, top + height),
			Scalar(0, 0, 255), 1);

		 std::cout<<"----------"<<"\n"<<"x:"<<x<<"\n"<<endl;	
	 	 std::cout<<"y:"<<y<<"\n"<<endl;

	     cv::imshow("picamera test",img);
		

}




int main(int argc, char **argv){

	raspicam::RaspiCam_Cv Camera;
	Mat image;

	Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);	//카메라의 레이아웃 설정 FORMAT				? : CV_8UC3의 역할.(레이아웃)
	Camera.set( CV_CAP_PROP_FRAME_WIDTH, 640);	//화면 너비
	Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 480);	//화면 높이
	//Camera.set( CV_CAP_PROP_POS_MSEC,222);		//현재 위치의 비디오 파일의 밀리초로 읽음.		필요없는 코드
	

	if (!Camera.open()) {cerr<<"Error opening the camera"<<endl; return -1;}

	while(1) {

		Camera.grab();				//현재 프레임을 가져옴
		Camera.retrieve(image);			//가져온 프레임을 디코드하여 image에 저장
		
		mask_oper(image);
		if (waitKey(20) == 27) break;
	}
	
	Camera.release();
} 
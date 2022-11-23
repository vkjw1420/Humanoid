#include <cstdio>
#include <iostream>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc.hpp>


using namespace std;
using namespace cv;

Mat mask_oper(Mat img);

void imgBlur(cv::Mat& img, float sigma); 

void coordinate(Mat img_mask, Mat img);

int main(int argc, char **argv){

	raspicam::RaspiCam_Cv Camera;
	Mat image;

	Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set( CV_CAP_PROP_FRAME_WIDTH, 640);
	Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 480);

	if (!Camera.open()) {cerr<<"Error opening the camera"<<endl; return -1;}

	while(1) {

		Camera.grab();
		Camera.retrieve(image);

		mask_oper(image);
		
		if (waitKey(20) == 27) break;

	}
	Camera.release();
} 

Mat mask_oper(Mat img)
{
	Mat img_hsv;
	Mat img_mask;
	cvtColor(img, img_hsv, COLOR_BGR2HSV);
	
	inRange(img_hsv, Scalar(150,50,50), Scalar(180, 255, 255), img_mask);

//opening 작은 점들을 제거 
	erode(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE,
Size(5, 5)));
	dilate(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE,
Size(5, 5)));	

//closing 영역의 구멍 메우기 
	dilate(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE,
 Size(5, 5)));
	erode(img_mask, img_mask, getStructuringElement(MORPH_ELLIPSE,
 Size(5, 5)));


	cv::imshow("hsv",img_hsv);
	cv::imshow("frame",img_mask);
	
	imgBlur(img_mask,30);

	coordinate(img_mask, img);	

	return img_mask;
}


void coordinate(Mat img_mask, Mat img){

	int x,y,area,left,top,width,height; 
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

				x = centroids.at<double>(j, 0); //중심좌표
				y = centroids.at<double>(j
, 1);

		
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

void imgBlur(cv::Mat& img, float sigma) 

{

    cv::Mat tmp;

    cv::GaussianBlur(img, tmp, cv::Size(0,0), sigma );

    cv::addWeighted(img, 1.5, tmp, -0.5, 0, img);

}





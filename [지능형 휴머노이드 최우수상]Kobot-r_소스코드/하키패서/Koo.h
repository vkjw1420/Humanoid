#ifndef _KOO_H_
#define _KOO_H_
//
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <cmath>

#define PI 3.1415926
#define MIN_RECGONIZE 16

void *updateFrame(void* arg);
void Koo();


 void draw_LINE(cv::Mat& inImage, cv::Point p1, cv::Point p2);
void cut(cv::Mat& image, cv::Mat& blue_image, int P1, int P2);
void  yellow_cut(cv::Mat inImage , cv::Mat& blue_image, bool is_cut);
bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x);
void SetColor(bool is_color, cv::Mat& image, int j, int i, int B, int G, int R);
void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G, int* R);


#endif

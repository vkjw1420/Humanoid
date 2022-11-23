#ifndef _VIDEO_PROCESS_
#define _VIDEO_PROCESS_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

int first_head_detect();
int first_find_puck(int INDEX);
int first_location();
int find_line(int INDEX);
int find_ball(int INDEX);
int goto_ball_LR();
int goto_start_position();
int before_hit();
int find_goal_test();
int find_puck();
int green_blue(int head_position, int gorb);
int GetAngleABC(cv::Point a, cv::Point b, cv::Point c);
int lineTracing();
int get_distance_line_puck(Mat& y_img, Mat& b_img);

void Imshow(string str, Mat image);
void hockey_shooter();
void BinaryColors(cv::Mat &frame, int color1,int color2, int num);
void Cout(string str1, int int1=0, string str2="", int int2=0);
void green_cut(Mat image, Mat& blue_image);
void PreProcess(cv::Mat &frame);
void PreProcess(cv::Mat &frame);
void Motion2_test(int motion);
void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G, int* R );
void SetColor(bool is_color, cv::Mat& image, int j, int i, int B, int G = 0, int R = 0);
void Get_approx_shape(std::vector<cv::Point2f>);
void draw_LINE(cv::Mat& inImage, Point p1, Point p2);
void cut(Mat& image, Mat& blue_image, int P1, int P2);
void getSTATEbyGREEN(int& LR, int& UD );
void SCAN();
void draw_garo(cv::Mat& image, int y);
void draw_sero(cv::Mat& image, int x);
void draw_rectangle(cv::Mat& image, int a,int b,int c, int d);
void getFrame(int color1, int color2, bool is_bin=1);
void draw_approx(cv::Mat& image, std::vector<cv::Point2f> approx);
void walk_evade_yellow(int motion, int head = 161);

void* ImageFromCamera(void *) ;
void* ImageProcess(void *) ;
void *updateFrame(void* arg);

bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x);
bool yellow_cut(Mat inImage , Mat& blue_image, bool is_cut=1);
bool MASK(Mat image, Vec3b color, int y, int x, int size ,int num);

std::vector<cv::Point2f> DrawShapeDetection_Control(cv::Mat &m_image,int* x, int* y);
#endif

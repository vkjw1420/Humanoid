#ifndef _VIDEO_PROCESS_
#define _VIDEO_PROCESS_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <cmath>

#define WALK_FORWARD 101
#define TURN_RIGHT 112
#define TURN_LEFT 113

#define HEAD_DOWN 192
#define HEAD_DOWN_LARGE 190
#define HEAD_UP 162//195
#define HEAD_LEFT_60 153
#define HEAD_RIGHT_60 157
#define HEAD_FORWARD 159

#define WALK_LEFT 110
#define WALK_RIGHT 111
#define SHOOT_PUCK6 116
#define SHOOT_PUCK7 117
#define SHOOT_PUCK8 118
#define SHOOT_PUCK 119
#define SHOOT_PUCK10 120
#define SHOOT_PUCK11 121
#define SHOOT_PUCK12 122
#define SHOOT_PUCK13 123
#define SHOOT_PUCK14 124
#define SHOOT_PUCK15 125

#define WALK_FORWARD_ONE 135
#define TURN_LEFT_SMALL 114
#define TURN_RIGHT_SMALL 115
#define WALK_LEFT_SMALL 103
#define WALK_RIGHT_SMALL 104

#define WALK_FORWARD_ONE_SMALL 134

#define HEAD_DOWN_30 164

#define SIT 105

#define BASE 99

void *updateFrame(void* arg);
void PreProcess(cv::Mat &frame);

void Motion2_test(int motion);
bool Detect_goal(int motion = HEAD_UP);
bool Detect_Puck(int motion = HEAD_DOWN);
bool Detect_Puck_in_goal();
bool Go_front_Puck();
bool Go_close_Puck(bool isFindPuck);
bool Go_to_Puck(bool isFindPuck);
void check_last_direction();
void Curling();
#endif 

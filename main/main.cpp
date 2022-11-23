#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

//�� �̵� ����� �� ���ڱ� �̵��ϴ��� Ȯ�� �ʿ�
#define TURN_LEFT_10            1
#define GO_FORWARD_SLOW_6   2
#define TURN_RIGHT_10            3
#define TURN_LEFT_3               4
#define CHECK_DISTANCE         5
#define TURN_RIGHT_3            6
#define TURN_LEFT_20            7
#define GO_FORWARD_SLOW      8
#define TURN_RIGHT_20            9
#define GO_FORWARD_FAST      10
#define GO_FORWARD_KEEP      11
#define GO_BACKWARD_KEEP      12
#define GO_RIGHT_KEEP            13
#define GO_LEFT_KEEP            14
#define GO_LEFT_20               15
#define SHUTDOWN               16
#define LEFT_HEAD_90            17
#define GYRO_OFF                  18
#define TURN_RIGHT_60            19
#define GO_RIGHT_20               20
#define FRONT_HEAD               21
#define TURN_LEFT_45            22
#define ERROR_SOUND            23   //���� ������� üũ �ʿ�.
#define TURN_RIGTH_45            24
#define TURN_LEFT_60            25
#define BASIC_POSTURE            26
#define RIGHT_HEAD_90            27
#define LEFT_HEAD_45            28
#define DOWN_HEAD_80            29
#define RIGHT_HEAD_45            30
#define DOWN_HEAD_60            31
#define GO_BACKWARD_SLOW   32

typedef struct detectedImage {
	int x;            //x��ǥ
	int y;            //y��ǥ
	int degree;         //����
	int s_flag;         //���� ���� Ȯ�� 0���� 1����
};      //������ ��� ��ֹ��� ���ؼ� �����ұ�? �ƴϸ� ���� �����ұ�?

int uart_serial;
int stage_count = 0;
detectedImage current_image;
detectedImage line_image;
detectedImage barricade_image;
detectedImage door_image;


int start_uart();

void action(int order);

//�� ���� ó���� �ٸ� �Լ��� Ȯ���ϴ°� ������ ������
detectedImage detect_line();         //�� Ȯ��
detectedImage detect_barricade();      //��ֹ� Ȯ��
detectedImage detect_line_barricade();   //�� + ��ֹ� Ȯ��?(�����ʿ�)
detectedImage detect_stair();         //��� ����
detectedImage detect_valve();         //��� Ÿ�� Ȯ��

void follow_line();

//�� ��ֹ� �ܰ� �Լ�
void start();
void stairs();
void bridge();
void before_tunnel();
void tunnel();
void before_valve();
void valve();

int main() {
	if (start_uart() < 0) {
		printf("uart error");
	}
	else {
		while (1) {
			switch (stage_count) {
			case 6: before_tunnel();      break;
			case 7: tunnel();               break;
			case 8: before_valve();      break;
			case 9: valve();               break;
			default: break;
			}
			if (stage_count > 9)
				break;
		}
	}

	return 0;
}

int start_uart() {
	if ((fd = serialOpen("/dev/ttyAMA0", 4800)) < 0) {
		return -1;
	}
	if (wiringPiSetup() == -1) {
		return -1;
	}
	return 1;
}

int action(int order) {               //�κ� �ൿ ���
	serialPutchar(uart_serial, order);

	switch (order) {
		//ex) case DOWN_HEAD_30: sleep(1000);
		//     case DOWN_HEAD_30: sleep(1000);
		//     case DOWN_HEAD_30: sleep(1000);
		//     case DOWN_HEAD_30: sleep(1000);
	case CHECK_DISTANCE: return; //���ܼ� �Ÿ� �� �����ϱ�
	default:;
	}

	if (order != serialGetchar(uart_serial)) {
		printf("(%2d) motion error\n", order);
	}
}



detectedImage detect_line() {                     //�� ����
	detectedImage image_info;

	//�� ���� ó�� �ڵ�.

	return image_info;
}
detectedImage detect_barricade() {                  //��ֹ� ����
	detectedImage image_info;

	//��ֹ� ���� ó�� �ڵ�.

	return image_info;
}
//or
detectedImage detect_barricade() {                  //�� + ��ֹ� ����
	detectedImage image_info;

	//�� ���� ó�� �ڵ�
	//��ֹ� ���� ó�� �ڵ�.

	return image_info;
}
detectedImage detect_stair() {                     //��� ����
	detectedImage image_info;

	//��� ���� �ڵ�

	return image_info;
}
detectedImage detect_valve() {                     //�� ����
	detectedImage image_info;

	//�� ���� ó�� �ڵ�.

	return image_info;
}

void follow_line() {
	if (current_image.degree > lean_to_the_left) { //���� �������� ����
		action(TURN_LEFT_10);
	}
	else if (current_image.degree < lean_to_the_right) { //���� ���������� ����
		action(TURN_RIGHT_10);
	}
	else if (current_image.x < lean_to_the_left) { //���� ���ʿ� �ִٸ�
		action(GO_RIGHT_20);
	}
	else if (current_image.x > lean_to_the_right) { //���� �����ʿ� �ִٸ�
		action(GO_LEFT_20);
	}
	else {
		action(GO_FORWARD_5);
	}
}

void start() {
	action(DOWN_HEAD_30);         //�Ӹ� ���� ����

	while (1) {
		current_image = detect_line();   //��ֹ��� ���� ������ ���� �����ؾ� ������ ���� �� ����.

		if (current_image.s_flag == 0) {   //���н� �ڼ� ���� �ʿ�
			continue;
		}
		else if (current_image.detected_object == LINE) {
			follow_line();
		}
		else if (current_image.detected_object == STAIR && current_image.y > enough_position) {
			break;
		}
		else {
			action(GO_FORWARD_1);
		}
	}
	stage_count++;
}

void stairs() {         //�������θ� �Ѿ����� ���� ��û, �� �������� �䱸
	int stair_count = 0;
	action(DOWN_HEAD_80);

	while (1) {
		current_image = detect_stair();

		if (current_image.s_flag == 0) {
			continue;
		}
		else if (current_image.detected_object == STAIR && current_image.degree != horizon) {
			if (current_image.degree > lean_to_the_left) { //����� �������� ����
				action(TURN_LEFT_10);
			}
			else if (current_image.degree < lean_to_the_right) { //����� ���������� ����
				action(TURN_RIGHT_10);
			}
		}
		else if (current_image.detected_object == STAIR && current_image.degree == horizon) {
			if (current_image.y == enough_position) {
				if (stair_count < 2)
					action(UP_STAIR);
				else
					action(DOWN_STAIR);
				stair_count++;
			}
			else {
				action(GO_FORWARD_1);
			}
		}
		if (stair_count >= 4)
			break;
	}
	stage_count++;
}

void bridge() {                  //�ܳ��� �ٸ��� �׳� �� Ʈ��ŷ���ؼ� �������� ����Ұ��̶� ����
	action(DOWN_HEAD_30);         //�Ӹ� ���� ����

	while (1) {
		current_image = detect_line();   //��ֹ��� ���� ������ ���� �����ؾ� ������ ���� �� ����.

		if (current_image.s_flag == 0) {
			continue;
		}
		else if (current_image.detected_object == LINE) {
			follow_line();
		}
		else if (current_image.detected_object == STAIR && current_image.y > enough_position) {
			break;
		}
		else {
			action(GO_FORWARD_1);
		}
	}
	stage_count++;
}         //�׳� �ܳ��� �ٸ� �������� ���� ��ֹ����� �ϳ��� �˰������� �����ϸ� ���?



void before_tunnel() {
	while (1) {
		current_image = detect_line_barricade();

		if (current_image.s_flag == 0) {
			continue;
		}
		else if (current_image.detected_object == LINE) {
			follow_line();
		}
		else if (current_image.detected_object == BARRICADE) {
			if (current_image.object_size < Near_object) {   //Near_object�� ���� �����ʿ�
				action(GO_FORWARD_1);
			}
			else {
				action(REMOVE_BARRICADE);
			}
		}

		if (action(CHECK_DISTANCE) < enough_distance) {      //���ܼ����� �ͳ� ����
			break;
		}
	}
	stage_count++;
}

void tunnel() {
	action(PASS_TUNNEL);
	stage_count++;
}

void before_valve() {
	while (1) {
		current_image = detect_line_barricade();

		if (current_image.s_flag == 0) {
			continue;
		}
		else if (current_image.detected_object == LINE) {
			follow_line();
		}
		else if (current_image.detected_object == BARRICADE) {
			if (current_image.object_size < enough_size) {   //enough_size�� ���� �����ʿ�
				action(GO_FORWARD_1);
			}
			else {
				action(REMOVE_BARRICADE);
			}
		}

		if (current_image.detected_object == VALVE_TOWER && current_image.object_size > enough_size) {      //��� Ÿ���� �����Ǹ� ���� ���࿡ ������� �����Ѵٸ�
			break;
		}
	}
	stage_count++;
}

void valve() {
	while (1) {
		current_image = detect_valve();

		if (current_image.detected_object == VALVE && action(CHECK_DISTANCE) < enough_distance) {      //��� Ÿ�� ���� Ȯ��. �̹����� ũ�� ó�����ٴ� ���ܼ��� Ȯ���ҵ���
			if (current_image.x < lean_to_the_left) { //��갡 ���ʿ� �ִٸ�
				action(GO_RIGHT_20);
			}
			else if (current_image.x > lean_to_the_right) { //��갡 �����ʿ� �ִٸ�
				action(GO_LEFT_20);
			}
			else if (current_image.degree < enough_degree) {      //����� �������� �������� �ʴ´ٸ�
				action(TOUCH_VALVE);
			}
			else {
				break;
			}
		}
	}
	stage_count++;
}
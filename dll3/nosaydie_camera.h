#pragma once
#ifndef CODE_NOSAYDIE_CAMERA_H_
#define CODE_NOSAYDIE_CAMERA_H_
#include "common.h"
//�궨����ر���-------------------------------
#define LCDW 160
#define LCDH 120
//��������ͷ����
#define MT9V03X_W               160             	//ͼ���� 	��Χ1-188
#define MT9V03X_H               120             	//ͼ��߶�	��Χ1-120
//�Զ��庯��-----------------------------------
void get_deal_image();      //ȡ��������ͼ��
int average_value(uint8* c);//���ֵ����ֵ�˲��ã�
void average_filter(void);  //��ֵ�˲�
uint8 OSTU_bin(uint8 width, uint8 height, uint8* Image);//�����̬��ֵ
uint8 GetOSTUThreshold(uint8(*img)[LCDW], uint16 start_row, uint16 end_row, uint16 start_col, uint16 end_col); //�����̬��ֵ
void get_binImage(uint8 thres);//��ֵ��
uint8 black_(uint8 x);  //�ж��Ƿ��Ǻ����ص�
uint8 white_(uint8 x);  //�ж��Ƿ��ǰ����ص�
int edge_point_ornot(uint8 row, uint8 side);//�ж��Ƿ���ڱ߽��,�����ء�-1��û���ҵ��߽�㣬��ֵ�������ҵ��ı߽�� ��
int Get_angle(uint8 ax, uint8 ay, uint8 bx, uint8 by, uint8 cx, uint8 cy);//��ȡ�սǵĽǶ�ֵ
void clear_point();     //��սṹ������
void get_mid();         //�������
uint8* Image_process(uint8 imagein[LCDH][LCDW], uint8 sidee[150 * 2 * 3]);
//�Զ���ṹ�弰��ر���
struct LEFT_EDGE
{
	uint8 row;  //������
	uint8 col;  //������
	int flag; //���ڱ߽�ı�־
};
struct RIGHT_EDGE
{
	uint8 row;  //������
	uint8 col;  //������
	int flag; //���ڱ߽�ı�־
};
#endif /* CODE_NOSAYDIE_CAMERA_H_ */
#pragma once
#ifndef CODE_NOSAYDIE_CAMERA_H_
#define CODE_NOSAYDIE_CAMERA_H_
#include "common.h"
//宏定义相关变量-------------------------------
#define LCDW 160
#define LCDH 120
//配置摄像头参数
#define MT9V03X_W               160             	//图像宽度 	范围1-188
#define MT9V03X_H               120             	//图像高度	范围1-120
//自定义函数-----------------------------------
void get_deal_image();      //取出待处理图像
int average_value(uint8* c);//求均值（均值滤波用）
void average_filter(void);  //均值滤波
uint8 OSTU_bin(uint8 width, uint8 height, uint8* Image);//大津法求动态阈值
uint8 GetOSTUThreshold(uint8(*img)[LCDW], uint16 start_row, uint16 end_row, uint16 start_col, uint16 end_col); //大津法求动态阈值
void get_binImage(uint8 thres);//二值化
uint8 black_(uint8 x);  //判断是否是黑像素点
uint8 white_(uint8 x);  //判断是否是白像素点
int edge_point_ornot(uint8 row, uint8 side);//判断是否存在边界点,并返回【-1：没有找到边界点，正值：返回找到的边界点 】
int Get_angle(uint8 ax, uint8 ay, uint8 bx, uint8 by, uint8 cx, uint8 cy);//求取拐角的角度值
void clear_point();     //清空结构体数据
void get_mid();         //拟合中线
uint8* Image_process(uint8 imagein[LCDH][LCDW], uint8 sidee[150 * 2 * 3]);
//自定义结构体及相关变量
struct LEFT_EDGE
{
	uint8 row;  //行坐标
	uint8 col;  //列坐标
	int flag; //存在边界的标志
};
struct RIGHT_EDGE
{
	uint8 row;  //行坐标
	uint8 col;  //列坐标
	int flag; //存在边界的标志
};
#endif /* CODE_NOSAYDIE_CAMERA_H_ */
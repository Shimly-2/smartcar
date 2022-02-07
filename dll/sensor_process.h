#pragma once
#pragma once
#ifndef _SENSOR_PROCESS_H_
#define _SENSOR_PROCESS_H_

#define COL     188//图像宽度   范围1-752     K60采集不允许超过188
#define ROW     64//图像高度	范围1-480

#define CAMERA_COLUMN_MAX 80 //定义图像列长度，实际只有120，预留多
typedef struct Coordinate
{
	float x[CAMERA_COLUMN_MAX];
	float y[CAMERA_COLUMN_MAX];
	//   float k[CAMERA_LINE+4];          //斜率
}*p_coordinate, coordinate;

void img_binary(void);
void Line_coordinates_init();
//void Get_line_LMR();
uint8* Get_line_LMR(uint8 imagein[ROW][COL], uint8 side[ROW * 3]);
void Track_calculate_judge(void/*unsigned char img_2[CAMERA_H][CAMERA_W]*/);

struct PID
{
	float KP;
	float KI;
	float KD;
	float tp;
	float td;
	int16 error;
	int16 cnt;
	int16 last_error;
	int16 prev_error;
	int16 out;
	int16 base_speed;
	int16 exp_speed;
	int16 real_speed;
	//int16 giving_speed;
};//L_motor,R_motor,Front_car,Later_car,Steer;
extern struct PID Steer;
extern struct PID L_motor;
extern struct PID R_motor;
void Steer_PD_control(void);
void Motor_PID_control(void);
void Steer_control(void);
void BOMA_speed_select(void);
#endif
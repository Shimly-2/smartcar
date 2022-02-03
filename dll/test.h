#pragma once
#include "stdint.h"

/* 摄像头读取数据的长宽 */
#define MT9V03X_W 188
#define MT9V03X_H 120

/* 摄像头存储单帧灰度图像的数组(4字节对齐) */
//uint8_t MT9V03X_image[MT9V03X_H][MT9V03X_W];

typedef struct
{
	uint16_t pixelMinimum;
	uint16_t upperCutLine;
	uint16_t lostBeaconLimit;
	uint16_t pixelCntLimit_Normal;
	uint16_t pixelCntLimit1;
	uint16_t pixelCntLimit2;
	float speedNormal;
	float speedGear1;
	float speedGear2;
	float speedGear3;
	float voltageStart;
	int16_t leftSearchSpeed;
	int16_t rightSearchSpeed;
	int16_t OTSUth_min;
} xlwParamStructure;


typedef struct
{
	float w;      //车轮距 m
	float r;      //车轮半径 m
	float h;      //质心高度 m
	float b;      //质心投影与P1距离 m
	float m;      //车体质量 kg
	float R;      //跑道环路与转弯半径 m
	float g;      //重力加速度 m/s^2
} motorbike;

typedef struct
{
	uint16_t rowWeight;
	uint16_t colWeight;
	uint32_t pixelCnt;
	float vcap;
} beaconInform;

uint8_t XLW_otsuThreshold(uint8_t* image, uint16_t col, uint16_t row);
beaconInform XLW_simpleTrace(uint8_t* image, uint16_t col, uint16_t row, uint8_t threshold);
//Mat Vec2Mat(uchar** array, int row, int col);
//uchar** Mat2Vec(Mat image);
//Mat Binarization(uchar** array, int row, int col, uint8_t th);
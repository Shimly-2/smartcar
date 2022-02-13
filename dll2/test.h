#pragma once
#include "stdint.h"

/* ����ͷ��ȡ���ݵĳ��� */
#define MT9V03X_W 188
#define MT9V03X_H 120

/* ����ͷ�洢��֡�Ҷ�ͼ�������(4�ֽڶ���) */
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
	float w;      //���־� m
	float r;      //���ְ뾶 m
	float h;      //���ĸ߶� m
	float b;      //����ͶӰ��P1���� m
	float m;      //�������� kg
	float R;      //�ܵ���·��ת��뾶 m
	float g;      //�������ٶ� m/s^2
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
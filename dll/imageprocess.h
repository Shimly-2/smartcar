#pragma once
/*!
  * @file     LQ_ImageProcess.h
  *
  * @brief
  *
  * @company  北京龙邱智能科技
  *
  * @author   LQ-005
  *
  * @note     Tab键 4个空格
  *
  * @version  V1.0
  *
  * @par URL  http://shop36265907.taobao.com
  *           http://www.lqist.cn
  *
  * @date     2020年6月9日
  */

#include "stdint.h"
#include "test.h"
#ifndef SRC_APPSW_TRICORE_USER_LQ_IMAGEPROCESS_H_
#define SRC_APPSW_TRICORE_USER_LQ_IMAGEPROCESS_H_

float* control(float omega, uint8_t cha, float motor[2], uint8_t mode);
void hhh(uint8_t image[MT9V03X_H][MT9V03X_W], uint8_t img[MT9V03X_H][MT9V03X_W], uint8_t th);
uint8_t Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W], unsigned char mode);
int GetOSTU(unsigned char tmImage[MT9V03X_H][MT9V03X_W]);
void SobelThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W], uint8_t Threshold);
void SobelAutoThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W]);
int GetOSTU(unsigned char tmImage[MT9V03X_H][MT9V03X_W]);
uint8_t XLW_otsuThreshold(uint8_t image[MT9V03X_H][MT9V03X_W], uint16_t col, uint16_t row);
void TFTSPI_BinRoadSide(uint8_t imageOut[MT9V03X_H][2]);
uint8_t RoadIsStraight(uint8_t imageSide[MT9V03X_H][2]);
uint8_t RoadIsZebra(uint8_t image[MT9V03X_H][MT9V03X_W], uint8_t* flag);
uint8_t RoadIsCross(uint8_t imageSide[MT9V03X_H][2], uint8_t* flag);
uint8_t RoadIsRoundabout(uint8_t image[MT9V03X_H][2], uint8_t* flag);
void RoundaboutGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t status);
uint8_t RoundaboutGetArc(uint8_t imageSide[MT9V03X_H][2], uint8_t status, uint8_t* index);
void ImageAddingLine(uint8_t imageSide[MT9V03X_H][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY);
uint8_t ImageGetHop(uint8_t imageSide[MT9V03X_H][2], uint8_t state, uint8_t* x, uint8_t* y);
void RoundaboutProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t* state);
void CrossGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2]);
void CrossProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t* state);
int16_t RoadGetSteeringError(uint8_t imageSide[MT9V03X_H][2], uint8_t lineIndex);
uint8_t RoadIsNoSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2], uint8_t lineIndex);
void RoadNoSideProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2], uint8_t mode, int lineIndex);
uint8_t ImageGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2]);
void ImagePortFilter(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][MT9V03X_W]);
void ImageProcess(void);
void ZebraProcess(uint8_t imageSide[MT9V03X_H][2], uint8_t state, int16_t* speed);

uint8_t* test(uint8_t image_in[MT9V03X_H][MT9V03X_W], uint8_t image_out[MT9V03X_H][MT9V03X_W]);

#endif /* SRC_APPSW_TRICORE_USER_LQ_IMAGEPROCESS_H_ */

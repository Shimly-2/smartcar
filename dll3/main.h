#pragma once
#ifndef MAIN_H
#define MAIN_H

//#include "headfile.h"
#include "imgproc.h"
//#include "utils.h"
#include "flash_param.h"
#include <stdbool.h>
#include <stdint.h>


extern image_t img_raw;
extern image_t img_thres;
extern image_t img_line;

extern bool line_show_sample;
extern bool line_show_blur;
extern bool track_left;

extern float angle;

//��������ͷ����
#define MT9V03X_CSI_W               188             //ͼ����  ��Χ1-752      RT105X RT106X �ɼ�ʱ�п�ȱ���Ϊ4�ı���
#define MT9V03X_CSI_H               120             //ͼ��߶�	��Χ1-480

#define ROAD_WIDTH      (0.45)
#define POINTS_MAX_LEN  (MT9V03X_CSI_H)

#define FAR_POINTS_MAX_LEN  (POINTS_MAX_LEN)

// ԭͼ���ұ���
extern int ipts0[POINTS_MAX_LEN][2];
extern int ipts1[POINTS_MAX_LEN][2];
extern int ipts0_num, ipts1_num;
// �任�����ұ���
extern float rpts0[POINTS_MAX_LEN][2];
extern float rpts1[POINTS_MAX_LEN][2];
extern int rpts0_num, rpts1_num;
// �任�����ұ���+�˲�
extern float rpts0b[POINTS_MAX_LEN][2];
extern float rpts1b[POINTS_MAX_LEN][2];
extern int rpts0b_num, rpts1b_num;
// �任�����ұ���+�Ⱦ����
extern float rpts0s[POINTS_MAX_LEN][2];
extern float rpts1s[POINTS_MAX_LEN][2];
extern int rpts0s_num, rpts1s_num;
// ���ұ��߾ֲ��Ƕȱ仯��
extern float rpts0a[POINTS_MAX_LEN];
extern float rpts1a[POINTS_MAX_LEN];
extern int rpts0a_num, rpts1a_num;
// ���ұ��߾ֲ��Ƕȱ仯��+�Ǽ�������
extern float rpts0an[POINTS_MAX_LEN];
extern float rpts1an[POINTS_MAX_LEN];
extern int rpts0an_num, rpts1an_num;
// ��/������
extern float rptsc0[POINTS_MAX_LEN][2];
extern float rptsc1[POINTS_MAX_LEN][2];
extern int rptsc0_num, rptsc1_num;
// ����
extern float(*rpts)[2];
extern int rpts_num;
// ��һ������
extern float rptsn[POINTS_MAX_LEN][2];
extern int rptsn_num;

// Y�ǵ�
extern int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
extern bool Ypt0_found, Ypt1_found;

// L�ǵ�
extern int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
extern bool Lpt0_found, Lpt1_found;

//����Ϊʮ�ֿ���ѰԶ�߲���,�㷨�볣��Ѱ����ͬ

extern bool far_Lpt0_found, far_Lpt1_found;
extern int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
//ԭͼԶ�����ұ���
extern int far_ipts0[FAR_POINTS_MAX_LEN][2];
extern int far_ipts1[FAR_POINTS_MAX_LEN][2];
extern int far_ipts0_num, far_ipts1_num;
//�任������Զ����
extern float far_rpts0[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0_num, far_rpts1_num;
//�任������Զ����+�˲�
extern float far_rpts0b[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1b[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0b_num, far_rpts1b_num;
//�任������Զ����+�Ⱦ����
extern float far_rpts0s[FAR_POINTS_MAX_LEN][2];
extern float far_rpts1s[FAR_POINTS_MAX_LEN][2];
extern int far_rpts0s_num, far_rpts1s_num;
// ����Զ���߾ֲ��Ƕȱ仯��
extern float far_rpts0a[FAR_POINTS_MAX_LEN];
extern float far_rpts1a[FAR_POINTS_MAX_LEN];
extern int far_rpts0a_num, far_rpts1a_num;
// ����Զ���߾ֲ��Ƕȱ仯��+�Ǽ�������
extern float far_rpts0an[FAR_POINTS_MAX_LEN];
extern float far_rpts1an[FAR_POINTS_MAX_LEN];
extern int far_rpts0an_num, far_rpts1an_num;

extern bool is_straight0, is_straight1;

enum track_type_e {
	TRACK_LEFT,
	TRACK_RIGHT,
};
extern enum track_type_e track_type;


#endif // MAIN_H
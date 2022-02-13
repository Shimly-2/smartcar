#pragma once
#ifndef CIRCLE_H
#define CIRCLE_H
#include "main.h"
enum circle_type_e {
	CIRCLE_NONE = 0,                            // ��Բ��ģʽ
	CIRCLE_LEFT_BEGIN, CIRCLE_RIGHT_BEGIN,      // Բ����ʼ��ʶ�𵽵���L�ǵ���һ�೤ֱ����
	CIRCLE_LEFT_IN, CIRCLE_RIGHT_IN,            // Բ�����룬���ߵ�һ��ֱ����һ��Բ����λ�á�
	CIRCLE_LEFT_RUNNING, CIRCLE_RIGHT_RUNNING,  // Բ���ڲ���
	CIRCLE_LEFT_OUT, CIRCLE_RIGHT_OUT,          // ׼����Բ������ʶ�𵽳�������L�ǵ㡣
	CIRCLE_LEFT_END, CIRCLE_RIGHT_END,          // Բ�����������ٴ��ߵ�����ֱ����λ�á�
	CIRCLE_NUM,                                 //
};

extern const char* circle_type_name[CIRCLE_NUM];

extern enum circle_type_e circle_type;

void check_circle();

void run_circle();

void draw_circle();

#endif // CIRCLE_H
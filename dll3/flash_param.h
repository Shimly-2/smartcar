#pragma once
#ifndef _FLASH_PARAM_H_
#define _FLASH_PARAM_H_

// �ɵ�����д��flash

#include <stdint.h>
#include <stdbool.h>


//
extern float thres;                 // ��ֵ����ֵ����Ҫ��������ʼ��(����ʹ������Ӧ��ֵ����ʹ�ø���ֵ)
extern float block_size;            // ����Ӧ��ֵ��block��С
extern float clip_value;            // ����Ӧ��ֵ����ֵ�ü���
extern float begin_x;               // ��ʼ�����ͼ�����ĵ�����ƫ����
extern float begin_y;               // ��ʼ�����ͼ��ײ�������ƫ����
extern float line_blur_kernel;      // ���������˲��˵Ĵ�С
extern float pixel_per_meter;       // ����ͼ�У�ÿ�����ض�Ӧ�ĳ���
extern float sample_dist;           // ���ߵȾ�����ļ��
extern float angle_dist;            // �������ת��ʱ�����������ľ���
extern float aim_distance;          // Ԥê�㳤��
extern float far_rate;              //
extern bool adc_cross;              // �Ƿ����õ�й�ʮ��


void flash_param_init();

bool flash_param_check();

void flash_param_load();

void flash_param_write();

#endif /* _FLASH_PARAM_H_ */
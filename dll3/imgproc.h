#pragma once
#ifndef IMGPROC_H
#define IMGPROC_H

#include <stdint.h>


typedef struct image {
	uint8_t* data;
	uint32_t width;
	uint32_t height;
	uint32_t step;
} image_t;

typedef struct fimage {
	float* data;
	uint32_t width;
	uint32_t height;
	uint32_t step;
} fimage_t;

#define AT_IMAGE(img, x, y)          ((img)->data[(y)*(img)->step+(x)])
#define AT_IMAGE_CLIP(img, x, y)     AT_IMAGE(img, clip(x, 0, (img)->width-1), clip(y, 0, (img)->height-1))

#define DEF_IMAGE(ptr, w, h)         {ptr, w, h, w}
#define ROI_IMAGE(img, x1, y1, w, h) {&AT_IMAGE(img, x1, y1), w, h, img.width}

// ���ͼƬ��img0��img1������ָ����ͬͼƬ
void clone_image(image_t* img0, image_t* img1);

// ���ͼƬ
void clear_image(image_t* img);

// ������ֵ����img0��img1����ָ����ͬͼƬ
void threshold(image_t* img0, image_t* img1, uint8_t thres, uint8_t low_value, uint8_t high_value);

// ����Ӧ��ֵ����img0��img1����ָ����ͬͼƬ
void adaptive_threshold(image_t* img0, image_t* img1, int block_size, int down_value, uint8_t low_value, uint8_t high_value);

// �����߼��룬����Ϊ255ʱ��Ϊ255
void image_and(image_t* img0, image_t* img1, image_t* img2);

// �����߼��򣬼���Ϊ255ʱ��Ϊ255
void image_or(image_t* img0, image_t* img1, image_t* img2);

// ��Сֵ������x2
void minpool2(image_t* img0, image_t* img1);

// 3x3��˹�˲���img0��img1������ָ����ͬͼƬ������Ȧ���ص㲻�������
void blur3(image_t* img0, image_t* img1);

// 3x3 Sobel�ݶȼ��㣬img0��img1������ָ����ͬͼƬ������Ȧ���ص㲻�������
void sobel3(image_t* img0, image_t* img1);

// 3x3��ʴ���㣬img0��img1������ָ����ͬͼƬ������Ȧ���ص㲻�������
void erode3(image_t* img0, image_t* img1);

// 3x3�������㣬img0��img1������ָ����ͬͼƬ������Ȧ���ص㲻�������
void dilate3(image_t* img0, image_t* img1);

// ͼ��任������ڲ�ֵ��img0��img1������ָ����ͬͼƬ��ȥ���䣬ͶӰ�任�ȡ�ʹ��opencv���ɱ任��
// δ���ԣ�������BUG
void remap(image_t* img0, image_t* img1, fimage_t* mapx, fimage_t* mapy);

// ����Ѳ��ͬʱ����Ӧ��ֵ������(x,y)��ʼ�����ذױ�����
void findline_lefthand_adaptive(image_t* img, int block_size, int clip_value, int x, int y, int pts[][2], int* num);

// ����Ѳ��ͬʱ����Ӧ��ֵ������(x,y)��ʼ�����ذױ�����
void findline_righthand_adaptive(image_t* img, int block_size, int clip_value, int x, int y, int pts[][2], int* num);

// ��򷨼����ֵ����ֵ
uint16_t getOSTUThreshold(image_t* img, uint8_t MinThreshold, uint8_t MaxThreshold);

// �㼯�����˲�
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel);

// �㼯�Ⱦ����
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int* num2, float dist);

// �㼯�ֲ��Ƕȱ仯��
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist);

// �Ƕȱ仯�ʷǼ�������
void nms_angle(float angle_in[], int num, float angle_out[], int kernel);

// ����߸�������
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist);

// �ұ��߸�������
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist);

// ����X
void draw_x(image_t* img, int x, int y, int len, uint8_t value);

// ����O
void draw_o(image_t* img, int x, int y, int radius, uint8_t value);

uint8_t SobelAuto(image_t* img, int x, int y);

#endif // IMGPROC_H
#include "imgproc.h"
#include "common.h"
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>

#define AT                  AT_IMAGE
#define AT_CLIP(img, x, y)  AT_IMAGE((img), clip((x), 0, (img)->width-1), clip((y), 0, (img)->height-1));

extern int clip(int x, int low, int up);

void clone_image(image_t* img0, image_t* img1) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(img0 != img1 && img0->data != img1->data);

	if (img0->width == img0->step && img1->width == img1->step) {
		memcpy(img1->data, img0->data, img0->width * img0->height);
	}
	else {
		for (int y = 0; y < img0->height; y++) {
			memcpy(&AT(img1, 0, y), &AT(img0, 0, y), img0->width);
		}
	}
}

void clear_image(image_t* img) {
	assert(img && img->data);
	if (img->width == img->step) {
		memset(img->data, 0, img->width * img->height);
	}
	else {
		for (int y = 0; y < img->height; y++) {
			memset(&AT(img, 0, y), 0, img->width);
		}
	}
}

// �̶���ֵ��ֵ��
void threshold(image_t* img0, image_t* img1, uint8_t thres, uint8_t low_value, uint8_t high_value) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img0->width == img1->width && img0->height == img1->height);

	// �ȱ���y�����x�Ƚ�cache-friendly
	for (int y = 0; y < img0->height; y++) {
		for (int x = 0; x < img0->width; x++) {
			AT(img1, x, y) = AT(img0, x, y) < thres ? low_value : high_value;
		}
	}
}

// ����Ӧ��ֵ��ֵ��
void adaptive_threshold(image_t* img0, image_t* img1, int block_size, int down_value, uint8_t low_value, uint8_t high_value) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img0->data != img1->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(block_size > 1 && block_size % 2 == 1);

	int half = block_size / 2;
	// �ȱ���y�����x�Ƚ�cache-friendly
	for (int y = 0; y < img0->height; y++) {
		for (int x = 0; x < img0->width; x++) {
			int thres_value = 0;
			for (int dy = -half; dy <= half; dy++) {
				for (int dx = -half; dx <= half; dx++) {
					thres_value += AT_CLIP(img0, x + dx, y + dy);
				}
			}
			thres_value /= block_size * block_size;
			thres_value -= down_value;
			AT(img1, x, y) = AT(img0, x, y) < thres_value ? low_value : high_value;
		}
	}
}

// ͼ���߼���
void image_and(image_t* img0, image_t* img1, image_t* img2) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img2 && img2->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(img0->width == img2->width && img0->height == img2->height);

	// �ȱ���y�����x�Ƚ�cache-friendly
	for (int y = 0; y < img0->height; y++) {
		for (int x = 0; x < img0->width; x++) {
			AT(img2, x, y) = (AT(img0, x, y) == 0 || AT(img1, x, y) == 0) ? 0 : 255;
		}
	}
}

// ͼ���߼���
void image_or(image_t* img0, image_t* img1, image_t* img2) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img2 && img2->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(img0->width == img2->width && img0->height == img2->height);

	// �ȱ���y�����x�Ƚ�cache-friendly
	for (int y = 0; y < img0->height; y++) {
		for (int x = 0; x < img0->width; x++) {
			AT(img2, x, y) = (AT(img0, x, y) == 0 && AT(img1, x, y) == 0) ? 0 : 255;
		}
	}
}

// 2x2��С�ػ�(�����߽��Ǻ�ɫ����С�ػ����ԽϺñ��������߽�)
void minpool2(image_t* img0, image_t* img1) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img0->width / 2 == img1->width && img0->height / 2 == img1->height);
	assert(img0 != img1 && img0->data != img1->data);

	uint8_t min_value;
	// �ȱ���y�����x�Ƚ�cache-friendly
	for (int y = 1; y < img0->height; y += 2) {
		for (int x = 1; x < img0->width; x += 2) {
			min_value = 255;
			if (AT(img0, x, y) < min_value) min_value = AT(img0, x, y);
			if (AT(img0, x - 1, y) < min_value) min_value = AT(img0, x - 1, y);
			if (AT(img0, x, y - 1) < min_value) min_value = AT(img0, x, y - 1);
			if (AT(img0, x - 1, y - 1) < min_value) min_value = AT(img0, x - 1, y - 1);
			AT(img1, x / 2, y / 2) = min_value;
		}
	}
}

// ͼ���˲�����
void blur(image_t* img0, image_t* img1, uint32_t kernel) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(img0 != img1 && img0->data != img1->data);

	// �ȱ���y�����x�Ƚ�cache-friendly
	for (int y = 1; y < img0->height - 1; y++) {
		for (int x = 1; x < img0->width - 1; x++) {
			AT(img1, x, y) = (1 * AT(img0, x - 1, y - 1) + 2 * AT(img0, x, y - 1) + 1 * AT(img0, x + 1, y - 1) +
				2 * AT(img0, x - 1, y) + 4 * AT(img0, x, y) + 2 * AT(img0, x + 1, y) +
				1 * AT(img0, x - 1, y + 1) + 2 * AT(img0, x, y + 1) + 1 * AT(img0, x + 1, y + 1)) / 16;
		}
	}
}

// 3x3sobel��Ե��ȡ
void sobel3(image_t* img0, image_t* img1){
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(img0 != img1 && img0->data != img1->data);

	int gx, gy;
	// �ȱ���y�����x�Ƚ�cache-friendly    
	for (int y = 1; y < img0->height - 1; y++) {
		for (int x = 1; x < img0->width - 1; x++) {
			gx = (-1 * AT(img0, x - 1, y - 1) + 1 * AT(img0, x + 1, y - 1) +
				-2 * AT(img0, x - 1, y) + 2 * AT(img0, x + 1, y) +
				-1 * AT(img0, x - 1, y + 1) + 1 * AT(img0, x + 1, y + 1)) / 4;
			gy = (1 * AT(img0, x - 1, y - 1) + 2 * AT(img0, x, y - 1) + 1 * AT(img0, x + 1, y - 1) +
				-1 * AT(img0, x - 1, y + 1) - 2 * AT(img0, x, y + 1) - 1 * AT(img0, x + 1, y + 1)) / 4;
			AT(img1, x, y) = (abs(gx) + abs(gy)) / 2;
		}
	}
}

// 3x3��ʴ
void erode3(image_t* img0, image_t* img1) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(img0 != img1 && img0->data != img1->data);

	int min_value;
	// �ȱ���y�����x�Ƚ�cache-friendly    
	for (int y = 1; y < img0->height - 1; y++) {
		for (int x = 1; x < img0->width - 1; x++) {
			min_value = 255;
			for (int dy = -1; dy <= 1; dy++) {
				for (int dx = -1; dx <= 1; dx++) {
					if (AT(img0, x + dx, y + dy) < min_value) min_value = AT(img0, x + dx, y + dy);
				}
			}
			AT(img1, x, y) = min_value;
		}
	}
}

// 3x3����
void dilate3(image_t* img0, image_t* img1) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(img0 != img1 && img0->data != img1->data);

	int max_value;
	// �ȱ���y�����x�Ƚ�cache-friendly    
	for (int y = 1; y < img0->height - 1; y++) {
		for (int x = 1; x < img0->width - 1; x++) {
			max_value = 0;
			for (int dy = -1; dy <= 1; dy++) {
				for (int dx = -1; dx <= 1; dx++) {
					if (AT(img0, x + dx, y + dy) > max_value) max_value = AT(img0, x + dx, y + dy);
				}
			}
			AT(img1, x, y) = max_value;
		}
	}
}

//
void remap(image_t* img0, image_t* img1, fimage_t* mapx, fimage_t* mapy) {
	assert(img0 && img0->data);
	assert(img1 && img1->data);
	assert(mapx && mapx->data);
	assert(mapy && mapy->data);
	assert(img0 != img1 && img0->data != img1->data);
	assert(img0->width == img1->width && img0->height == img1->height);
	assert(mapx->width == mapy->width && mapx->height == mapy->height);
	assert(img0->width == mapx->width && img0->height == mapx->height);

	// �ȱ���y�����x�Ƚ�cache-friendly    
	for (int y = 1; y < img0->height - 1; y++) {
		for (int x = 1; x < img0->width - 1; x++) {
			AT(img1, x, y) = AT(img0, (int)(AT(mapx, x, y) + 0.5), (int)(AT(mapy, x, y) + 0.5));
		}
	}
}

/* ǰ�������壺
 *   0
 * 3   1
 *   2
 */const int dir_front[4][2]= { {-1,  0},
								{0,  1},
								{1,  0},
								{0, -1} };
/* ǰ�������壺
 * 0   1
 *      
 * 3   2
*/const int dir_frontleft[4][2] = { {-1, -1},
								{-1,  1},
								{1,  1},
								{1, -1} };
/* ǰ�������壺
 * 3   0
 *
 * 2   1
*/const int dir_frontright[4][2] = { {-1,  1},
								 {1,  1},
								{1, -1},
								 {-1, -1} };

// �����sobel�Զ���ֵ
uint8_t SobelAuto(image_t* img, int x, int y)
{
	/** ����˴�С */
	int KERNEL_SIZE = 3;
	int xStart = x-KERNEL_SIZE / 2;
	int xEnd = x+KERNEL_SIZE / 2;
	int yStart = y-KERNEL_SIZE / 2;
	int yEnd = y+KERNEL_SIZE / 2;
	int i, j, k;
	int temp[4];
	for (i = yStart; i <= yEnd; i++)
	{
		for (j = xStart; j <= xEnd; j++)
		{
			printf("1=%d", yStart);
			/* ���㲻ͬ�����ݶȷ�ֵ  */
			temp[0] = -(int)AT(img, i - 1, j - 1) + (int)AT(img, i - 1, j + 1)    // -1,  0,  1
				- (int)AT(img, i, j - 1) + (int)AT(img, i, j + 1)                  // -1,  0,  1
				- (int)AT(img, i + 1, j - 1) + (int)AT(img, i + 1, j + 1);         // -1,  0,  1
			/* 0 deg  */
			printf("2=%d", yEnd);
			temp[1] = -(int)AT(img, i - 1, j - 1) + (int)AT(img, i + 1, j - 1)     // -1, -1, -1
				- (int)AT(img, i - 1, j) + (int)AT(img, i + 1, j)                  //  0,  0,  0
				- (int)AT(img, i - 1, j + 1) + (int)AT(img, i + 1, j + 1);         //  1,  1,  1

			/* 45 deg  */
			printf("3=%d", xStart);
			temp[2] = -(int)AT(img, i - 1, j) + (int)AT(img, i, j - 1)			   //  0, -1, -1
				- (int)AT(img, i, j + 1) + (int)AT(img, i + 1, j)				   //  1,  0, -1
				- (int)AT(img, i - 1, j + 1) + (int)AT(img, i + 1, j - 1);		   //  1,  1,  0
			/* 135 deg  */
			printf("4=%d", xEnd);
			temp[3] = -(int)AT(img, i - 1, j) + (int)AT(img, i, j + 1)             // -1, -1,  0
				- (int)AT(img, i, j - 1) + (int)AT(img, i + 1, j)                 // -1,  0,  1
				- (int)AT(img, i - 1, j - 1) + (int)AT(img, i + 1, j + 1);         //  0,  1,  1

			temp[0] = abs(temp[0]);
			temp[1] = abs(temp[1]);
			temp[2] = abs(temp[2]);
			temp[3] = abs(temp[3]);

			/* �ҳ��ݶȷ�ֵ���ֵ  */
			for (k = 1; k < 4; k++)
			{
				if (temp[0] < temp[k])
				{
					temp[0] = temp[k];
				}
			}

			/* ʹ�����ص����������ص�֮�͵�һ������    ��Ϊ��ֵ  */
			temp[3] = (int)AT(img, i - 1, j - 1) + (int)AT(img, i - 1, j) + (int)AT(img, i - 1, j + 1)
				+ (int)AT(img, i, j - 1) + (int)AT(img, i, j) + (int)AT(img, i, j + 1)
				+ (int)AT(img, i + 1, j - 1) + (int)AT(img, i + 1, j) + (int)AT(img, i + 1, j + 1);

			if (temp[0] > temp[3] / 10.0f)
			{
				return 255;
			}
			else
			{
				return 0;
			}
		}
	}
}



// �����Թ�Ѳ��  //y=115,x=30
void findline_lefthand_adaptive(image_t* img, int block_size, int clip_value, int x, int y, int pts[][2], int* num) {
	assert(img && img->data);
	assert(num && *num >= 0);
	assert(block_size > 1 && block_size % 2 == 1);
	int half = block_size / 2;    //half=3
	int step = 0, dir = 0, turn = 0,dd=0;
	while (step < *num && half < y && y < img->width - half - 1 && half < x && x < img->height - half - 1 && turn < 4) {   //3 < 115 < 188-4   3 < 30 < 120-4
		int local_thres = 0;
		//printf("(x=%d,y=%d)", y, x);
		// ����7*7�ڵ�ƽ����ֵ
		//for (int dy = -half; dy <= half; dy++) {
		//	for (int dx = -half; dx <= half; dx++) {
		//		local_thres += AT(img, y + dy, x + dx);
		//	}
		//}
		//local_thres /= block_size * block_size;
		//local_thres -= clip_value;
		

		int current_value = AT(img, y, x);
		int front_value = AT(img, y + dir_front[dir][0], x + dir_front[dir][1]);
		int frontleft_value = AT(img, y + dir_frontleft[dir][0], x + dir_frontleft[dir][1]);
		//printf("  local_th=%d  front_value = %d frontleft_value =%d\n", local_thres, front_value, frontleft_value);

		//if (SobelAuto(img, y + dir_front[dir][0], x + dir_front[dir][1])==0) 
		//if (front_value<local_thres)
		if (front_value ==255)
		{
			dir = (dir + 1) % 4;
			turn++;
		}
		//else if (SobelAuto(img, y + dir_frontleft[dir][0], x + dir_frontleft[dir][1])==0)
		//else if (frontleft_value<local_thres)
		else if (frontleft_value ==255)
		{
			y += dir_front[dir][0];
			x += dir_front[dir][1];
			pts[step][0] = y;
			pts[step][1] = x;
			step++;
			turn = 0;
		}
		else
		{
			y += dir_frontleft[dir][0];
			x += dir_frontleft[dir][1];
			dir = (dir + 3) % 4;
			pts[step][0] = y;
			pts[step][1] = x;
			step++;
			turn = 0;
		}
		//printf("(x1=%d,y1=%d)", pts[dd][0], pts[dd][1]);
		//dd = dd + 1;
	}
	*num = step;
}

// �����Թ�Ѳ��   //y=115,x=164
void findline_righthand_adaptive(image_t* img, int block_size, int clip_value, int x, int y, int pts[][2], int* num) {
	assert(img && img->data);
	assert(num && *num >= 0);
	assert(block_size > 1 && block_size % 2 == 1);
	int half = block_size / 2;
	int step = 0, dir = 0, turn = 0;
	while (step < *num && 0 < x && x < img->width - 1 && 0 < y && y < img->height - 1 && turn < 4) {  //0 < 164 < 188-1  0 < 115 < 120-1
		//printf("  x=%d  y=%d", x, y);
		int local_thres = 0;
		for (int dy = -half; dy <= half; dy++) {
			for (int dx = -half; dx <= half; dx++) {
				local_thres += AT(img, y + dy, x + dx);
			}
		}
		local_thres /= block_size * block_size;
		local_thres -= clip_value;
		//printf("m");
		int current_value = AT(img, y, x);
		int front_value = AT(img, y + dir_front[dir][0], x + dir_front[dir][1]);
		int frontright_value = AT(img, y + dir_frontright[dir][0], x + dir_frontright[dir][1]);
		//printf("  local_th=%d  front_value = %d frontleft_value =%d\n", local_thres, front_value, frontright_value);
		if (front_value < local_thres) {
			dir = (dir + 3) % 4;
			turn++;
		}
		else if (frontright_value < local_thres) {
			//printf("d");
			y += dir_front[dir][0];
			x += dir_front[dir][1];
			pts[step][0] = y;
			pts[step][1] = x;
			step++;
			turn = 0;
		}
		else {
			y += dir_frontright[dir][0];
			x += dir_frontright[dir][1];
			dir = (dir + 1) % 4;
			pts[step][0] = y;
			pts[step][1] = x;
			step++;
			turn = 0;
		}
	}
	*num = step;
}

// ���߶ν���
void approx_lines(int pts[][2], int pts_num, float epsilon, int lines[][2], int* lines_num) {
	assert(pts);
	assert(epsilon > 0);

	int dx = pts[pts_num - 1][0] - pts[0][0];
	int dy = pts[pts_num - 1][1] - pts[0][1];
	float nx = -dy / sqrtf(dx * dx + dy * dy);
	float ny = dx / sqrtf(dx * dx + dy * dy);
	float max_dist = 0, dist;
	int idx = -1;
	for (int i = 1; i < pts_num - 1; i++) {
		dist = fabs((pts[i][0] - pts[0][0]) * nx + (pts[i][1] - pts[0][1]) * ny);
		if (dist > max_dist) {
			max_dist = dist;
			idx = i;
		}
	}
	if (max_dist >= epsilon) {
		int num1 = *lines_num;
		approx_lines(pts, idx + 1, epsilon, lines, &num1);
		int num2 = *lines_num - num1 - 1;
		approx_lines(pts + idx, pts_num - idx, epsilon, lines + num1 - 1, &num2);
		*lines_num = num1 + num2 - 1;
	}
	else {
		lines[0][0] = pts[0][0];
		lines[0][1] = pts[0][1];
		lines[1][0] = pts[pts_num - 1][0];
		lines[1][1] = pts[pts_num - 1][1];
		*lines_num = 2;
	}
}

// float���͵����߶ν���
void approx_lines_f(float pts[][2], int pts_num, float epsilon, float lines[][2], int* lines_num){
	assert(pts);
	assert(epsilon > 0);

	int dx = pts[pts_num - 1][0] - pts[0][0];
	int dy = pts[pts_num - 1][1] - pts[0][1];
	float nx = -dy / sqrtf(dx * dx + dy * dy);
	float ny = dx / sqrtf(dx * dx + dy * dy);
	float max_dist = 0, dist;
	int idx = -1;
	for (int i = 1; i < pts_num - 1; i++) {
		dist = fabs((pts[i][0] - pts[0][0]) * nx + (pts[i][1] - pts[0][1]) * ny);
		if (dist > max_dist) {
			max_dist = dist;
			idx = i;
		}
	}
	if (max_dist >= epsilon && *lines_num > 2) {
		int num1 = *lines_num;
		approx_lines_f(pts, idx + 1, epsilon, lines, &num1);
		int num2 = *lines_num - num1 - 1;
		approx_lines_f(pts + idx, pts_num - idx, epsilon, lines + num1 - 1, &num2);
		*lines_num = num1 + num2 - 1;
	}
	else {
		lines[0][0] = pts[0][0];
		lines[0][1] = pts[0][1];
		lines[1][0] = pts[pts_num - 1][0];
		lines[1][1] = pts[pts_num - 1][1];
		*lines_num = 2;
	}
}

void draw_line(image_t* img, int pt0[2], int pt1[2], uint8_t value) {
	int dx = pt1[0] - pt0[0];
	int dy = pt1[1] - pt0[1];
	if (abs(dx) > abs(dy)) {
		for (int x = pt0[0]; x != pt1[0]; x += (dx > 0 ? 1 : -1)) {
			int y = pt0[1] + (x - pt0[0]) * dy / dx;
			AT(img, clip(x, 0, img->width - 1), clip(y, 0, img->height - 1)) = value;
		}
	}
	else {
		for (int y = pt0[1]; y != pt1[1]; y += (dy > 0 ? 1 : -1)) {
			int x = pt0[0] + (y - pt0[1]) * dx / dy;
			AT(img, clip(x, 0, img->width - 1), clip(y, 0, img->height - 1)) = value;
		}
	}
}

// ��������ֵ
uint16_t getOSTUThreshold(image_t* img, uint8_t MinThreshold, uint8_t MaxThreshold){
	uint8_t Histogram[256];
	uint16_t OUSTThreshold = 0;
	uint32_t PixelAmount = 0, Value_Sum = 0;
	uint64_t sigma = 0, maxSigma = 0;
	float w1 = 0, w2 = 0;
	int32_t u1 = 0, u2 = 0;
	uint8_t MinValue = 0, MaxValue = 255;

	//�����ص����
	uint8_t* ptr = img->data;
	uint8_t* ptrEnd = img->data + img->width * img->height;
	while (ptr != ptrEnd) {
		++Histogram[*ptr++];
	}

	for (uint8_t m = 0; m < 100; m++) {

		Histogram[m] = 0;
	}

	for (MinValue = 0; Histogram[MinValue] == 0 && MinValue < 255; ++MinValue);
	for (MaxValue = 255; Histogram[MaxValue] == 0 && MaxValue > 0; --MaxValue);

	if (MaxValue == MinValue) return MaxValue;         // ֻ��һ����ɫ
	if (MinValue + 1 == MaxValue) return MinValue;        // ֻ�ж�����ɫ

	if (MinValue < MinThreshold) {
		MinValue = MinThreshold;
	}
	if (MaxValue > MaxThreshold) {
		MaxValue = MaxThreshold;
	}

	uint32_t Pixel_Integral[256] = { 0 };   //���ػ��� 
	uint32_t Value_Integral[256] = { 0 };    //�ҶȻ���
	for (uint8_t i = MinValue; i <= MaxValue; ++i) {
		PixelAmount += Histogram[i];      //��������
		Value_Sum += Histogram[i] * i;     //�Ҷ��ܺ�
		Pixel_Integral[i] = PixelAmount;
		Value_Integral[i] = Value_Sum;
	}
	for (uint8_t i = MinValue; i < MaxValue + 1; ++i) {
		w1 = (float)Pixel_Integral[i] / PixelAmount;  //ǰ�����ص����
		w2 = 1 - w1;                               //��������
		u1 = (int32_t)(Value_Integral[i] / w1);                   //ǰ��ƽ���Ҷ�
		u2 = (int32_t)((Value_Sum - Value_Integral[i]) / w2);      //����ƽ���Ҷ�
		sigma = (uint64_t)(w1 * w2 * (u1 - u2) * (u1 - u2));
		if (sigma >= maxSigma) {
			maxSigma = sigma;
			OUSTThreshold = i;
		}
		else {
			break;
		}
	}
	return OUSTThreshold;
}

// �㼯�����˲�
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) {
	assert(kernel % 2 == 1);
	int half = kernel / 2;
	for (int i = 0; i < num; i++) {
		pts_out[i][0] = pts_out[i][1] = 0;
		for (int j = -half; j <= half; j++) {
			pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
			pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
		}
		pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
		pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
	}
}

// �㼯�Ⱦ����  ʹ�߹��Ĳ���ǰ���߶εľ���Ϊ`dist`
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int* num2, float dist) {
	int remain = 0, len = 0;
	for (int i = 0; i < num1 - 1 && len < *num2; i++) {
		float x0 = pts_in[i][0];
		float y0 = pts_in[i][1];
		float dx = pts_in[i + 1][0] - x0;
		float dy = pts_in[i + 1][1] - y0;
		float dn = sqrt(dx * dx + dy * dy);
		dx /= dn;
		dy /= dn;

		while (remain < dn && len < *num2) {
			x0 += dx * remain;
			pts_out[len][0] = x0;
			y0 += dy * remain;
			pts_out[len][1] = y0;

			len++;
			dn -= remain;
			remain = dist;
		}
		remain -= dn;
	}
	*num2 = len;
}

// �㼯�Ⱦ����2  ʹ����������ľ���Ϊ`dist`
// TODO: fix bug
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int* num2, float dist) {
	if (num1 < 0) {
		*num2 = 0;
		return;
	}
	pts_out[0][0] = pts_in[0][0];
	pts_out[0][1] = pts_in[0][1];
	int len = 1;
	for (int i = 0; i < num1 - 1 && len < *num2; i++) {
		float x0 = pts_in[i][0];
		float y0 = pts_in[i][1];
		float x1 = pts_in[i + 1][0];
		float y1 = pts_in[i + 1][1];

		do {
			float x = pts_out[len - 1][0];
			float y = pts_out[len - 1][1];

			float dx0 = x0 - x;
			float dy0 = y0 - y;
			float dx1 = x1 - x;
			float dy1 = y1 - y;

			float dist0 = sqrt(dx0 * dx0 + dy0 * dy0);
			float dist1 = sqrt(dx1 * dx1 + dy1 * dy1);

			float r0 = (dist1 - dist) / (dist1 - dist0);
			float r1 = 1 - r0;

			if (r0 < 0 || r1 < 0) break;
			x0 = x0 * r0 + x1 * r1;
			y0 = y0 * r0 + y1 * r1;
			pts_out[len][0] = x0;
			pts_out[len][1] = y0;
			len++;
		} while (len < *num2);

	}
	*num2 = len;
}

// �㼯�ֲ��Ƕȱ仯��
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist) {
	for (int i = 0; i < num; i++) {
		if (i <= 0 || i >= num - 1) {
			angle_out[i] = 0;
			continue;
		}
		float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
		float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
		float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
		float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
		float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
		float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
		float c1 = dx1 / dn1;
		float s1 = dy1 / dn1;
		float c2 = dx2 / dn2;
		float s2 = dy2 / dn2;
		angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
	}
}

// �Ƕȱ仯�ʷǼ�������
void nms_angle(float angle_in[], int num, float angle_out[], int kernel) {
	assert(kernel % 2 == 1);
	int half = kernel / 2;
	for (int i = 0; i < num; i++) {
		angle_out[i] = angle_in[i];
		for (int j = -half; j <= half; j++) {
			if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
				angle_out[i] = 0;
				break;
			}
		}
	}
}

// ����߸�������
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) {
	for (int i = 0; i < num; i++) {
		float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
		float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
		float dn = sqrt(dx * dx + dy * dy);
		dx /= dn;
		dy /= dn;
		pts_out[i][0] = pts_in[i][0] - dy * dist;
		pts_out[i][1] = pts_in[i][1] + dx * dist;
	}
}

// �ұ��߸�������
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) {
	for (int i = 0; i < num; i++) {
		float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] - pts_in[clip(i - approx_num, 0, num - 1)][0];
		float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] - pts_in[clip(i - approx_num, 0, num - 1)][1];
		float dn = sqrt(dx * dx + dy * dy);
		dx /= dn;
		dy /= dn;
		pts_out[i][0] = pts_in[i][0] + dy * dist;
		pts_out[i][1] = pts_in[i][1] - dx * dist;
	}
}

void draw_x(image_t* img, int x, int y, int len, uint8_t value) {
	for (int i = -len; i <= len; i++) {
		AT(img, clip(x + i, 0, img->width - 1), clip(y + i, 0, img->height - 1)) = value;
		AT(img, clip(x - i, 0, img->width - 1), clip(y + i, 0, img->height - 1)) = value;
	}
}

void draw_o(image_t* img, int x, int y, int radius, uint8_t value) {
	for (float i = - M_PI; i <= M_PI; i += M_PI / 10) {
		AT(img, clip(x + radius * cosf(i), 0, img->width - 1), clip(y + radius * sinf(i), 0, img->height - 1)) = value;
	}
}

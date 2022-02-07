#include "stdafx.h"
#include<stdio.h>
#include "test.h"
#include "common.h"
#include "sensor_process.h"

/*!
  * @file     imageprocess.cpp
  * @brief    ͼ����
  * @author   YYY
  * @version  V1.2
  * @date     2022/2/7
  */


/**  @brief    ������  */
#define ROAD_MAIN_ROW      40

/**  @brief    ʹ����ʼ��120  */
#define ROAD_START_ROW     119

/**  @brief    ʹ�ý�����10  */
#define ROAD_END_ROW       1

#define turn_big           40  //����������ֵ���40�ж�Ϊǰ���д���


/*******************�����ǻ���ʶ�����õ������ȫ�ֱ��� �ȷ�����***************/
/********************�����ǻ���ʶ�����õ�����ر����ȷ�����*********************/
uint8 Island_flag = 0;
uint8 Big_island_flag = 0;
int16 Fir_jump_point_row;
int16 Fir_jump_point_column;
int16 Sec_jump_point_row;
int16 Sec_jump_point_column;

int16 Island_change_time = 400;//����״̬�л���ʱ 400ms �ٶȴ�ʱ��С �ٶ�Сʱ�Ӵ�

int16 Island_time;
int16 meet_time;

/**********************�����ǻᳵ��ʶ�����õ�����ر���************************/
uint8 meet_flag = 0;
uint8 Left_car_flag_mid;
int16 Start_time;
uint8 Left_car_flag_end;
int16 Stop_ready2;


uint8 tesssss[ROW][COL];

/********************���µ�ȫ�ֱ��������ڿ��Ż�Ϊ�ֲ�����*********************/



/*************************����Ϊ����Ѳ�������ȫ�ֱ���*************************/
coordinate Left_line; //����������� ö�� �������������
coordinate Right_line;//�ұ��������� ö�� ���ұ���������
coordinate Midd_line; //����������   ö�� ������������
struct PID Steer;// PID PDö�� ���������
uint8 binary_img[ROW][COL];//��ֵ�����ͼ������
uint8 image[ROW][COL];
uint8 Left_line_lost[70]; //�洢��Ӧ������ߵ� ���������1 Ϊ����δ�ҵ�                                                //40
uint8 Right_line_lost[70];//�洢��Ӧ���ұ��ߵ� ���������1 Ϊ����δ�ҵ�                                                //40
uint8 Found_left_flag, Found_right_flag;//�������ұ��ߺڵ�Ѱ�ұ�־λ 1Ϊ�ҵ�
uint8 Near_flag;//���������־ С��ǰ�˳�ͷ�����ٽ������߽� ����ȫ����(����һƬ��)
uint8 Cross_road_flag;//ʮ�� ��־λ
uint8 out_flag;//�����־λ
uint8 Highest_speed_flag, Mult_speed_flag, Lowest_speed_flag;//С������ ���� ��־λ
uint8 Ram_flag = 0;//�µ���־λ
uint8 Thre = 130;
int16 L_lost_cnt = 0, R_lost_cnt = 0;//���Ҷ������ߣ��㣩����  
int16 L_R_lost_cnt, Cross_road_cnt;//����ͬʱ�����ߣ��㣩����  ʮ�ֶ��ߴ������б�˫�߶��ߴ���
int16 Row_begging = 63, Row_end = 0, Last_row_end, Column_end;//ɨ�� ��ʼ�� ������  ��һ��ɨ�߽����� �����У���ʼ�У�   //63
int16 L_last_lost_row = 63, R_last_lost_row = 63;//���ұ���󶪵��ߵ�����                                           //63
int16 L_R_lost_row = 63;//ʮ������                                                                                   //63
int16 Last_mid_line = 93;//���� �е������
int16 Vertical_longest_length_1;//�����������ȣ���Զ�ˣ�
int16 old_error;//�ϴ�����ƫ��
int16 row = 63;//����                                                                                               //63
int16 Row_start = 63;//ɨ����ʼ��;                                                                                   //63
int16 Last_L_column;//��һ��ͼ��ĳһ�е���߽����� ��ʱδ�õ�
int16 Last_R_column;//��һ��ͼ��ĳһ�е��ұ߽����� ��ʱδ�õ�
int16 Zebra_first_row;

float Middle_line = 93;//ͼ������  �ɸĶ�
float Left_slope = 0; //�����  ����б��
float Right_slope = 0;//�ұ��� ����б��
float Last_left_slope = 0; //��һ��ͼ��������б��
float Last_right_slope = 0;//��һ��ͼ����ұ���б��

float Add_weight[60] = {
							2.3,2.3,2.3,2.3,2.3,2,2,2,2,2,
							2,2,1.7,1.7,1.7,1.7,1.5,1,1,1,
							1,1,1,1,1,1,1,1,1,1,

							1,1,1,1,1,1,1,1,1,1,//�����ʮ��  �����õ���������ʮ��
							1,1,1,1,1,1,1,1,1,1,

							1,1,1,1,1,1,1,1,1,1,

};//��Ȩ�㷨 Ȩ�� ��·��  ���ڿ��÷ֶμ�Ȩ 
float Single_L_line[35] = {
								78,76,75,73,71,69,68,66,64,62,
								60,58,57,55,53,52,50,49,47,46,
								44,42,41,39,38,37,35,63,61,31,
								29,28,27,25,24,
};
/*float Single_R_line[35] = {
								90,92,94,96,99,100,102,104,105,107,
								109,110,112,114,115,117,118,120,122,123,
								125,126,128,129,130,131,133,135,136,137,
								139,140,141,143,144,
						   };*/
float Single_R_line[35] = {
								91,92,94,95,96,97,99,101,103,105,
								107,110,111,112,114,116,117,118,120,122,
								123,125,126,128,129,131,132,163,135,137,
								138,140,141,142,144,
};


/**  @brief    ������־λ  */
uint8_t g_ucFlagRoundabout = 0;

/**  @brief    ʮ�ֱ�־λ  */
uint8_t g_ucFlagCross = 0;

/**  @brief    �����߱�־λ  */
uint8_t g_ucFlagZebra = 0;

const motorbike motor_contorl = {
	0.189,      //���־� m
	0.03375,    //���ְ뾶 m
	0.07,       //���ĸ߶� m
	0.09425,    //����ͶӰ��P1���� m
	0.7,        //�������� kg
	0.5,        //�ܵ���·��ת��뾶 m
	9.794,      //�������ٶ� m/s^2
};

int add(int num1, int num2);

void main()
{
	int a = 2;
	int c = 0;
	c = add(a, c);
	printf("%d", c);
}

/**
 * @brief ���������
 */
const xlwParamStructure paramTrace = {
	5,          /* ��ʶ��Ϊ�ű�Ƶ���С���ظ��� */
	28,         /* ���������ص��Ϸ�����ĸ߶� */
	1,			/* ������ʧĿ�굼���ж�Ϊ��ʧ�ű�ƵĴ������� */
	60,			/* �������ٵ�����ű�Ƴߴ磨��Զ���������٣� */
	80,			/* ��һ�����ٵ�����ű�Ƴߴ� */
	100,		/* �ڶ���... */
	500,        /* �������� */
	450,        /* ��һ������ */
	300,        /* �ڶ������� */
	200,        /* ���������� */
	10.3f,      /* ������ѹ��ֵ */
	-50,		/* Ѱ���ٶ���ֵ */
	200,		/* Ѱ���ٶ���ֵ */
	0          /* ��ֵ����ֵ */
};

//ѹ����ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
//srclen �Ƕ�ֵ��ͼ���ռ�ÿռ��С
//��ӥ�۽�ѹ��ӥ��ͼ���ѹ��תΪ ��ά���� - ���ܳ������� - ɽ����̳ http://vcan123.com/forum.php?mod=viewthread&tid=17&ctid=6
//��ѹ��ʱ�������и����飬���úڡ��׶�Ӧ��ֵ�Ƕ��١�
/*void img_extract(void *dst, void *src, uint32_t srclen)
{
	uint8_t colour[2] = {1, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
	uint8_t * mdst = dst;
	uint8_t * msrc = src;
	//ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
	uint8_t tmpsrc;
	while(srclen --)
	{
		tmpsrc = *msrc++;
		*mdst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
		*mdst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
	}
}*/
void img_extract(uint8* img, uint8* buff, uint32 length)    //��ѹ�󣬽�ѹǰ��ԭ����
{
	uint8 colour[2] = { 255, 0 };   //��,��
	uint8 temp;

	while (length--)
	{
		temp = *buff++;
		*img++ = colour[(temp >> 7) & 0x01];
		*img++ = colour[(temp >> 6) & 0x01];
		*img++ = colour[(temp >> 5) & 0x01];
		*img++ = colour[(temp >> 4) & 0x01];
		*img++ = colour[(temp >> 3) & 0x01];
		*img++ = colour[(temp >> 2) & 0x01];
		*img++ = colour[(temp >> 1) & 0x01];
		*img++ = colour[(temp >> 0) & 0x01];
	}
}

//motor[0]:���ת�ǣ�motor[1]:�������
float* control(float omega, uint8_t cha,float motor[2], uint8_t mode)
{
	if (mode == 0)  //ֱ��ģʽ�����ת���������в�ֵ���������Ϊ0
	{
		motor[0] = atan(cha / 1.5 / ROAD_MAIN_ROW) * 180.0 / 3.1416;
		motor[1] = atan(cha / 2.0 / ROAD_MAIN_ROW) * 180.0 / 3.1416;
	}
	if (mode == 1)  //��⵽�����Ĵ���ʽ
	{
		float op1 = 0.0, Va = 0.0;
		op1 = sqrt(motor_contorl.R * motor_contorl.R - motor_contorl.b * motor_contorl.b);
		Va = motor_contorl.R * omega;  //Va=V ?
		motor[0] = atan(motor_contorl.w / op1) * 180.0 / 3.1416;
		motor[1] = atan(motor_contorl.R * motor_contorl.g / (Va * Va)) * 180.0 / 3.1416;
	}
	if (mode == 2)  //����ģʽ�������в�ֵ����һ������ʵ��
	{
		float op1 = 0.0, Va = 0.0;
		op1 = sqrt(motor_contorl.R * motor_contorl.R - motor_contorl.b * motor_contorl.b);
		Va = motor_contorl.R * omega;  //Va=V ?
		motor[0] = atan(motor_contorl.w / op1) * 180.0 / 3.1416;
		motor[1] = atan(motor_contorl.R * motor_contorl.g / (Va * Va)) * 180.0 / 3.1416;
		//�˳�������ͬ
	}
	if (mode == 3)  //���ģʽ
	{
		motor[0] = 30;  //���ֵ
		motor[1] = 20;  
	}
	return motor;
}


void hhh(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t img[MT9V03X_H][MT9V03X_W], uint8_t th)
{
	for (int i = 0; i < MT9V03X_H; i++)
	{
		for (int j = 0; j < MT9V03X_W; j++)
		{
			img[i][j] = image[i][j];
		}
	}
	for (int i = 0; i < MT9V03X_H; i++)
	{
		for (int j = 0; j < MT9V03X_W; j++)
		{
			if(img[i][j]>th)
				img[i][j] = 255;
			else
				img[i][j] = 0;
		}
	}
}

/**
 * @brief OTSU��򷨶�ֵ��������ֵ
 * @param image (uint8_t*)ͼ����ʼָ��
 * @param col   (uint16_t)����
 * @param row   (uint16_t)����
 * @return (uint8_t)����ó�����ֵ
 */
uint8_t XLW_otsuThreshold(uint8_t image[MT9V03X_H][MT9V03X_W], uint16_t col, uint16_t row)
{
	/* ����Ҷȵ����ȼ���128�� */
#define GrayScale 128
	static uint32_t pixelCount[GrayScale];
	static float pixelPro[GrayScale];
	uint16_t th_max = 130, th_min;
	th_min = paramTrace.OTSUth_min;
	//th_min = 999;
	uint16_t width = col;
	uint16_t height = row;
	uint32_t i, j, pixelSum = width * height;
	uint8_t threshold = 0;
	uint8_t* data = image[0];
	for (i = 0; i < GrayScale; i++)
	{
		pixelCount[i] = 0;
		pixelPro[i] = 0;
	}

	//ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
	for (i = paramTrace.upperCutLine; i < height; i += 2)
	{
		for (j = 0; j < width; j += 2)
		{
			pixelCount[(int)data[i * width + j] / 2]++; //������ֵ��Ϊ����������±�
		}
	}

	//����ÿ������������ͼ���еı���
	float maxPro = 0.0;
	for (i = 0; i < GrayScale; i++)
	{
		pixelPro[i] = (float)pixelCount[i] / pixelSum;
		if (pixelPro[i] > maxPro)
		{
			maxPro = pixelPro[i];
		}
	}

	//�����Ҷȼ�[0,255]
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (i = 0; i < GrayScale; i++) // i��Ϊ��ֵ
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
		for (j = 0; j < GrayScale; j++)
		{
			if (j <= i) //��������
			{
				w0 += pixelPro[j];
				u0tmp += j * pixelPro[j];
			}
			else //ǰ������
			{
				w1 += pixelPro[j];
				u1tmp += j * pixelPro[j];
			}
		}
		u0 = u0tmp / w0;
		u1 = u1tmp / w1;
		u = u0tmp + u1tmp;
		deltaTmp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u);
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = (uint8_t)i;
		}
	}
	threshold -= 1;
	if (threshold * 2 > th_max)
		threshold = th_max / 2;
	if (threshold * 2 < th_min)
		threshold = th_min / 2;

	return threshold * 2;
}


/*!
  * @brief    �������ֵ��С(����)
  * @param    tmImage �� ͼ������
  * @return   ��ֵ
 * @note     Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
  * @note     1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
  * @note     2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
  * @note     3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ����	1
  * @note     4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��ı���w0����ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����������) ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
  * @note     5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
  * @note     6) i++��ת��4)��ֱ��iΪ256ʱ��������
  * @note     7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
  * @note     ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
  * @note     ������ղ�����  https://blog.csdn.net/kk55guang2/article/details/78475414
  * @note     https://blog.csdn.net/kk55guang2/article/details/78490069
  * @note     https://wenku.baidu.com/view/84e5eb271a37f111f0855b2d.html
  * @see      GetOSTU(Image_Use);//�����ֵ
  * @date     2019/6/25 ���ڶ�
  */
int GetOSTU(unsigned char tmImage[MT9V03X_H][MT9V03X_W])
{
	signed short i, j;
	unsigned long Amount = 0;
	unsigned long PixelBack = 0;
	unsigned long PixelIntegralBack = 0;
	unsigned long PixelIntegral = 0;
	signed long PixelIntegralFore = 0;
	signed long PixelFore = 0;
	float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��; 
	signed short MinValue, MaxValue;
	signed short Threshold = 0;
	unsigned char HistoGram[256];              //

	for (j = 0; j < 256; j++)  HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ 

	for (j = 0; j < MT9V03X_H; j++)
	{
		for (i = 0; i < MT9V03X_W; i++)
		{
			HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
		}
	}

	for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
	for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

	if (MaxValue == MinValue)     return MaxValue;         // ͼ����ֻ��һ����ɫ    
	if (MinValue + 1 == MaxValue)  return MinValue;        // ͼ����ֻ�ж�����ɫ

	for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  ��������

	PixelIntegral = 0;
	for (j = MinValue; j <= MaxValue; j++)
	{
		PixelIntegral += HistoGram[j] * j;//�Ҷ�ֵ����
	}
	SigmaB = -1;
	for (j = MinValue; j < MaxValue; j++)
	{
		PixelBack = PixelBack + HistoGram[j];   //ǰ�����ص���
		PixelFore = Amount - PixelBack;         //�������ص���
		OmegaBack = (float)PixelBack / Amount;//ǰ�����ذٷֱ�
		OmegaFore = (float)PixelFore / Amount;//�������ذٷֱ�
		PixelIntegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
		MicroBack = (float)PixelIntegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
		MicroFore = (float)PixelIntegralFore / PixelFore;   //�����ҶȰٷֱ�
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//������䷽��
		if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
		{
			SigmaB = Sigma;
			Threshold = j;
		}
	}
	return Threshold;                        //���������ֵ;
}


/**
  * @brief    ����soble���ؼ�����ӵ�һ�ֱ��ؼ��
  * @param    imageIn    ��������
  * @param    imageOut   �������      ����Ķ�ֵ����ı�����Ϣ
  * @param    Threshold  ��ֵ
  * @return
  * @note
  * @date     2020/5/15
  */
void SobelThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W], uint8_t Threshold)
{
	/** ����˴�С */
	int KERNEL_SIZE = 3;
	int xStart = KERNEL_SIZE / 2;
	int xEnd = MT9V03X_W - KERNEL_SIZE / 2;
	int yStart = KERNEL_SIZE / 2;
	int yEnd = MT9V03X_H - KERNEL_SIZE / 2;
	int i, j, k;
	int temp[4];
	uint8_t imageOut[MT9V03X_H][MT9V03X_W];
	for (i = yStart; i < yEnd; i++)
	{
		for (j = xStart; j < xEnd; j++)
		{
			/* ���㲻ͬ�����ݶȷ�ֵ  */
			/* 90 deg  */
			temp[0] = -(int)imageIn[i - 1][j - 1] + (int)imageIn[i - 1][j + 1]     // -1,  0,  1
				- (int)imageIn[i][j - 1] + (int)imageIn[i][j + 1]                  // -1,  0,  1
				- (int)imageIn[i + 1][j - 1] + (int)imageIn[i + 1][j + 1];         // -1,  0,  1
			/* 0 deg  */
			temp[1] = -(int)imageIn[i - 1][j - 1] + (int)imageIn[i + 1][j - 1]     // -1, -1, -1
				- (int)imageIn[i - 1][j] + (int)imageIn[i + 1][j]                  //  0,  0,  0
				- (int)imageIn[i - 1][j + 1] + (int)imageIn[i + 1][j + 1];         //  1,  1,  1

			/* 45 deg  */
			temp[2] = -(int)imageIn[i - 1][j] + (int)imageIn[i][j - 1]			   //  0, -1, -1
				- (int)imageIn[i][j + 1] + (int)imageIn[i + 1][j]				   //  1,  0, -1
				- (int)imageIn[i - 1][j + 1] + (int)imageIn[i + 1][j - 1];		   //  1,  1,  0
			/* 135 deg  */
			temp[3] = -(int)imageIn[i - 1][j] + (int)imageIn[i][j + 1]             // -1, -1,  0
				- (int)imageIn[i][j - 1] + (int)imageIn[i + 1][j]                  // -1,  0,  1
				- (int)imageIn[i - 1][j - 1] + (int)imageIn[i + 1][j + 1];         //  0,  1,  1

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

			if (temp[0] > Threshold)
			{
				imageOut[i][j] = 255;
			}
			else
			{
				imageOut[i][j] = 1;
			}
		}
	}
	for (int i = 0; i < MT9V03X_H; i++)
	{
		for (int j = 0; j < MT9V03X_W; j++)
		{
			imageIn[i][j] = imageOut[i][j];
		}
	}
}

/**
  * @brief    ����soble���ؼ�����ӵ�һ���Զ���ֵ���ؼ��
  * @param    imageIn    ��������
  * @param    imageOut   �������      ����Ķ�ֵ����ı�����Ϣ
  * @return
  * @note
  * @date     2020/5/15
  */
void SobelAutoThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W])
{
	/** ����˴�С */
	int KERNEL_SIZE = 3;
	int xStart = KERNEL_SIZE / 2;
	int xEnd = MT9V03X_W - KERNEL_SIZE / 2;
	int yStart = KERNEL_SIZE / 2;
	int yEnd = MT9V03X_H - KERNEL_SIZE / 2;
	int i, j, k;
	int temp[4];
	uint8_t imageOut[MT9V03X_H][MT9V03X_W];
	for (i = yStart; i < yEnd; i++)
	{
		for (j = xStart; j < xEnd; j++)
		{
			/* ���㲻ͬ�����ݶȷ�ֵ  */
			/* 90 deg  */
			temp[0] = -(int)imageIn[i - 1][j - 1] + (int)imageIn[i - 1][j + 1]     // -1,  0,  1
				- (int)imageIn[i][j - 1] + (int)imageIn[i][j + 1]                  // -1,  0,  1
				- (int)imageIn[i + 1][j - 1] + (int)imageIn[i + 1][j + 1];         // -1,  0,  1
			/* 0 deg  */
			temp[1] = -(int)imageIn[i - 1][j - 1] + (int)imageIn[i + 1][j - 1]     // -1, -1, -1
				- (int)imageIn[i - 1][j] + (int)imageIn[i + 1][j]                  //  0,  0,  0
				- (int)imageIn[i - 1][j + 1] + (int)imageIn[i + 1][j + 1];         //  1,  1,  1

			/* 45 deg  */
			temp[2] = -(int)imageIn[i - 1][j] + (int)imageIn[i][j - 1]			   //  0, -1, -1
				- (int)imageIn[i][j + 1] + (int)imageIn[i + 1][j]				   //  1,  0, -1
				- (int)imageIn[i - 1][j + 1] + (int)imageIn[i + 1][j - 1];		   //  1,  1,  0
			/* 135 deg  */
			temp[3] = -(int)imageIn[i - 1][j] + (int)imageIn[i][j + 1]             // -1, -1,  0
				- (int)imageIn[i][j - 1] + (int)imageIn[i + 1][j]                  // -1,  0,  1
				- (int)imageIn[i - 1][j - 1] + (int)imageIn[i + 1][j + 1];         //  0,  1,  1

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
			temp[3] = (int)imageIn[i - 1][j - 1] + (int)imageIn[i - 1][j] + (int)imageIn[i - 1][j + 1]
				+ (int)imageIn[i][j - 1] + (int)imageIn[i][j] + (int)imageIn[i][j + 1]
				+ (int)imageIn[i + 1][j - 1] + (int)imageIn[i + 1][j] + (int)imageIn[i + 1][j + 1];

			if (temp[0] > temp[3] / 10.0f)
			{
				imageOut[i][j] = 255;
			}
			else
			{
				imageOut[i][j] = 1;
			}
		}
	}
	for (int i = 0; i < MT9V03X_H; i++)
	{
		for (int j = 0; j < MT9V03X_W; j++)
		{
			imageIn[i][j] = imageOut[i][j];
		}
	}
}


/*!
  * @brief    ��ֵ��
  * @param    mode ��0��ʹ����������ֵ,1��ʹ��ƽ����ֵ,2: sobel ���ӸĽ���-�ֶ���ֵ,3��sobel ���ӸĽ���-��̬��ֵ,4��OTSU
  * @return   ��
  * @note     ��
  * @see      Get_01_Value(0); //ʹ�ô�򷨶�ֵ��
  * @date     2020/5/15 ���ڶ�
  */
//uint8_t Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W],unsigned char mode)
//{
//	int i = 0, j = 0;
//	int Threshold = 0;
//	unsigned long  tv = 0;
//	char txt[16];
//
//	if (mode == 0)
//	{
//		Threshold = GetOSTU(image_in);//�����ֵ
//		return Threshold;
//	}
//	if (mode == 1)
//	{
//		//�ۼ�
//		for (i = 0; i < MT9V03X_H; i++)
//		{
//			for (j = 0; j < MT9V03X_W; j++)
//			{
//				tv += image_in[i][j];   //�ۼ�
//			}
//		}
//		Threshold = tv / MT9V03X_H / MT9V03X_W;        //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
//		Threshold = Threshold + 20;      //�˴���ֵ���ã����ݻ����Ĺ������趨
//		return Threshold;
//	}
//	else if (mode == 2)
//	{
//		Threshold = 50;
//		//�ֶ�������ֵ
//		SobelThreshold(image_in, (uint8_t)Threshold);
//		return 1;
//
//	}
//	else if (mode == 3)
//	{
//		SobelAutoThreshold(image_in);  //��̬������ֵ
//		return 1;
//	}
//	else if (mode == 4)
//	{
//		Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
//		//return Threshold ;
//	}
//	/* ��ֵ�� */
//	for (i = 0; i < MT9V03X_H; i++)
//	{
//		for (j = 0; j < MT9V03X_W; j++)
//		{
//			if (image_in[i][j] > Threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
//			{
//				image_in[i][j] = 255;
//			}
//			else
//			{
//				image_in[i][j] = 1;
//			}
//		}
//	}
//	return 1;
//}

// TODO
uint8_t* Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W], uint8_t image_out[MT9V03X_H*MT9V03X_W+1], unsigned char mode)
{
	int i = 0, j = 0;
	int Threshold = 0;
	unsigned long  tv = 0;
	char txt[16];

	if (mode == 0)
	{
		Threshold = GetOSTU(image_in);//�����ֵ
		//return Threshold;
	}
	if (mode == 1)
	{
		//�ۼ�
		for (i = 0; i < MT9V03X_H; i++)
		{
			for (j = 0; j < MT9V03X_W; j++)
			{
				tv += image_in[i][j];   //�ۼ�
			}
		}
		Threshold = tv / MT9V03X_H / MT9V03X_W;        //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
		Threshold = Threshold + 20;      //�˴���ֵ���ã����ݻ����Ĺ������趨
		//return Threshold;
	}
	else if (mode == 2)
	{
		Threshold = 50;
		//�ֶ�������ֵ
		SobelThreshold(image_in, (uint8_t)Threshold);
		//return 1;

	}
	else if (mode == 3)
	{
		SobelAutoThreshold(image_in);  //��̬������ֵ
		//return 1;
	}
	else if (mode == 4)
	{
		Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
		//return Threshold ;
	}
	/* ��ֵ�� */
	int m = 0;
	if (mode == 0 || mode == 1 || mode == 4) 
	{
	    for (i = 0; i < MT9V03X_H; i++)
		{
			for (j = i*188,m=0; j < (i+1)*188; j++,m++)
			{
				if (image_in[i][m] > Threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
				{
					//	//
					image_out[j] = 255; //printf("%d��", image_out[i][j]);
				}
				else
				{
					//
					image_out[j] = 1; //printf("%d��", image_out[i][j]);
				}
			}
			//printf("||||%d",i*188);
			//printf("\nTH:%d", Threshold);
		}
	}
	else
	{
		for (i = 0; i < MT9V03X_H; i++)
		{
			for (j = i * 188, m = 0; j < (i + 1) * 188; j++, m++)
			{
				image_out[j] = image_in[i][m]; //printf("%d��", image_out[i][j]);
			}
			//printf("||||%d",i*188);
			//printf("\nTH:%d", Threshold);
		}
	}
	image_out[MT9V03X_H * MT9V03X_W] = Threshold;
	//printf("\nTH:%d\n%d", Threshold, image_out[MT9V03X_H * MT9V03X_W]);

	return image_out;
}

int add(int num1, int num2)
{
	return num1 + num2;
}


/**********************************************************************************
Func    Name: ����ͼ�������� ���� ��ʼ��
Descriptions:  Midd_line.x[i]  Left_line.x[i]  Right_line.x[i]  �ֱ�Ϊͼ��������� �����ߵ�x����W������

Input   para: ~
In&Out  Para: ~
Output  para: ~
Return value: ~
***********************************************************************************/
void Line_coordinates_init()//��ʼ�����������ߵ�����  ���ʼ��Ϊ0  �ҳ�ʼ��Ϊ159  �г�ʼ��Ϊ79 
{
	int16 i;
	for (i = 0; i < 80; i++)
	{
		Left_line.x[i] = 0;
		Midd_line.x[i] = 93;
		Right_line.x[i] = 187;
		Left_line_lost[i] = 1;//��ʼ��Ϊ1  ��ʾδ����
		Right_line_lost[i] = 1;//��ʼ��Ϊ1 ��ʾδ����
	}
}
/***************************************************************
* �������ƣ�void GetHistGram(uint8 image[Row][Col])
* ����˵������ȡͼ��ĻҶ���Ϣ��ͳ��ͼ����ÿ���Ҷ�ֵ�ĸ���
* ����˵����
* �������룺Img_row_start, Img_row_end  Ҫ��ȡ��ͼ����Ϣ���У���Χ
* �������أ�void
* �޸�ʱ�䣺2018��5��30��
* �� ע��
***************************************************************/
int16 HistGram[256] = { 0 };
void GetHistGram(int16 Img_row_start, int16 Img_row_end)
{
	int16 X, Y;
	for (Y = 0; Y < 256; Y++)
	{
		HistGram[Y] = 0;//��ʼ���Ҷ�ֱ��ͼ
	}
	for (Y = Img_row_start; Y <= Img_row_end; Y++)
	{
		for (X = 0; X < COL; X++)
		{
			HistGram[image[Y][X]]++;//ͳ��ÿ���Ҷ�ֵ�ĸ���
		}
	}
}

/***************************************************************
*
*
* �������ƣ�uint8t OSTUThreshold()
* ����˵������򷨻�ȡͼ����ֵ
* ����˵����
* �������룺void
* �������أ�uint8t ��ֵ
* �޸�ʱ�䣺2018��5��30��
* �� ע��
***************************************************************/
uint8 OSTU_Threshold()
{
	uint16 Y;
	uint32 Amount = 0;
	uint32 PixelBack = 0;
	uint32 PixelIntegralBack = 0;
	uint32 PixelIntegral = 0;
	int32 PixelIntegralFore = 0;
	int32 PixelFore = 0;
	float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
	int16 MinValue, MaxValue;
	uint8 Threshold = 0;
	for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
	for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

	if (MaxValue == MinValue)
	{
		return MaxValue;          // ͼ����ֻ��һ����ɫ���Ҷ�ֵ��    
	}
	if (MinValue + 1 == MaxValue)
	{
		return MinValue;      // ͼ����ֻ�ж�����ɫ���Ҷ�ֵ��
	}

	for (Y = MinValue; Y <= MaxValue; Y++)
	{
		Amount += HistGram[Y];        //  ������������
	}

	PixelIntegral = 0;
	for (Y = MinValue; Y <= MaxValue; Y++)
	{
		PixelIntegral += HistGram[Y] * Y;//�Ҷ�ֵ����
	}
	SigmaB = -1;
	for (Y = MinValue; Y < MaxValue; Y++)
	{
		PixelBack = PixelBack + HistGram[Y];    //ǰ�����ص���
		PixelFore = Amount - PixelBack;         //�������ص���
		OmegaBack = (float)PixelBack / Amount;//ǰ�����ذٷֱ�
		OmegaFore = (float)PixelFore / Amount;//�������ذٷֱ�
		PixelIntegralBack += HistGram[Y] * Y;  //ǰ���Ҷ�ֵ
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
		MicroBack = (float)PixelIntegralBack / PixelBack;//ǰ���ҶȰٷֱ�
		MicroFore = (float)PixelIntegralFore / PixelFore;//�����ҶȰٷֱ�
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
		if (Sigma > SigmaB)//����������䷽��g
		{
			SigmaB = Sigma;
			Threshold = Y;
		}
	}
	return Threshold;
}
/**********************************************************************************
Func    Name: ͼ��̬��ֵ��ֵ��
Descriptions:

Input   para: ~
In&Out  Para: ~
Output  para: ~
Return value: ~
***********************************************************************************/
void img_binary(void)
{
	uint32 img_size;
	uint8* p1;
	uint8* p2;
	//uint8 th1;
	//GetHistGram(0,9);
	//uint8 th1 = OSTU_Threshold();//75;//66  Զ����0~6�У�ͼ�����ֵ
	//GetHistGram(10,63);
	//uint8 th2 = OSTU_Threshold();//75;//66  ������7~20�У�ͼ�����ֵ
   // GetHistGram(21,63);
	//uint8 th3 = OSTU_Threshold();//75;//66  ������21~63�У�ͼ�����ֵ
	//GetHistGram(0,59);
	//th1 = OSTU_Threshold();
	//uint8 th1 = 75;
	p1 = &image[0][0];
	p2 = &binary_img[0][0];
	img_size = ROW * COL;
	while (img_size)
	{
		img_size--;
		//*(p2+img_size) = (*(p1+img_size)>(img_size<3570?((img_size<1190?th1:th2)):th3))?0xff:0;
		//*(p2+img_size) = (*(p1+img_size)>(img_size<1700?th2:th1))?0xff:0;
		*(p2 + img_size) = (*(p1 + img_size)) > Thre ? 0xff : 0;

	}
}
/**********************************************************************************
Func    Name: ���ߡ�������ȡ
Descriptions:

Input   para: ~
In&Out  Para: ~
Output  para: ~
Return value: ~
***********************************************************************************/
uint8* Get_line_LMR(uint8 imagein[ROW][COL],uint8 side[ROW*3])
{
	uint8 Get_L_near_slope_flag = 0;//����ͼ���Ƿ��г��Խ��н�������б�ʵļ����ȡ  �Զ���ǰ�漸��Ϊ��׼��б��
	uint8 Get_R_near_slope_flag = 0;//����ͼ���Ƿ��г��Խ��н�������б�ʵļ����ȡ  �Զ���ǰ�漸��Ϊ��׼��б��
	uint8 Vertical_search_flag = 0;//����������־λ
	uint8 Fill_line_flag = 0;//�Ƿ���Խ��е��߻�ʮ�ֲ��߱�־λ  
	uint8 L_line_lost_flag = 0;//�� ���߱�־λ  ������¼���ȶ��ߵ���
	uint8 R_line_lost_flag = 0;//�� ���߱�־λ  ������¼���ȶ��ߵ���
	uint8 L_fir_line_flag = 0;//�� ���߱�־λ  ������¼�����ҵ�����
	uint8 R_fir_line_flag = 0;//�� ���߱�־λ  ������¼�����ҵ�����
	uint8 L_R_line_lost_flag = 0;//���ұ���ͬʱ���߱�־λ  ������¼���ȶ��ߵ���
	uint8 Vertical_find_left_flag = 0;//����������ǰ�Ƽ��к��Ƿ���������߽� Ĭ��0Ϊδ���ţ�0δ��������������б��������һ��ͼ��б��
	uint8 Vertical_find_right_flag = 0;//����������ǰ�Ƽ��к��Ƿ��������ұ߽� Ĭ��0Ϊδ���ţ�0δ��������������б��������һ��ͼ��б��
	uint8 out_point = 0;//�����ж�С����ǰ��һ���Ƿ�ӽ�ȫ�� �����ж��Ƿ����
	uint8 out_point1 = 0;//�����ж�С����ǰ��һ���Ƿ�ӽ�ȫ�� �����ж��Ƿ����
	uint8 out_point2 = 0;
	uint8 L_near_slope_flag = 0;//�Ƿ��и���С��ǰ�漸�б߽��������߽�б�� 1Ϊ��
	uint8 R_near_slope_flag = 0;//�Ƿ��и���С��ǰ�漸�б߽�������ұ߽�б�� 1Ϊ��
	int16 L_earliest_lost_row = 0, R_earliest_lost_row = 0, L_R_earliest_lost_row = 0;//���ұ����綪������
	//int16 L_earliest_find_row = 63,R_earliest_find_row = 63;
	int16 Left_end = 0;//ÿ��ͼ��ɨ��ʱ ��߽���������
	int16 Right_end = 187;//ÿ��ͼ��ɨ��ʱ �ұ߽���������
	int16 i = 63, j = (int16)Middle_line;                                                                          //63
	int16 Left_start = Last_mid_line, Right_start = Last_mid_line;//��ʼ������ɨ����ʼ�� ��ʼΪ��һ��ͼ������
	int16 Vertical_row_1 = 63;//����������Զ�˺ڵ����ڵ���                                                                         //63
	int16 Vertical_longest_length_1 = 0;//�����������ȣ���Զ�ˣ�
	int16 Vertical_length = 63;//������������                                                                          //63
	int16 Vertical_longest_length_2 = 63;//������������  �����Ƚ�                                                                         //63
	int16 Vertical_column_1 = 93;//����������¼������Զ�˵� ���� ����
	int16 Vertical_column_2 = 93;
	int16 Vertical_column = 93;//���ռ�¼������������¼������Զ�˵� ���� ����
	int16 Vertical_search_row_left = 0, Vertical_search_row_right = 0;//��������ʱ ����Զ����ǰ���n�� ��Ϊ�ñ�������̬ ����������Զ��ǰ��(����С��)n���������ߵĺڵ� ���ڶ�����ǰһ����������б����
	int16 Aver_mid_line;//����ƽ��ֵ  ��ȡƫ����
	float Miline_weight_sum = 0;//��Ȩ�㷨  ���߼�Ȩ�ܺ�
	float Weight_sum = 0;//Ȩ���ܺ� ���߼�Ȩ�ܺͳ���Ȩ���ܺ� ���ɵõ��ó�ͼ���Ȩ�����������(��)������  ��ͼ������(79)������ɵõ����ڶ��PD�õ�ƫ��


	Left_slope = 0, Right_slope = 0;//���ұ��� ����  б��  ����ɿ��ǲ�����
	Cross_road_flag = 0;//ʮ�ֶ��߱�־λ  ��һ��Ϊ������ʮ��
	L_lost_cnt = 0;//��߽綪�ߴ���
	R_lost_cnt = 0;//�ұ߽綪�ߴ���
	L_R_lost_cnt = 0;//���ұ߽�ͬʱ���ߴ���
	Cross_road_cnt = 0;//ʮ�ֶ��ߴ���
	Row_start = 63;//ɨ����ʼ��;                                                                         //63

/********************�����ǻ���ʶ�����õ�����ر����ȷ�����*********************/
	uint8 Island_flag1 = 0;
	uint8 L0_R1 = 0;
	uint8 L1_R0 = 0;
	int16 Island_inflexion_row = 0;
	int16 Left_line_swerve_R = 0;
	int16 Left_line_swerve_L = 0;
	int16 Right_line_swerve_L = 0;
	int16 Right_line_swerve_R = 0;

	int16 L1_R1_cnt = 0;
	int16 L0_R1_cnt = 0;
	int16 L1_R0_cnt = 0;
	int16 L1_R1_row = 63;                                                                         //63
	int16 L0_R1_row = 63;                                                                         //63
	int16 L1_R0_row = 63;                                                                         //63
	int16 O_R_Island_row = 63;                                                                         //63
	int16 O_L_Island_row = 63;                                                                         //63
	int16 Big_island_error2 = 0;
	int16 Big_island_error3 = 0;

	Fir_jump_point_row = 0;
	Fir_jump_point_column = 0;
	Sec_jump_point_row = 0;
	Sec_jump_point_column = 0;
	/********************�����ǻ���ʶ�����õ�����ر����ȷ�����*********************/

	/**********************�����ǻᳵ��ʶ�����õ�����ر���************************/
	uint8 L_dent_flag = 0;
	uint8 L_cripling_flag = 0;
	int16 L_dent_row = 63;                                                                         //63
	int16 L_cripling_row = 63;                                                                         //63
	int16 L_cripling = 0;
	int16 L_dotted_line = 0;
	int16 L_cripling2 = 0;
	uint8 R_dent_flag = 0;
	uint8 R_cripling_flag = 0;
	int16 R_dent_row = 63;                                                                         //63
	int16 R_cripling_row = 63;                                                                         //63
	int16 R_cripling = 0;
	int16 R_dotted_line = 0;
	int16 R_cripling2 = 0;

	for (int i = 0; i < ROW; i++)
	{
		for (int j = 0; j < COL; j++)
		{
			binary_img[i][j] = imagein[i][j];
		}
	}

	Line_coordinates_init();
	//img_binary();



	for (row = Row_start; row >= 0; row--)
	{
		/*
		 *  @brief        �߽����ƣ���ֹԽ��
		 *  @param        ~
		 *  Sample usage  ~
		 */
		Found_left_flag = 0;                                         //��߽綪�߱��
		Found_right_flag = 0;                                        //�ұ߽綪�߱��
		Left_line.x[row] = Left_line.x[row + 1] - 4;                     //ɨ�߽߱����ƣ����ݱ��е���ֵ��Ԥ����һ��ɨ��λ�ã���С��������
		Right_line.x[row] = Right_line.x[row + 1] + 4;
		if (Left_line.x[row] < 0)      Left_line.x[row] = 0;            //��������ƽ�棬����Խ��
		if (Right_line.x[row] > 187)   Right_line.x[row] = 187;
		Left_end = (int16)Left_line.x[row];                     //ɨ���н�������
		Right_end = (int16)Right_line.x[row];
		if (Left_end < 0)      Left_end = 0;
		if (Right_end > 187)   Right_end = 187;
		/*
		 *  @brief        Ѱ��ɨ��߽���ߣ��ҵ����¼�ڵ����겢����־
		 *  @param        start       ɨ����ʼ��
		 *  @param        end         ɨ����ֹ��
		 *  Sample usage
		 */

		for (j = Left_start; j > Left_end; j--)
		{
			if (binary_img[row][j] == 0 && binary_img[row][j - 1] == 0)                     //�ں�  ���������������Ϻڵ����  ���ڿ��Ǽ��˲� ���������������Ļ�  ��ԭʼ�˲������׵��м���ֺڵ㣬��úڵ�Ϊ��
			{
				Left_line.x[row] = j;                                //��¼�ڵ������е�����
				Found_left_flag = 1;                                 //�ڵ㲶׽�ɹ����
				Left_line_lost[row] = 0;                            //��¼���ڵ���ɫ��Ϊ��ɫ
				break;
			}
		}
		for (j = Right_start; j < Right_end; j++)
		{
			if (binary_img[row][j] == 0 && binary_img[row][j + 1] == 0)                     //�ں�
			{
				Right_line.x[row] = j;
				Found_right_flag = 1;
				Right_line_lost[row] = 0;
				break;
			}
		}
		/*
		 *  @brief        ͼ��Զ���Ƿ��а����ߣ��еĻ�ֹͣ����Ѳ�ߣ���ֹ���ţ�
		 *  @param
		 *  @param
		 *  Sample usage
		 */


		 /*
		  *  @brief        ���������
		  *  @param
		  *  @param
		  *  Sample usage
		  */
		if (Found_left_flag && Left_line.x[row] - Left_line.x[row + 1] < 0)//������������  ƫ�ң��˴�����ƫ��  ���ƫ����Ϊ��ת  ���߿���ʮ�ֵ��µ�ͼ�����  ��ʱ�ж�����ת����ͼ��������ҵ��ı߽�
		{
			if (!Found_right_flag || Right_line.x[row] - Right_line.x[row + 1] > 0)//�ұ�δ�ҵ������ұ��߼�ȥ��һ�е��ұ��ߴ���0������Ϊ��ͼ�����
			{
				Found_left_flag = 0;                                 //��Ǳ���δ�ҵ������ı߽磬���ҵ��ı߽�Ϊͼ����䵼�º����������ɵ�����߽�
				Left_line_lost[row] = 1;                            //��Ǳ��ж���
			}
		}
		if (Found_right_flag && Right_line.x[row] - Right_line.x[row + 1] > 0)//������������  ƫ�󣬴˴�����ƫ��  ���ƫ����Ϊ��ת  ���߿���ʮ�ֵ��µ�ͼ�����  ��ʱ�ж�����ת����ͼ��������ҵ��ı߽�
		{
			if (!Found_left_flag || Left_line.x[row] - Left_line.x[row + 1] < 0)//���δ�ҵ���������߼�ȥ��һ�е��ұ���С��0������Ϊ��ͼ�����
			{
				Found_right_flag = 0;                                 //��Ǳ���δ�ҵ������ı߽磬���ҵ��ı߽�Ϊͼ����䵼�º����������ɵ�����߽�
				Left_line_lost[row] = 1;                            //��Ǳ��ж���
			}
		}
		/*
		*  @brief        ��������  �����������������Ų�������
		*  @param
		*  @param
		*  Sample usage
		*/
		if (row < 63 && Found_left_flag && Found_right_flag)                                                                         //63
		{
			if (Left_line.x[row] - Left_line.x[row + 1] > 50)
			{
				Left_line.x[row] = Left_line.x[row + 1];
				//gpio_set(PTC10,1);
				//DELAY_MS(4);
				//gpio_set(PTC10,0);
			}
			else if (Right_line.x[row + 1] - Right_line.x[row] > 50)
			{
				Right_line.x[row] = Right_line.x[row + 1];
				//gpio_set(PTC10,1);
				//DELAY_MS(4);
				//gpio_set(PTC10,0);
			}
		}
		/*
		 *  @brief          ��¼���ұ߽����ȶ������ڵ���
		 *  @param         ~null
		 *  @param         ~null
		 *  Sample usage   ~null
		 */
		if (!Found_left_flag && Found_right_flag && !L_line_lost_flag)  //��¼��һ����߽絥�߶������ڵ���
		{
			L_line_lost_flag = 1;
			L_earliest_lost_row = row;
		}
		if (!Found_right_flag && Found_left_flag && !R_line_lost_flag)//��¼��һ���ҽ絥�߶������ڵ���
		{
			R_line_lost_flag = 1;
			R_earliest_lost_row = row;
		}
		if (!Found_left_flag && !Found_right_flag && !L_R_line_lost_flag)//��¼��һ�����ұ߽�ͬʱ�������ڵ���
		{
			L_R_line_lost_flag = 1;
			L_R_earliest_lost_row = row;
		}
		if (!L_fir_line_flag && !Left_line_lost[row])//��¼��һ����߽�û���ߵ�������
		{
			L_fir_line_flag = 1;
			//L_earliest_find_row = row;
		}
		if (!R_fir_line_flag && !Right_line_lost[row])//��¼��һ���ұ߽�û���ߵ�������
		{
			R_fir_line_flag = 1;
			//R_earliest_find_row = row;
		}
		/*
		 *  @brief        ��¼���߶��ߴ���  ������ߵ� ��
		 *  @param         ~null
		 *  @param         ~null
		 *  Sample usage   ~null
		 */
		if (row > 5)
		{
			if (!Found_left_flag || !Found_right_flag)//���ߵ�������� �����жϷǶ��һ�٣�������߶�δ�����������ٽ����ж�
			{
				if (!Found_left_flag && !Found_right_flag)//ʮ�ֶ��� ����ȫ��
				{
					L_R_lost_cnt++;//ʮ�ֶ��ߴ���(Ҳ����˵��˫�߶��ߴ�����������д��Cross_road  ��Ϊ����������ʮ�ֶ���)
					L_R_lost_row = row;//��¼һ��ͼ�����һ������ͬʱ���ߵ��� 
					if (row > 10)
						Cross_road_cnt++;
				}
				if (!Found_left_flag && Found_right_flag)
				{
					L_lost_cnt++;//����߶��ߴ���
					L_last_lost_row = row;//��¼һ��ͼ��������һ�ζ��ߵ�������
				}
				if (Found_left_flag && !Found_right_flag)
				{
					R_lost_cnt++;//���ұ߶��ߴ���
					R_last_lost_row = row;//��¼һ��ͼ���ұ����һ�ζ��ߵ�������
				}
			}
		}
		/*
	   *  @brief ���ö�����ǰ�����Ч����б��   �������������������б��   ����б�ʱ�Զ��б��׼ȷ�ȶ�������б���޷����ٲ���Զ��б��
	   *  @param
	   *  @param
	   *  Sample usage   ~null
	   */
		if ((L_line_lost_flag || L_R_line_lost_flag) && !Get_L_near_slope_flag)
		{
			uint8 L_slope_point = 0;
			int16 Scan_L_slope_row;
			Get_L_near_slope_flag = 1;
			if (L_line_lost_flag)
			{
				Scan_L_slope_row = L_earliest_lost_row;
			}
			else
				Scan_L_slope_row = L_R_earliest_lost_row;
			for (i = Scan_L_slope_row + 1; i < Scan_L_slope_row + 11; i++)
			{
				if (!Left_line_lost[i])
				{
					L_slope_point++;
				}
			}
			if (L_slope_point > 3)
			{
				Left_slope = (Left_line.x[Scan_L_slope_row + 2] - Left_line.x[Scan_L_slope_row + L_slope_point]) / (L_slope_point - 2);
				if (Left_slope > 0)
				{
					L_near_slope_flag = 1;
				}
				else
				{
					L_near_slope_flag = 0;
				}
			}
		}

		if ((R_line_lost_flag || L_R_line_lost_flag) && !Get_R_near_slope_flag)
		{
			uint8 R_slope_point = 0;
			int16 Scan_R_slope_row;
			Get_R_near_slope_flag = 1;

			if (R_line_lost_flag)
			{
				Scan_R_slope_row = R_earliest_lost_row;
			}
			else
				Scan_R_slope_row = L_R_earliest_lost_row;
			for (i = Scan_R_slope_row + 1; i < Scan_R_slope_row + 11; i++)
			{
				if (!Right_line_lost[i])
				{
					R_slope_point++;
				}
			}
			if (R_slope_point > 3)
			{
				Right_slope = (Right_line.x[Scan_R_slope_row + 2] - Right_line.x[Scan_R_slope_row + R_slope_point]) / (R_slope_point - 2);
				if (Right_slope < 0)
				{
					R_near_slope_flag = 1;
				}
				else
				{
					R_near_slope_flag = 0;
				}
			}
		}

		/*
		 *  @brief          ʮ�� ���� ����
		 *  @param         ~null
		 *  @param         ~null
		 *  Sample usage   ~null
		 */

		if (1)
		{
			if (Vertical_search_flag || (L_near_slope_flag && R_near_slope_flag))
			{
				if (!Island_flag && !Found_left_flag && !Found_right_flag)
				{
					Left_line.x[row] = Left_line.x[row + 1] + Left_slope;
					Right_line.x[row] = Right_line.x[row + 1] + Right_slope;
				}
				else if ((Island_flag == 0 || Island_flag == 1) && Left_slope > 0 && (((Left_line.x[row + 1] - Left_line.x[row + 2] > 0 || Left_line.x[row + 2] - Left_line.x[row + 3] > 0 || L_near_slope_flag)) && !Found_left_flag && Found_right_flag))
				{
					Left_line.x[row] = Left_line.x[row + 1] + Left_slope;//������б��Ϊk>0(k<0) ��ÿ��λ��(��)��|k|���Դ�����
					//gpio_set(PTC10,0);
				}
				else if ((Island_flag == 0 || Island_flag == 2) && Right_slope < 0 && (((Right_line.x[row + 2] - Right_line.x[row + 1] > 0 || Right_line.x[row + 3] - Right_line.x[row + 2] > 0 || R_near_slope_flag)) && Found_left_flag && !Found_right_flag))
				{
					Right_line.x[row] = Right_line.x[row + 1] + Right_slope;//������б��Ϊk>0(k<0) ��ÿ��λ��(��)��|k|���Դ�����

				}
			}
		}

		if (!Island_flag)
		{
			/*if(meet_flag)
			{
				Midd_line.x[row] = (Right_line.x[row] - Single_R_line[row]) + 93 + 25;//������ʻ�� �����ô��ַ�ʽ��ֱ�ӽ����յ���ֵƽ�Ƽ���
			}
			else*/ Midd_line.x[row] = (Left_line.x[row] + Right_line.x[row]) / 2;//ÿ������������
		}
		else if (Island_flag)
		{
			if (Island_flag == 1 || Island_flag == 5)
			{
				if (Island_flag == 1)        Midd_line.x[row] = (Right_line.x[row] - Single_R_line[row]) + 93;
				else if (Island_flag == 5)   Midd_line.x[row] = (Right_line.x[row] - Single_R_line[row]) + 93;
				if (Island_flag == 5 && Right_line.x[row] - Single_R_line[row] > 0)    Midd_line.x[row] = 93;
			}
			else if (Island_flag == 2 || Island_flag == 6)
			{
				if (Island_flag == 2)        Midd_line.x[row] = (Left_line.x[row] - Single_L_line[row]) + 93;
				else if (Island_flag == 6)        Midd_line.x[row] = (Left_line.x[row] - Single_L_line[row]) + 93;
				if (Island_flag == 6 && Left_line.x[row] - Single_L_line[row] < 0)    Midd_line.x[row] = 93;
			}
			else if (Island_flag == 3 || Island_flag == 4)
			{
				if (Island_flag == 3)
				{
					Midd_line.x[row] = (Left_line.x[row] - Single_L_line[row]) + 93;
				}
				else if (Island_flag == 4)
				{
					Midd_line.x[row] = (Right_line.x[row] - Single_R_line[row]) + 93;
				}
			}
		}


		/*
	   *  @brief        ���������������������б�ʣ�б���൱��  ÿ��λ������ ���� �ƶ�����λ ��һ�ε����Һ���
	   *  @param         ~null
	   *  @param         ~null
	   *  Sample usage   ~null
	   */

	   /*
	   *  @brief        ������������¼��Զ�˱�Ե�ڵ����꣬
	   *  @param         ~null
	   *  @param         ~null
	   *  Sample usage   ~null
	   */
		if ((!Found_left_flag || !Found_right_flag || Island_flag == 3 || Island_flag == 4 || meet_flag) && !Vertical_search_flag)  //���߶��� ������һ��ͼ��ȷ��Ϊ����������������
		{
			Vertical_search_flag = 1;                                     //����������־λ
			if (!Found_left_flag || !Found_right_flag || Island_flag == 3 || Island_flag == 4 || meet_flag)
			{
				for (j = 1; j < 187; j++)//49~110  �����з�Χ������ܻ��ܵ��ٽ���������
				{
					for (i = row; i > 0; i--)
					{


						if (((binary_img[i][j] == 0 && binary_img[i - 1][j] == 0) || (binary_img[i + 1][j] != 0 && binary_img[i][j] == 0)) || (i == 1))
						{
							Vertical_length = row - i;//����row��������Զ���ĳ���
							if (Vertical_length > Vertical_longest_length_1)
							{
								Vertical_longest_length_1 = Vertical_length;//��¼����row��������Զ���ڵ��� ����
								Vertical_row_1 = i;//��¼����row���� ��Զ���ڵ��W���꣨������
								Vertical_column_1 = j;//��¼����row���� ��Զ���ڵ��H���꣨������
							}
							else if (Vertical_length == Vertical_longest_length_1)
							{
								Vertical_longest_length_2 = Vertical_length;
								//Vertical_row_2 = i;
								Vertical_column_2 = j;
							}

							break;
						}
					}
				}
			}
			if (Vertical_longest_length_1 == Vertical_longest_length_2)
			{
				Vertical_column = (Vertical_column_1 + Vertical_column_2) / 2;
			}
			else
				Vertical_column = Vertical_column_1;
		}


		/*
		 *  @brief        ���������������������б�ʣ�б���൱��  ÿ��λ������ ���� �ƶ�����λ ��һ�ε����Һ���
		 *  @param         ~null  �˴�������������Զ�������е�ǰ��n�����������������ڵ�  ���붪������б�� ������ټ�һ����б�ʷ���
		 *  @param         ~null  ��ĳһ�߽綪��ʱ����ǰ�����Ч�߽�������ĳ��ֵʱ����������ǰ������������Ч�߽���б�ʣ������ý�����������
		 *  Sample usage   ~null
		 */
		if (Vertical_search_flag && !Fill_line_flag && (!L_near_slope_flag || !R_near_slope_flag))                 //������������
		{
			Fill_line_flag = 1;
			if (!L_near_slope_flag)
			{
				Vertical_search_row_left = Vertical_row_1 + 3;  //��Զ��ͼ������������ ǰ������
				while (!Vertical_find_left_flag)
				{
					for (j = Vertical_column; j > 30; j--)//�˴����Ʒ�Χ���Ƿſ� ��С
					{
						if (binary_img[Vertical_search_row_left][j] == 0 && binary_img[Vertical_search_row_left][j - 1] == 0)
						{
							Left_line.x[Vertical_search_row_left] = j;
							Vertical_find_left_flag = 1;
							break;
						}
					}
					if (!Vertical_find_left_flag)
					{
						Vertical_search_row_left++;
						if (Vertical_search_row_left - Vertical_row_1 - 3 > 10)
						{
							break;//����Զ��ǰ��ʮ�о�δ�������߽��ʱ�����˳��������� б��������һ��ͼ��ı߽�б��
						}
					}
				}
			}
			if (!R_near_slope_flag)
			{
				Vertical_search_row_right = Vertical_row_1 + 3;
				while (!Vertical_find_right_flag)
				{
					for (j = Vertical_column; j < 139; j++)//�˴����Ʒ�Χ���Ƿſ� ����
					{
						if (binary_img[Vertical_search_row_right][j] == 0 && binary_img[Vertical_search_row_right][j + 1] == 0)
						{
							Right_line.x[Vertical_search_row_right] = j;
							Vertical_find_right_flag = 1;
							break;
						}
					}
					if (!Vertical_find_right_flag)
					{
						Vertical_search_row_right++;
						if (Vertical_search_row_right - Vertical_row_1 - 3 > 10)
						{
							break;//����Զ��ǰ��ʮ�о�δ�������߽��ʱ�����˳��������� б��������һ��ͼ��ı߽�б��
						}
					}
				}
			}
			if (Vertical_find_left_flag)
			{
				Left_slope = (Left_line.x[Vertical_search_row_left] - Left_line.x[row]) / (row - Vertical_search_row_left);
			}

			if (Vertical_find_right_flag)
			{
				Right_slope = (Right_line.x[Vertical_search_row_right] - Right_line.x[row]) / (row - Vertical_search_row_right);
			}
		}

		/*
		 *  @brief        ��������   ������Ҫ�޸�
		 *  @param         Near_flag  �������� ��־
		 *  @param         ~null
		 *  Sample usage   ~null
		 */
		if (!meet_flag && row < Row_start)
		{
			if ((fabs)(Midd_line.x[row] - Midd_line.x[row + 1]) > 20)//����������ֵ����ĳ��ֵ�������Ϊ���⵼��Զ����������ֵ���䣬�ɿ���ֱ��break�˳����������
			{
				Midd_line.x[row] = Midd_line.x[row + 1];
			}
			else
			{
				if (Midd_line.x[row] - Midd_line.x[row + 1] > 8)
				{
					Midd_line.x[row] = Midd_line.x[row + 1] + 8;
					if (Midd_line.x[row] > 187)
					{
						Midd_line.x[row] = 187;
					}
				}
				else if (Midd_line.x[row] - Midd_line.x[row + 1] < -8)
				{
					Midd_line.x[row] = Midd_line.x[row + 1] - 8;
					if (Midd_line.x[row] < 0)
					{
						Midd_line.x[row] = 0;
					}
				}
			}
		}
		if ((((binary_img[row][(int16)Midd_line.x[row]] == 0) && (binary_img[row][(int16)Midd_line.x[row + 1]] == 0)) || ((binary_img[row][(int16)Midd_line.x[row + 1]] == 0) && (binary_img[row][(int16)Midd_line.x[row + 2]] != 0))) || Midd_line.x[row] == 0 || Midd_line.x[row] == 187)
		{
			row = row + 1;//���ж���� ��Ҫ���¶���  ���������е�Ϊ��ɫ���˳�ɨ��
			break;
		}
		else if ((Island_flag == 3 && Midd_line.x[row] - Midd_line.x[row + 1] > 0) || (Island_flag == 4 && Midd_line.x[row] - Midd_line.x[row + 1] < 0))
		{
			row = row + 1;   //�ƻ�ʱ �� ���������������˳�ɨ��  ���һ������������˳�ɨ��
			break;
		}

		Left_start = (int16)Midd_line.x[row];//��һ��ɨ�����ʱ����ʼ��Ϊ��������������
		Right_start = (int16)Midd_line.x[row];//��һ��ɨ�ұ���ʱ����ʼ��Ϊ��������������
	}

	if (Cross_road_cnt > 8)
	{
		Cross_road_flag = 1;//���������������ұ߽�ͬʱ���� ʱ  ȷ��Ϊʮ��
	}

	Row_end = row + 1;//��¼����ͼ������������Զ�˵�������
	Column_end = (int16)Midd_line.x[Row_end];//��¼����ͼ������������Զ�˵������� ����������������

	if (Row_end > Row_start - 2)     Near_flag = 1;//����ͼ������������Զ�˵������� ֻ��ɨ����ʼ��С1�������������ֻ��һ�У�����ΪС�����������߽� ���������ߣ�
	else                Near_flag = 0;
	Row_begging = Row_start;
	Last_row_end = Row_end;
	/*
	 *  @brief        �����ж�
	 *  @param
	 *  @param         ~null
	 *  Sample usage   ~null   Reconfirm_L_island
	 */

	if (Island_flag == 0 && !Cross_road_flag && !Ram_flag && Vertical_row_1 < 5)
	{
		for (i = 63; i > 5; i--)                                                                         //63
		{
			if (!Left_line_lost[i] && !Right_line_lost[i])
			{
				L1_R1_cnt++;
				if (L0_R1 || L1_R0)
				{
					L1_R1_row = i;
					break;
				}
			}
			else if (Left_line_lost[i] && !Right_line_lost[i])
			{
				L0_R1_cnt++;
				if (!L0_R1)
				{
					L0_R1 = 1;
					L0_R1_row = i;
				}
			}
			else if (!Left_line_lost[i] && Right_line_lost[i])
			{
				L1_R0_cnt++;
				if (!L1_R0)
				{
					L1_R0 = 1;
					L1_R0_row = i;
				}
			}
		}
		if (L1_R0_cnt < 1 && L0_R1 && R_lost_cnt < 1 && L_R_lost_cnt < 1 && L0_R1_row - L1_R1_row>5) //����ʮ��  ���ֿ��޸�    �ı������ϸ�̶�
		{
			if (Left_line.x[L1_R1_row - 1] - Left_line.x[L1_R1_row] > 3 && ((!Left_line_lost[63] && L0_R1_cnt > 9 && Left_line.x[L1_R1_row + 1] - Left_line.x[L1_R1_row + 2] > 0) || (Left_line_lost[63] && L0_R1_cnt > 12 && Left_line.x[L1_R1_row] - Left_line.x[L1_R1_row + 1] > 15 && Left_line.x[L1_R1_row - 2] - Left_line.x[L1_R1_row - 1] > 3)))                                                                         //63
			{
				if (fabs((Right_line.x[L1_R1_row + 5] - Right_line.x[L1_R1_row]) - (Right_line.x[L0_R1_row] - Right_line.x[L0_R1_row - 5])) < 5)
				{
					Island_flag = 1;
				}
			}
		}
		else if (L0_R1_cnt < 1 && L1_R0 && L_lost_cnt < 1 && L_R_lost_cnt < 1 && L1_R0_row - L1_R1_row>5) //����ʮ��  ���ֿ��޸�    �ı������ϸ�̶�
		{
			if (Right_line.x[L1_R1_row - 1] - Right_line.x[L1_R1_row] < -3 && ((!Right_line_lost[63] && L1_R0_cnt > 9 && Right_line.x[L1_R1_row + 1] - Right_line.x[L1_R1_row + 2] < 0) || (Right_line_lost[63] && L1_R0_cnt > 12 && Right_line.x[L1_R1_row] - Right_line.x[L1_R1_row + 1] < -15 && Right_line.x[L1_R1_row - 2] - Right_line.x[L1_R1_row - 1] < -3)))
			{
				if (fabs((Left_line.x[L1_R1_row + 5] - Left_line.x[L1_R1_row]) - (Left_line.x[L1_R0_row] - Left_line.x[L1_R0_row - 5])) < 5)
				{
					Island_flag = 2;
				}
			}
		}
	}
	else if (Island_flag == 1)
	{
		for (i = 62; i > Row_end; i--)
		{
			if (Left_line.x[i] - Left_line.x[i + 1] > 3)
			{
				Big_island_error3++;//�������е����������������Ĵ���  �������ִ�С��
			}
			else if (Left_line.x[i] - Left_line.x[i + 1] > 2)
			{
				Big_island_error2++;//�������е������������ڶ��Ĵ���  �������ִ�С��
			}
			if (!Left_line_lost[i + 1] && !Left_line_lost[i] && !Left_line_lost[i - 1] && L_lost_cnt < 7)
			{
				if (Left_line.x[i] - Left_line.x[i - 1] > 0)
				{
					Island_inflexion_row = i;
					break;
				}
			}
		}
		if (Big_island_error3 > 12 || (Big_island_error3 + Big_island_error2 > 15))//�󻷵��뾶��Сʱ  ��С12 15
		{
			Big_island_flag = 1;
		}
		if (Island_inflexion_row > (Big_island_flag ? 10 : 15))//�󻷵��뾶��Сʱ  ��С15
		{
			Island_flag = 3;
		}
	}
	else if (Island_flag == 2)
	{
		for (i = 62; i > Row_end; i--)
		{
			if (Right_line.x[i + 1] - Right_line.x[i] > 3)
			{
				Big_island_error3++;//�������е����������������Ĵ���  �������ִ�С��
			}
			else if (Right_line.x[i + 1] - Right_line.x[i] > 2)
			{
				Big_island_error2++;//�������е������������ڶ��Ĵ���  �������ִ�С��
			}
			if (!Right_line_lost[i + 1] && !Right_line_lost[i] && !Right_line_lost[i - 1] && R_lost_cnt < 7)
			{
				if (Right_line.x[i] - Right_line.x[i - 1] < 0)
				{
					Island_inflexion_row = i;
					break;
				}
			}
		}
		if (Big_island_error3 > 12 || (Big_island_error3 + Big_island_error2 > 15))//�󻷵��뾶��Сʱ  ��С12 15
		{
			Big_island_flag = 1;
		}
		if (Island_inflexion_row > (Big_island_flag ? 10 : 15))//�󻷵��뾶��Сʱ  ��С15
		{
			Island_flag = 4;
		}
	}
	else if (Island_flag == 3 && Island_time > Island_change_time)
	{
		if (Right_line_lost[63] && Right_line_lost[62] && Right_line_lost[61])//���ҵ�ʱǰ�����������ȫ��
		{
			for (i = 63; i > 0; i--)
			{
				if (binary_img[i][187] == 0 && binary_img[i - 1][187] == 0)
				{
					O_L_Island_row = i;
					break;
				}
			}
			if (O_L_Island_row < 63 && O_L_Island_row>20)
			{
				Island_flag = 5;
			}
		}
	}
	else if (Island_flag == 4 && Island_time > Island_change_time)
	{
		if (Left_line_lost[63] && Left_line_lost[62] && Left_line_lost[61])//���ҵ�ʱǰ�����������ȫ��
		{
			for (i = 63; i > 0; i--)
			{
				if (binary_img[i][0] == 0 && binary_img[i - 1][0] == 0)
				{
					O_R_Island_row = i;
					break;
				}
			}
			if (O_R_Island_row < 63 && O_R_Island_row>20)
			{
				Island_flag = 6;
			}
		}
	}
	else if (Island_flag == 5)
	{
		for (i = 63; i > (Big_island_flag ? 10 : 15); i--)
		{
			if (Left_line_lost[i])
			{
				Island_flag1 = 1;
			}
			if (Left_line.x[i] - Left_line.x[i - 1] < 0)
				Left_line_swerve_R++;//������������
			else if (Left_line.x[i] - Left_line.x[i - 1] > 0)
				Left_line_swerve_L++;//������������
			if (Right_line.x[i] - Right_line.x[i - 1] > 0)
				Right_line_swerve_L++;//������������
			else if (Right_line.x[i] - Right_line.x[i - 1] < 0)
				Right_line_swerve_R++;//������������
		}
		if (!Big_island_flag && !Island_flag1 && Left_line_swerve_L < 1 || Cross_road_flag)
		{
			Island_flag = 0;
		}
		else if (Big_island_flag && !Island_flag1 && Left_line_swerve_L < 1 || Cross_road_flag)
		{
			Island_flag = 0;
			Big_island_flag = 0;
		}
	}
	else if (Island_flag == 6)
	{
		for (i = 63; i > (Big_island_flag ? 10 : 15); i--)
		{
			if (Right_line_lost[i])
			{
				Island_flag1 = 1;
			}
			if (Left_line.x[i] - Left_line.x[i - 1] < 0)
				Left_line_swerve_R++;//������������
			else if (Left_line.x[i] - Left_line.x[i - 1] > 0)
				Left_line_swerve_L++;//������������
			if (Right_line.x[i] - Right_line.x[i - 1] > 0)
				Right_line_swerve_L++;//������������
			else if (Right_line.x[i] - Right_line.x[i - 1] < 0)
				Right_line_swerve_R++;//������������
		}
		if (!Big_island_flag && !Island_flag1 && Right_line_swerve_R < 1 || Cross_road_flag)
		{
			Island_flag = 0;
		}
		else if (Big_island_flag && !Island_flag1 && Right_line_swerve_R < 1 || Cross_road_flag)
		{
			Island_flag = 0;
			Big_island_flag = 0;
		}
	}

	if (Island_flag == 3 || Island_flag == 4)
	{
		Steer.tp = -0.4;
		Steer.td = -3;

	}
	else if (meet_flag)
	{
		Steer.tp = 1;
	}
	else
	{
		Steer.tp = 0;
		Steer.td = 0;
	}

	/*
	 *  @brief        �ᳵ��ʶ��
	 *  @param         Near_flag
	 *  @param         ~null
	 *  Sample usage   ~null
	 */
	if (((meet_time == 0 && Start_time == 2000) || (meet_time == 1500)) && !meet_flag && !Island_flag && !Cross_road_flag && L_lost_cnt < 1 && R_lost_cnt < 1)//ֱ���ᳵ��
	{
		for (i = 63; i > Row_end; i--)
		{
			if (!L_dent_flag && !Left_line_lost[i] && !Left_line_lost[i - 1] && Left_line.x[i] - Left_line.x[i - 1] > 0)
			{
				L_dent_flag = 1;//�����а���
				L_dent_row = i;//��¼���ݵ������е�ǰһ�У�������Ϊǰ��СΪ��
			}
			else if (!L_dent_flag && !Left_line_lost[i] && !Left_line_lost[i - 1] && Left_line.x[i - 1] - Left_line.x[i] > 3)
			{
				L_cripling_flag = 1;//������͹����
				L_cripling_row = i;//��¼͹���������е�ǰһ��
				L_cripling++;//͹���Ĵ���
			}
			else if (L_dent_flag && ((!Left_line_lost[i] && !Left_line_lost[i - 1] && Left_line.x[i - 1] - Left_line.x[i] > 3) || L_cripling_flag))
			{
				if (L_cripling_flag && L_cripling_row - L_dent_row < 10 && L_cripling_row - L_dent_row>2)
				{
					L_dotted_line++;//�����ְ��ݲ��������������������������ֵ����3 �����а�������͹�����Ĵ���  ��������һ�α�׼������
					L_cripling_flag = 0;//ֻ��͹���ı�־ ���尼�ݵı�־
				}
				else if (L_dent_row - i < 10 && L_dent_row - i>0)
				{
					L_dotted_line++;//�����ְ��ݲ��������������������������ֵ����3 �����а�������͹�����Ĵ���  ��������һ�α�׼������
					L_cripling_flag = 0;//��͹���ı�־ �尼�ݵı�־
					L_dent_flag = 0;//�尼�ݵı�־
				}
			}
			if (!Left_line_lost[i] && !Left_line_lost[i - 1] && Left_line.x[i - 1] - Left_line.x[i] > 2)
			{
				L_cripling2++;
			}


			if (!R_dent_flag && !Right_line_lost[i] && !Right_line_lost[i - 1] && Right_line.x[i - 1] - Right_line.x[i] > 0)
			{
				R_dent_flag = 1;//�����а���
				R_dent_row = i;//��¼���ݵ������е�ǰһ�У�������Ϊǰ��СΪ��
			}
			else if (!R_dent_flag && !Right_line_lost[i] && !Right_line_lost[i - 1] && Right_line.x[i - 1] - Right_line.x[i] > 3)
			{
				R_cripling_flag = 1;//������͹����
				R_cripling_row = i;//��¼͹���������е�ǰһ��
				R_cripling++;//͹���Ĵ���
			}
			else if (R_dent_flag && (!Right_line_lost[i] && !Right_line_lost[i - 1] && Right_line.x[i] - Right_line.x[i - 1] > 3 || R_cripling_flag))
			{
				if (R_cripling_flag && R_cripling_row - R_dent_row < 10 && R_cripling_row - R_dent_row>2)
				{
					R_dotted_line++;//�����ְ��ݲ��������������������������ֵ����3 �����а�������͹�����Ĵ���  ������������һ�α�׼������
					R_cripling_flag = 0;//ֻ��͹���ı�־ ���尼�ݵı�־
				}
				else if (R_dent_row - i < 10 && R_dent_row - i>0)
				{
					R_dotted_line++;//�����ְ��ݲ��������������������������ֵ����3 �����а�������͹�����Ĵ���  ������������һ�α�׼������
					R_cripling_flag = 0;//��͹���ı�־ �尼�ݵı�־
					R_dent_flag = 0;//�尼�ݵı�־
				}
			}
			if (!Right_line_lost[i] && !Right_line_lost[i - 1] && Right_line.x[i] - Right_line.x[i - 1] > 2)
			{
				R_cripling2++;
			}
		}
		if ((L_dotted_line + R_dotted_line > 3) && L_cripling2 > 2 && R_cripling2 > 2)
		{
			if (meet_time < 1500)
				meet_flag = 1;
			else
				meet_flag = 2;
		}
	}

	/*
	 *  @brief        Ȩ�ؼ�Ȩ�ۼ�  ���ж��е��Ƿ�Ϊ�� ����Ϊ�׾�����
	 *  @param         Near_flag
	 *  @param         ~null
	 *  Sample usage   ~null
	 */
	for (i = Row_end; i <= Row_start; i++)  //ɨ����ʼ��;
	{
		if (binary_img[i][(int16)Midd_line.x[i]] != 0)
		{
			Miline_weight_sum += Midd_line.x[i] * Add_weight[i - Row_end];//�����������ܺ�
			Weight_sum += Add_weight[i - Row_end];//Ȩֵ����
		}
	}

	if (meet_flag == 1)
		Aver_mid_line = (int16)(Miline_weight_sum / Weight_sum) + 20;//�����������ֵ
	else if (meet_flag == 2 && !Stop_ready2)
		Aver_mid_line = (int16)(Miline_weight_sum / Weight_sum) + 24;//�����������ֵ
	else
		Aver_mid_line = (int16)(Miline_weight_sum / Weight_sum);

	if (meet_flag && (Left_car_flag_mid || Left_car_flag_end))
		Last_mid_line = Aver_mid_line;


	Steer.error = Aver_mid_line - 93;//ƫ��
	if (Near_flag)
		Steer.error = old_error;
	old_error = Steer.error;
	for (j = 0; j < 188; j++)
	{
		if (binary_img[63][j] == 0)
			out_point++;
		if (binary_img[15][j] == 0)
			out_point1++;
		if (binary_img[62][j] == 0)
			out_point2++;
	}
	if ((out_point > 187 && out_point1 > 165 && out_point2 > 187 && binary_img[63][40] == 0 && binary_img[63][129] == 0 && binary_img[63][0] == 0 && binary_img[63][187] == 0 && binary_img[63][93] == 0 && binary_img[62][40] == 0 && binary_img[62][129] == 0 && binary_img[62][0] == 0 && binary_img[62][187] == 0 && binary_img[62][93] == 0 && abs(Steer.error < 5)))
		out_flag = 1;
	
	for (int i = 0; i < ROW; i++)
	{
		side[i] = Left_line.x[i];
		if (side[i] > 187)side[i] = 187;
	}
	for (int i = 64, m = 0; i < 2 * ROW; i++, m++)
	{
		side[i] = Midd_line.x[m];
		if (side[i] > 187)side[i] = 187;
	}
	for (int i = 64 * 2, m = 0; i < 3 * ROW; i++, m++)
	{
		side[i] = Right_line.x[m];
		if (side[i] > 187)side[i] = 187;
	}
	return side;
}


/**********************************************************************************
Func    Name: ����ͼ�������� ���� ��ʼ��
Descriptions:  Mid.x[i]  Left.x[i]  Right.x[i]  �ֱ�Ϊͼ��������� �����ߵ�x����W������

Input   para: ~
In&Out  Para: ~
Output  para: ~
Return value: ~
***********************************************************************************/
int16 First_row;
int16 Last_row;


void Track_calculate_judge(void)
{
	//Get_line_LMR();
	uint8 i;
	Last_row = Row_end;
	int16 Row_save_num = Last_row;
	int16 L_turn_row = 0, R_turn_row = 0;
	int16 Straight_row = 0, Swerve_trend = 0;
	//Ram_flag = 0;
/*
 *  @brief        ת������   �����������߲�ֵ����
 *  @param         Swerve_trend ת������
 *  @param         ~null
 *  Sample usage   ~null
 */
	for (i = Row_start; i > Row_save_num + 1; i--)
	{
		if (Midd_line.x[i + 1] - Midd_line.x[i] >= 1)
		{
			L_turn_row++;
		}
		else if (Midd_line.x[i] - Midd_line.x[i + 1] >= 1)
		{
			R_turn_row++;
		}
		else
		{
			Straight_row++;
		}
	}
	Swerve_trend = abs(L_turn_row - R_turn_row);//ת������
/*
 *  @brief        �µ����
 *  @param         ~null
 *  @param         ~null
 *  Sample usage   ~null
 */
	if (!Island_flag && !Cross_road_flag)
	{
		if (!Ram_flag && !Left_line_lost[15] && !Right_line_lost[15])
		{
			if (L_R_lost_cnt < 1 && L_lost_cnt < 1 && R_lost_cnt < 1 && Right_line.x[15] - Left_line.x[15]>95 && Swerve_trend < 5)
			{
				Ram_flag = 1;//����
			}
			/*else if(L_R_lost_cnt<1&&L_lost_cnt<8&&R_lost_cnt<8&&Right_line.x[15]-Left_line.x[15]>95&&Swerve_trend<5)
			{
				Ram_flag = 2;//����  �˴���Ϊʵ����ԭ��  ���µ�ǰ��Ǽ��� ����õ�һ��ֱ���ж�Ϊ���»�������
			}*/
		}
		if (Ram_flag)
		{
			Last_mid_line = 93;
			Ram_flag++;
			if (Ram_flag > 10)
			{
				Ram_flag = 0;
			}
		}

	}
	if (Ram_flag || Island_flag || meet_flag)
	{
		//gpio_set(PTC10, 1);
	}
	else {}
	//gpio_set(PTC10, 0);
/*
 *  @brief        ֱ�����
 *  @param         ~null
 *  @param         ~null
 *  Sample usage   ~null
 */
	if (!Ram_flag && Swerve_trend < 5 && Straight_row>22)
	{
		Highest_speed_flag = 1;
		Lowest_speed_flag = 0;
	}
	else        Highest_speed_flag = 0;
}


//
///*!
//  * @brief    ������
//  * @param
//  * @return
//  * @date     2020/6/28 ������
//  */
//void TFTSPI_BinRoadSide(uint8_t imageOut[MT9V03X_H][2])
//{
//	int i = 0;
//
//	for (i = 0; i < ROAD_START_ROW; i++)
//	{
//		//TFTSPI_Draw_Dot(imageOut[i][0], i, u16RED);
//		imageOut[i][0] = 100;
//		//TFTSPI_Draw_Dot(imageOut[i][1], i, u16GREEN);
//		imageOut[i][1] = 200;
//
//	}
//
//}
//
//
//
///*!
//  * @brief    �ж��Ƿ���ֱ��
//  * @param    image �� ��ֵͼ����Ϣ
//  * @return   0������ֱ���� 1��ֱ��
//  * @note     ˼·�����߱��߶�����
//  * @date     2020/6/23 ���ڶ�
//  */
//uint8_t RoadIsStraight(uint8_t imageSide[MT9V03X_H][2])
//{
//	uint8_t i = 0;
//	uint8_t leftState = 0, rightState = 0;
//
//	/* ������Ƿ񵥵� */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		if (imageSide[i][0] + 5 < imageSide[i + 1][0])
//		{
//			leftState = 1;
//			break;
//		}
//	}
//
//	/* �ұ����Ƿ񵥵� */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		if (imageSide[i][1] - 5 > imageSide[i + 1][1])
//		{
//			rightState = 1;
//			break;
//		}
//	}
//
//	if (leftState == 1 && rightState == 1)
//	{
//		return 1;
//	}
//
//	return 0;
//}
//
//
///*!
//  * @brief    �ж��Ƿ��ǰ�����
//  * @param    image �� ��ֵͼ����Ϣ
//  * @return   0�����ǣ� 1����
//  * @note     ˼·��
//  * @date     2020/6/23 ���ڶ�
//  */
//uint8_t RoadIsZebra(uint8_t image[MT9V03X_H][MT9V03X_W], uint8_t* flag)
//{
//	int i = 0, j = 0;
//	int count = 0;
//
//	for (i = ROAD_MAIN_ROW - 10; i > ROAD_MAIN_ROW + 10; i++)
//	{
//		for (j = 1; j < MT9V03X_W; j++)
//		{
//			if (image[i][j] == 1 && image[i][j - 1] == 0)
//			{
//				count++;
//			}
//		}
//		if (count > 8)
//		{
//			*flag = 1;
//			return 1;
//		}
//	}
//
//
//	return 0;
//}
//
///*!
//  * @brief    �ж��Ƿ���ʮ��
//  * @param    imageSide �� ͼ�������Ϣ
//  * @param    flag      �� ʮ��״̬��Ϣ
//  * @return   0�����ǣ� 1����
//  * @note     ˼·���������߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� ��֤����ʮ��
//  * @date     2020/6/23 ���ڶ�
//  */
//uint8_t RoadIsCross(uint8_t imageSide[MT9V03X_H][2], uint8_t* flag)
//{
//	int i = 0;
//	uint8_t  rightState = 0;
//	int start[5] = { 0, 0, 0, 0, 0 }, end[5] = { 0, 0, 0, 0, 0 };
//	uint8_t count = 0;
//	uint8_t index = 0;
//
//
//	/* ����Ҳ���߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		if (imageSide[i][1] == 187)
//		{
//			count++;
//		}
//		else
//		{
//			if (count > 10 && index < 5)
//			{
//				start[index] = i + count;
//				end[index] = i;
//				index++;
//			}
//			count = 0;
//		}
//
//	}
//
//	if (index > 1)
//	{
//		if (end[0] - start[1] > 10)
//		{
//			rightState = 1;
//		}
//	}
//
//	index = 0;
//
//	/* ��������߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� */
//	if (rightState == 1)
//	{
//		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//		{
//			if (imageSide[i][0] == 0)
//			{
//				count++;
//			}
//			else
//			{
//				if (count > 10 && index < 5)
//				{
//					start[index] = i + count;
//					end[index] = i;
//					index++;
//				}
//				count = 0;
//			}
//
//		}
//
//		if (index > 1)
//		{
//			if (end[0] - start[1] > 10)
//			{
//				*flag = 1;
//				return 1;
//			}
//		}
//	}
//
//	return 0;
//
//}
//
//
//
//
///*!
//  * @brief    �ж��Ƿ��ǻ���
//  * @param    image �� ������Ϣ
//  * @param    flag  �� ����״̬��Ϣ
//  * @return   0�����ǣ� 1���󻷵�  2���һ���
//  * @note     ˼·��һ�������ϸ񵥵�����һ�����߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� ��֤���л���
//  * @date     2020/6/23 ���ڶ�
//  */
//uint8_t RoadIsRoundabout(uint8_t image[MT9V03X_H][2], uint8_t* flag)
//{
//	int i = 0;
//	uint8_t leftState = 0, rightState = 0;
//	int start[5] = { 0, 0, 0, 0, 0 }, end[5] = { 0, 0, 0, 0, 0 };
//	uint8_t count = 0;
//	uint8_t index = 0;
//
//	/* ������Ƿ񵥵� */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		if (image[i][0] + 5 < image[i + 1][0])
//		{
//			leftState = 1;
//			break;
//		}
//	}
//
//	/* �ұ����Ƿ񵥵� */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		if (image[i][1] - 5 > image[i + 1][1])
//		{
//			rightState = 1;
//			break;
//		}
//	}
//
//	/* ��ߵ����� ����Ҳ��Ƿ��ǻ��� */
//	if (leftState == 0)
//	{
//		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//		{
//			if (image[i][1] == 187)
//			{
//				count++;
//			}
//			else
//			{
//				if (count > 10 && index < 5)
//				{
//					start[index] = i + count;
//					end[index] = i;
//					index++;
//				}
//				count = 0;
//			}
//
//		}
//
//		if (index > 1)
//		{
//			if (end[0] - start[1] > 10)
//			{
//				*flag = 2;
//				return 2;
//			}
//		}
//	}
//
//	/* �ұߵ����� �������Ƿ��ǻ��� */
//	if (rightState == 0)
//	{
//		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//		{
//			if (image[i][0] == 0)
//			{
//				count++;
//			}
//			else
//			{
//				if (count > 10 && index < 5)
//				{
//					start[index] = i + count;
//					end[index] = i;
//					index++;
//				}
//				count = 0;
//			}
//
//		}
//
//		if (index > 1)
//		{
//			if (end[0] - start[1] > 10)
//			{
//				*flag = 1;
//				return 1;
//			}
//		}
//	}
//
//	return 0;
//}
//
//
///*!
//  * @brief    ��ȡ��������
//  * @param    imageInput �� ��ֵͼ����Ϣ
//  * @param    imageOut   �� ��������
//  * @param    status     �� 1���󻷵�  2���һ���
//  * @return
//  * @note     ˼·������һ�߱����ϸ񵥵�������һ�߱��ߣ���ȡ��һ����
//  * @date     2020/6/23 ���ڶ�
//  */
//void RoundaboutGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t status)
//{
//	int i = 0, j = 0;
//
//	switch (status)
//	{
//
//		/* �󻷵� */
//	case 1:
//	{
//		/* ����ȷ����߽� */
//		for (i = ROAD_START_ROW; i > ROAD_END_ROW; i--)
//		{
//			for (j = MT9V03X_W / 2; j > 0; j--)
//			{
//				if (imageInput[i][j])
//				{
//					imageSide[i][0] = j;
//					break;
//				}
//			}
//		}
//		break;
//	}
//
//	case 2:
//	{
//		/* ����ȷ���ұ߽� */
//		for (i = ROAD_START_ROW; i > ROAD_END_ROW; i--)
//		{
//			for (j = MT9V03X_W / 2; j < MT9V03X_W; j++)
//			{
//				if (imageInput[i][j])
//				{
//					imageSide[i][1] = j;
//					break;
//				}
//			}
//		}
//		break;
//	}
//	}
//
//
//}
//
///*!
//  * @brief    �жϻ��������Ƿ���ڻ���
//  * @param    imageInput �� ��ֵͼ����Ϣ
//  * @param    imageOut   �� ��������
//  * @param    status     �� 1���󻷵�  2���һ���
//  * @return
//  * @note
//  * @date     2020/6/23 ���ڶ�
//  */
//uint8_t RoundaboutGetArc(uint8_t imageSide[MT9V03X_H][2], uint8_t status, uint8_t* index)
//{
//	int i = 0;
//	uint8_t inc = 0, dec = 0;
//	switch (status)
//	{
//	case 1:
//		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//		{
//			if (imageSide[i][0] != 0 && imageSide[i + 1][0] != 0)
//			{
//				if (imageSide[i][0] >= imageSide[i + 1][0])
//				{
//					inc++;
//				}
//				else
//				{
//					dec++;
//				}
//
//				/* �л��� */
//				if (inc > 5 && dec > 5)
//				{
//					*index = i + 5;
//					return 1;
//				}
//			}
//			else
//			{
//				inc = 0;
//				dec = 0;
//			}
//		}
//
//		break;
//
//	case 2:
//		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//		{
//			if (imageSide[i][1] != 187 && imageSide[i + 1][1] != 187)
//			{
//				if (imageSide[i][1] > imageSide[i + 1][1])
//				{
//					inc++;
//				}
//				else
//				{
//					dec++;
//				}
//
//				/* �л��� */
//				if (inc > 5 && dec > 5)
//				{
//					*index = i + 5;
//					return 1;
//				}
//			}
//			else
//			{
//				inc = 0;
//				dec = 0;
//			}
//		}
//
//		break;
//	}
//
//	return 0;
//}
//
///*!
//  * @brief    ���ߴ���
//  * @param    imageSide  : ����
//  * @param    status     : 1������߲���   2���ұ��߲���
//  * @param    startX     : ��ʼ�� ����
//  * @param    startY     : ��ʼ�� ����
//  * @param    endX       : ������ ����
//  * @param    endY       : ������ ����
//  * @return
//  * @note     endY һ��Ҫ���� startY
//  * @date     2020/6/24 ������
//  */
//void ImageAddingLine(uint8_t imageSide[MT9V03X_H][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
//{
//	int i = 0;
//
//	/* ֱ�� x = ky + b*/
//	float k = 0.0f, b = 0.0f;
//	switch (status)
//	{
//	case 1:
//	{
//		k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
//		b = (float)startX - (float)startY * k;
//
//		for (i = startY; i < endY; i++)
//		{
//			imageSide[i][0] = (uint8_t)(k * i + b);
//		}
//		break;
//	}
//
//	case 2:
//	{
//		k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
//		b = (float)startX - (float)startY * k;
//
//		for (i = startY; i < endY; i++)
//		{
//			imageSide[i][1] = (uint8_t)(k * i + b);
//		}
//		break;
//	}
//
//	}
//}
//
///*!
//  * @brief    Ѱ�������
//  * @param    imageSide   �� ��������
//  * @param    status      ��1����߽�   2���ұ߽�
//  * @return
//  * @note
//  * @date     2020/6/24 ������
//  */
//uint8_t ImageGetHop(uint8_t imageSide[MT9V03X_H][2], uint8_t state, uint8_t* x, uint8_t* y)
//{
//	int i = 0;
//	uint8_t px = 0, py = 0;
//	uint8_t count = 0;
//	switch (state)
//	{
//	case 1:
//		/* Ѱ������� */
//		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//		{
//			if (imageSide[i][0] == 0 && i > (ROAD_END_ROW + 5))
//			{
//				count++;
//
//				if (count > 5)
//				{
//					if (imageSide[i - 1][0] > (imageSide[i][0] + 20))
//					{
//						py = i - 1;
//						px = imageSide[py][0];
//						break;
//					}
//					if (imageSide[i - 2][0] > (imageSide[i - 1][0] + 20))
//					{
//						py = i - 2;
//						px = imageSide[py][0];
//						break;
//					}
//					if (imageSide[i - 3][0] > (imageSide[i - 2][0] + 20))
//					{
//						py = i - 3;
//						px = imageSide[py][0];
//						break;
//					}
//					if (imageSide[i - 4][0] > (imageSide[i - 3][0] + 20))
//					{
//						py = i - 4;
//						px = imageSide[py][0];
//						break;
//					}
//
//				}
//
//			}
//			else
//			{
//				count = 0;
//			}
//		}
//
//		if (py != 0)
//		{
//			*x = px;
//			*y = py;
//			return 1;
//		}
//
//		break;
//
//
//	case 2:
//		/* Ѱ������� */
//		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//		{
//			if (imageSide[i][1] == 187 && i > (ROAD_END_ROW + 5))
//			{
//				count++;
//
//				if (count > 5)
//				{
//					if (imageSide[i - 1][1] < (imageSide[i][1] - 20))
//					{
//						py = i - 1;
//						px = imageSide[py][1];
//						break;
//					}
//					if (imageSide[i - 2][1] < (imageSide[i - 1][1] - 20))
//					{
//						py = i - 2;
//						px = imageSide[py][1];
//						break;
//					}
//					if (imageSide[i - 3][1] < (imageSide[i - 2][1] - 20))
//					{
//						py = i - 3;
//						px = imageSide[py][1];
//						break;
//					}
//					if (imageSide[i - 4][1] < (imageSide[i - 3][1] - 20))
//					{
//						py = i - 4;
//						px = imageSide[py][1];
//						break;
//					}
//
//				}
//
//			}
//			else
//			{
//				count = 0;
//			}
//		}
//
//		if (py != 0)
//		{
//			*x = px;
//			*y = py;
//			return 1;
//		}
//
//		break;
//	}
//
//	return 0;
//
//}
//
//
///*!
//  * @brief    �������ߴ���
//  * @param    imageInput �� ��ֵͼ����Ϣ
//  * @param    imageSide  �� ��������
//  * @param    status     ��������־λ   1�������󻷵�   2�������һ���   3���󻷵������뻷  4���һ��������뻷  5���󻷵���������  6���һ�����������
//  * @return
//  * @note     ����ֻд���󻷵����һ�����ҿ��Բο��󻷵��Լ�����
//  * @date     2020/6/24 ������
//  */
//uint8_t* RoundaboutProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t* state)
//{
//	int i = 0;
//	uint8_t pointX = 0, pointY = 0;
//	uint8_t err = 0;
//	static uint8_t cnt = 0;
//	switch (*state)
//	{
//		/* �����󻷵� �������ڴ����� */
//	case 1:
//
//		/* ����ȷ����߽� */
//		RoundaboutGetSide(imageInput, imageSide, 1);
//
//		/* ��黡�� */
//		err = RoundaboutGetArc(imageSide, 1, &pointY);
//
//		/* �л��� ���в��� ���ӻ������ҵ� �� ͼ�����½� */
//		if (err)
//		{
//			pointX = imageSide[pointY][0];
//
//			/* ׼���뻷�� */
//			if ((pointY + 10) > ROAD_MAIN_ROW)
//			{
//				*state = 3;
//			}
//		}
//		else
//		{
//			pointY = ROAD_START_ROW - 1;
//
//			/* û�л��� ���в��� ���ӱ������ҵ� �� ͼ�����½� */
//			for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//			{
//				if (imageSide[i][0] > imageSide[pointY][0])
//				{
//					pointY = i;
//				}
//			}
//
//			pointX = imageSide[pointY][0];
//		}
//
//		/* ���� */
//		ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
//
//		break;
//
//		/* �����󻷵� �������ڴ����� */
//	case 2:
//
//		*state = 0;
//		break;
//
//
//		/* ׼�����뻷���� ֱ�����ֲ��� */
//	case 3:
//		pointY = ROAD_START_ROW - 1;
//
//		/* ����ȷ����߽� */
//		RoundaboutGetSide(imageInput, imageSide, 1);
//
//		/* ��黡�� */
//		err = RoundaboutGetArc(imageSide, 1, &pointY);
//
//		/* �л��� ���в��� ���ӻ������ҵ� �� ͼ�����½� */
//		if (err)
//		{
//			pointX = imageSide[pointY][0];
//
//			if ((pointY + 10) > ROAD_MAIN_ROW)
//			{
//				/* �������ڲ��� */
//				ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
//			}
//		}
//
//		/* ������ڲ��� */
//		pointX = MT9V03X_W / 3;
//		pointY = ROAD_START_ROW - 1;
//
//		/* Ѱ������� */
//		ImageGetHop(imageSide, 1, &pointX, &pointY);
//
//		if (pointY >= ROAD_MAIN_ROW && pointY != ROAD_START_ROW - 1)
//		{
//			imageSide[ROAD_MAIN_ROW][0] = 0;
//			*state = 5;
//		}
//
//		/* ���� */
//		ImageAddingLine(imageSide, 2, pointX + 30, pointY, (MT9V03X_W - 1), ROAD_START_ROW);
//
//		break;
//
//	case 4:
//
//		break;
//
//		/* �������� ֱ�������� */
//	case 5:
//
//
//		/* ��黡�� */
//		err = RoundaboutGetArc(imageSide, 2, &pointY);
//
//		if (err || cnt)
//		{
//			cnt++;
//
//			imageSide[ROAD_MAIN_ROW][0] = 0;
//			imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W / 3;
//		}
//
//		if (cnt > 20)
//		{
//			cnt = 0;
//			*state = 0;
//
//		}
//
//		break;
//
//		/* �������� ֱ�������� */
//	case 6:
//
//		break;
//
//	}
//	for (int i = 0; i < 120; i++)
//	{
//		if (imageSide[i][0] == 0) {
//			imageSide[i][0] = 1;
//		}
//		if (imageSide[i][0] >= 187) {
//			imageSide[i][0] = 187;
//		}
//		if (imageSide[i][1] == 0) {
//			imageSide[i][1] = 1;
//		}
//		if (imageSide[i][1] >= 187) {
//			imageSide[i][1] = 187;
//		}
//		//imageOut[i][1] = 110;
//	}
//	return imageSide[0];
//}
//
///*!
//  * @brief    ��ȡʮ�ֱ���
//  * @param    imageInput �� ��ֵͼ����Ϣ
//  * @param    imageOut   �� ��������
//  * @return
//  * @note     ˼·�����м�����������
//  * @date     2020/6/23 ���ڶ�
//  */
//void CrossGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2])
//{
//	int i = 0, j = 0;
//
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		for (j = 78; j > 1; j--)
//		{
//			if (imageInput[i][j])
//			{
//				imageSide[i][0] = j;
//				break;
//			}
//		}
//
//		for (j = 78; j < 187; j++)
//		{
//			if (imageInput[i][j])
//			{
//				imageSide[i][1] = j;
//				break;
//			}
//		}
//	}
//
//}
//
///*!
//  * @brief    ʮ�ֲ��ߴ���
//  * @param    imageInput �� ��ֵͼ����Ϣ
//  * @param    imageSide  �� ��������
//  * @param    status     ��ʮ�ֱ�־λ   1������ʮ��    2������ʮ��   3����ʮ��
//  * @return
//  * @note
//  * @date     2020/6/24 ������
//  */
//void CrossProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t* state)
//{
//
//	uint8_t pointX = 0, pointY = 0;
//	uint8_t leftIndex = 0;
//	static uint8_t count = 0;
//	switch (*state)
//	{
//	case 1:
//	{
//		/* ���»�ȡ���� */
//		CrossGetSide(imageInput, imageSide);
//
//		/* Ѱ������� */
//		if (ImageGetHop(imageSide, 1, &pointX, &pointY))
//		{
//			/* ���� */
//			ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
//		}
//
//		leftIndex = pointY;
//		pointX = 0;
//		pointY = 0;
//
//		/* Ѱ������� */
//		if (ImageGetHop(imageSide, 2, &pointX, &pointY))
//		{
//			/* ���� */
//			ImageAddingLine(imageSide, 2, pointX, pointY, (MT9V03X_W - 1), ROAD_START_ROW);
//		}
//
//		if (leftIndex != 0 && pointY != 0 && leftIndex >= ROAD_MAIN_ROW && pointY >= ROAD_MAIN_ROW)
//		{
//			*state = 2;
//			count = 0;
//		}
//
//		if (count++ > 20)
//		{
//			*state = 2;
//			count = 0;
//		}
//
//		break;
//	}
//
//	case 2:
//	{
//
//		/* ��黡�� */
//		if (RoundaboutGetArc(imageSide, 1, &leftIndex))
//		{
//			/* ����ȷ����߽� */
//			RoundaboutGetSide(imageInput, imageSide, 1);
//
//			if (ImageGetHop(imageSide, 1, &pointX, &pointY))
//			{
//				/* ���� */
//				ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);
//
//				*state = 3;
//
//				count = 0;
//			}
//			else
//			{
//				imageSide[ROAD_MAIN_ROW][0] = MT9V03X_W / 2;
//				imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W - 1;
//			}
//		}
//
//		break;
//	}
//
//	case 3:
//	{
//
//		/* ����ȷ����߽� */
//		RoundaboutGetSide(imageInput, imageSide, 1);
//
//
//		if (ImageGetHop(imageSide, 1, &pointX, &pointY))
//		{
//			/* ��黡�� */
//			if (RoundaboutGetArc(imageSide, 1, &leftIndex))
//			{
//				/* ���� */
//				ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);
//			}
//			else
//			{
//				/* ���� */
//				ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
//			}
//
//			if (pointY >= ROAD_MAIN_ROW)
//			{
//				*state = 0;
//				count = 0;
//			}
//		}
//		else
//		{
//			imageSide[ROAD_MAIN_ROW][0] = 120;
//			imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W - 1;
//		}
//
//		if (count++ > 10)
//		{
//			*state = 0;
//			count = 0;
//		}
//
//		break;
//	}
//	}
//
//}
//
///*!
//  * @brief    ͣ���ߴ���
//  * @param    imageSide  �� ��������
//  * @param    state      �� ͣ��״̬  1�����������   2���������Ҳ�
//  * @param    speed      �� �ٶ�
//  * @return
//  * @note
//  * @date     2020/6/24 ������
//  */
//void ZebraProcess(uint8_t imageSide[MT9V03X_H][2], uint8_t state, int16_t* speed)
//{
//	static uint16_t count = 0;
//
//	count++;
//
//	if (state == 1)
//	{
//		imageSide[ROAD_MAIN_ROW][0] = 0;
//		imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W / 2;
//	}
//	else
//	{
//		imageSide[ROAD_MAIN_ROW][0] = MT9V03X_W / 2;
//		imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W - 1;
//	}
//
//	if (count > 100)
//	{
//		*speed = 0;
//		while (1);
//	}
//
//}
//
///*!
//  * @brief    ���������У���ȡ���ƫ��
//  * @param
//  * @return
//  * @note
//  * @date     2020/6/24 ������
//  */
//int16_t RoadGetSteeringError(uint8_t imageSide[MT9V03X_H][2], uint8_t lineIndex)
//{
//
//	return imageSide[lineIndex][0] + imageSide[lineIndex][1] - 158;
//
//}
//
//typedef struct Side
//{
//	uint8_t arr_left[120];
//	uint8_t arr_right[120];
//}Side, * Sidepoint;
//Sidepoint p = (Sidepoint)malloc(sizeof(Side));
///*!
//  * @brief    �ж��Ƿ���
//  * @param    imageInput �� ��ֵͼ����Ϣ
//  * @param    imageOut   �� ��������
//  * @param    lineIndex  �� ��
//  * @return   0��û�ж���   1:��߶���  2���ұ߶���  3�� ���Ҷ�����   4������
//  * @note
//  * @date     2020/6/24 ������
//  */
//uint8_t RoadIsNoSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2], uint8_t lineIndex)
//{
//	uint8_t state = 0;
//	int i = 0;
//	static uint8_t last = 78;
//
//	imageOut[lineIndex][0] = 0;
//	imageOut[lineIndex][1] = 187;
//	/* �þ���С���ȽϽ����У� �ж��Ƿ��� */
//	for (i = last; i > 1; i--)
//	{
//		if (imageInput[lineIndex][i])
//		{
//			imageOut[lineIndex][0] = i;
//			break;
//		}
//	}
//
//	if (i == 1)
//	{
//		/* ��߽綪�� */
//		state = 1;
//	}
//
//
//	for (i = last; i < 187; i++)
//	{
//		if (imageInput[lineIndex][i])
//		{
//			imageOut[lineIndex][1] = i;
//			break;
//		}
//	}
//
//	if (i == 187)
//	{
//		/* ���ұ߽綪�� */
//		if (state == 1)
//		{
//			state = 3;
//		}
//
//		/* �ұ߽綪�� */
//		else
//		{
//			state = 2;
//		}
//
//	}
//
//	if(imageOut[lineIndex][0] > last - 50)
//	{
//	    if(last + 50 < MT9V03X_W)
//	    {
//	        last += 50;
//	    }
//	    else
//	    {
//	        last = MT9V03X_W - 5;
//	    }
//	}
//	else if(imageOut[lineIndex][1] < last + 50)
//	{
//	    if(last - 50 > 0)
//	    {
//	        last -= 50;
//	    }
//	    else
//	    {
//	        last = 5;
//	    }
//	}
//	else
//	{
//	    last = 78;
//	}
//
//	if (imageOut[lineIndex][1] <= imageOut[lineIndex][0])
//	{
//		state = 4;
//	}
//	return state;
//
//}
//
//
///*!
//  * @brief    ���ߴ���
//  * @param    imageInput �� ��ֵͼ����Ϣ
//  * @param    imageOut   �� ��������
//  * @param    mode       �� �Ǳ߶��ߣ�   1����߶���  2���ұ߶���
//  * @param    lineIndex  �� ��������
//  * @return
//  * @note
//  * @date     2020/6/24 ������
//  */
//void RoadNoSideProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2], uint8_t mode, int lineIndex)
//{
//	int i = 0, j = 0, count = 0;
//
//	switch (mode)
//	{
//	case 1:
//		for (i = imageOut[lineIndex][1]; i > 1; i--)
//		{
//			count++;
//			for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
//			{
//				if (imageInput[j][i])
//				{
//					imageOut[lineIndex - count][0] = 0;
//					imageOut[lineIndex - count][1] = i;
//					break;
//				}
//
//			}
//		}
//		break;
//
//
//	case 2:
//		for (i = imageOut[lineIndex][0]; i < 187; i++)
//		{
//			count++;
//			for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
//			{
//				if (imageInput[j][i])
//				{
//					imageOut[lineIndex - count][0] = i;
//					imageOut[lineIndex - count][1] = 187;
//					break;
//				}
//
//			}
//		}
//		break;
//
//	}
//
//}
//
//
//uint8_t* test(uint8_t image_in[MT9V03X_H][MT9V03X_W],uint8_t image_out[MT9V03X_H][MT9V03X_W])
//{
//	for (int i = 0; i < MT9V03X_H; i++)
//	{
//		for (int j = 0; j < MT9V03X_W; j++)
//		{
//			image_out[i][j] = 100;
//		}
//	}
//	return image_out[0];
//}
//
//
///*!
//  * @brief    ��ȡ����
//  * @param    imageInput �� ��ֵͼ����Ϣ
//  * @param    imageOut   �� ��������
//  * @return   �Ƿ���
//  * @note     ˼·���Ӿ��복ͷ�Ͻ����п�ʼ���м�����������
//  * @date     2020/6/23 ���ڶ�
//  */
//  uint8_t* ImageGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2])
//  {
//  	int i = 0, j = 0;
//  	RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);
//  
//  	/* �복ͷ����40�У�Ѱ�ұ��� */
//  	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//  	{
//  		imageOut[i][0] = 0;   //0
//  		imageOut[i][1] = 187;
//  
//  		/* ���ݱ߽��������� Ѱ�ұ߽� */
//  		for (j = imageOut[i + 1][0] + 10; j > 0; j--)
//  		{
//  			if (imageInput[i][j])
//  			{
//  				imageOut[i][0] = j;
//  				break;
//  			}
//  		}
//  
//  		for (j = imageOut[i + 1][1] - 10; j < 160; j++)
//  		{
//  			if (imageInput[i][j])
//  			{
//  				imageOut[i][1] = j;
//  				break;
//  			}
//  		}
//  
//  		/* �����߽� �����������ߣ� �����Ƿ��Ҷ��� */
//  		if (imageOut[i][0] > (MT9V03X_W / 2 - 10) && imageOut[i][1] > (MT9V03X_W - 5))
//  		{
//  			/* �Ҷ��ߴ��� */
//  			RoadNoSideProcess(imageInput, imageOut, 2, i);
//  
//  			if (i > 70)
//  			{
//  				imageOut[i][0] += 50;
//  			}
//  			//return 1;
//  		}
//  
//  		/* ����ұ߽� �����������ߣ� �����Ƿ����� */
//  		if (imageOut[i][1] < (MT9V03X_W / 2 + 10) && imageOut[i][0] < (5))
//  		{
//  			/* ���ߴ��� */
//  			RoadNoSideProcess(imageInput, imageOut, 1, i);
//  
//  			if (i > 70)
//  			{
//  				imageOut[i][1] -= 50;
//  			}
//  			//return 2;
//  
//  		}
//  	}
//	for (int i = 0; i < 120; i++)
//	{
//		if (imageOut[i][0] == 0) {
//			imageOut[i][0] = 1;
//		}
//		if (imageOut[i][0] >= 187) {
//			imageOut[i][0] = 187;
//		}
//		if (imageOut[i][1] == 0) {
//			imageOut[i][1] = 1;
//		}
//		if (imageOut[i][1] >= 187) {
//			imageOut[i][1] = 187;
//		}
//		//imageOut[i][1] = 110;
//	}
//  	return imageOut[0];
//  }
//
////uint8_t ImageGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2])
////{
////	int i = 0, j = 0;
////
////	RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);
////
////	/* �복ͷ����40�У�Ѱ�ұ��� */
////	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
////	{
////		imageOut[i][0] = 0;
////		imageOut[i][1] = 187;
////
////		/* ���ݱ߽��������� Ѱ�ұ߽� */
////		for (j = imageOut[i + 1][0] + 10; j > 0; j--)
////		{
////			if (imageInput[i][j])
////			{
////				imageOut[i][0] = j;
////				break;
////			}
////		}
////
////		for (j = imageOut[i + 1][1] - 10; j < 160; j++)
////		{
////			if (imageInput[i][j])
////			{
////				imageOut[i][1] = j;
////				break;
////			}
////		}
////
////		/* �����߽� �����������ߣ� �����Ƿ��Ҷ��� */
////		if (imageOut[i][0] > (MT9V03X_W / 2 - 10) && imageOut[i][1] > (MT9V03X_W - 5))
////		{
////			/* �Ҷ��ߴ��� */
////			RoadNoSideProcess(imageInput, imageOut, 2, i);
////
////			if (i > 70)
////			{
////				imageOut[i][0] += 50;
////			}
////			return 1;
////		}
////
////		/* ����ұ߽� �����������ߣ� �����Ƿ����� */
////		if (imageOut[i][1] < (MT9V03X_W / 2 + 10) && imageOut[i][0] < (5))
////		{
////			/* ���ߴ��� */
////			RoadNoSideProcess(imageInput, imageOut, 1, i);
////
////			if (i > 70)
////			{
////				imageOut[i][1] -= 50;
////			}
////			return 2;
////
////		}
////	}
////
////	return 0;
////}
//
//
///*!
//  * @brief    ����һ�����
//  * @param
//  * @return
//  * @note     ˼·�� �����������ڵ�9���㣬�����������ֵ�������õ�
//  * @date     2020/6/24 ������
//  */
//void ImagePortFilter(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][MT9V03X_W])
//{
//	uint8_t temp = 0;
//
//	for (int i = 1; i < MT9V03X_H - 1; i++)
//	{
//		for (int j = 1; j < MT9V03X_W - 1; j++)
//		{
//			temp = imageInput[i - 1][j - 1] + imageInput[i - 1][j] + imageInput[i - 1][j + 1] +
//				imageInput[i][j - 1] + imageInput[i][j] + imageInput[i][j + 1] +
//				imageInput[i + 1][j - 1] + imageInput[i + 1][j] + imageInput[i + 1][j + 1];
//
//			/* ������5�����Ǳ��� �����õ� ���Ե��������Ż��˲�Ч�� */
//			if (temp > 4)
//			{
//				imageOut[i][j] = 1;
//			}
//			else
//			{
//				imageOut[i][j] = 0;
//			}
//
//		}
//	}
//}
//
//
///*!
//  * @brief    ��������
//  * @note
//  */
//uint8_t ImageSide[MT9V03X_H][2];


/*************************ͼ������꣩********************/
//void xunxian(uint8_t imagein[MT9V03X_H][MT9V03X_W])
//{
//	uint8_t AX, AY; //A��
//	uint8_t BX, BY; //B��
//	uint8_t CX, CY; //C��
//	uint8_t DX, DY; //D��
//
//	AY = 59; //��ȡAB��
//	BY = 59;
//	//���������һ��������
//	for (int i = 39; i >= 1; i--) //���м�������������
//	{
//		if (imagein[AY][i - 1] - imagein[AY][i] == 1) //�ҵ�������
//		{
//			AX = i; //A������
//		}
//	}
//	for (int i = 39; i < 79; i++) //���м�������������
//	{
//		if (imagein[BY][i + 1] - imagein[BY][i] == 1) //�ҵ�������
//		{
//			BX = i; //B������
//		}
//	}
//
//	CY = AY - 1; //����C��
//	CX = AX - 1;
//	for (int i = CY; i > 0; i--) //�ɽ���Զ
//	{
//		for (int j = CX; j < 80; i++) //��������
//		{
//			if (imagein[i][j] == 0) //�ҵ��׵�
//			{
//				CX = j - 1; //�õ���һ��C��Xλ��
//				break;
//			}
//		}
//		if (imagein[i - 1][CX] == 1) //�ж��Ϸ�����û�кڵ�
//		{
//			CY = i;   //�õ�C��Yλ��
//			break;
//		}
//	}
//
//	DY = BY - 1; //����D��
//	DX = BX - 1;
//	for (int i = DY; i > 0; i--) //�ɽ���Զ
//	{
//		for (int j = DX; j > 0; i--) //��������
//		{
//			if (imagein[i][j] == 0) //�ҵ��׵�
//			{
//				DX = j + 1; //�õ���һ��D��Xλ��
//				break; //������ѭ��
//			}
//		}
//		if (imagein[i - 1][DX] == 1) //�ж��Ϸ�����û�кڵ�
//		{
//			DY = i;   //�õ�C��Yλ��
//			break;//������ѭ��
//		}
//	}
//
//	if (abs(CY - DY) < 10 && CY > 30 && DY > 30)  //�����ж�ʮ��·��
//	{
//		uint8_t Y = min(CY, DY);   //ȡ��CD�߶Ƚ�Сֵ
//		uint8_t HEI = 0;          //ʮ��·���Ϸ�����ڵ�����
//		for (i = Y; i > Y - 10; i -= 2)  //Y�����ѯ
//		{
//			for (j = 10; j < 70; j += 5) //X�����ѯ
//			{
//				if (imagein[i][j] == 1)  //�кڵ�
//				{
//					HEI++;  //��������++
//				}
//			}
//		}
//
//		if (HEI < 10) //�����ж�ʮ��·�ڣ�������
//		{
//			float K;    //����б��
//			K = (CX - AX) / (CY - AY); //����AC��б��
//
//			for (i = CY; i > CY - 20; i--)  //��AC�ӳ�2���ؿ���
//			{
//				imagein[i][CX + (CY - i) * K] = 1;     //��ͼ���Ӧ��Ϳ��
//				imagein[i][(CX + (CY - i) * K) - 1] = 1; //��ͼ���Ӧ��Ϳ��
//			}
//
//			K = (DX - BX) / (DY - BY); //����BD��б��
//
//			for (i = DY; i > DY - 20; i--)  //��BD�ӳ�2���ؿ���
//			{
//				imagein[i][DX + (DY - i) * K] = 1;     //��ͼ���Ӧ��Ϳ��
//				imagein[i][(DX + (DY - i) * K) - 1] = 1; //��ͼ���Ӧ��Ϳ��
//			}
//		}
//	}
//}
//
///*************************************************************/
//
///*************************������******************************/
//uint8_t ZHONGJIAN[60] = [39]; //����λ��
//uint8_t ZUO[60] = { 0 }; //����λ��
//uint8_t YOU[60] = { 79 }; //����λ��
//
//ZHONGJIAN[59] = (AX + BX) / 2; //�����һ��������λ���ҵ�
//
//for (i = 58; i > 0; i--) //���ϵ�����ʣ��������
//{
//	for (j = ZHONGJIAN[i + 1]; j >= 1; j--) //���м�������������
//	{
//		if (imagein[i][j - 1] - imagein[i][j] == 1) //�ҵ�������
//		{
//			ZUO[i] = j; //����
//		}
//	}
//	for (j = ZHONGJIAN[i + 1]; j < 79; j++) //���м�������������
//	{
//		if (imagein[i][j + 1] - imagein[i][j] == 1) //�ҵ�������
//		{
//			ZUO[i] = j; //����
//		}
//	}
//	ZHONGJIAN[i] = (ZUO[i] + YOU[i]) / 2; //���㵱ǰ�����ĵ�
//}
///*************************************************************************/
//
///***************************����ƫ��************************************/
//uint8_t QIANZHAN = 15;  //����ͷǰհ
//uint8_t YUAN, ZHONG, JIN;
//
//char ERR = 0; //ǰհƫ��
//char YERR = 0; //�������ƫ��
//
//JIN = ZHONGJIAN[59];
//ZHONG = ZHONGJIAN[59 - QIANZHAN];
//YUAN = ZHONGJIAN[59 - QIANZHAN * 2];
//
//if (YUAN < ZHONG && ZHONG < JIN)   //���1
//{
//	ERR = ((ZHONG - YUAN) + (JIN - ZHONG)) / 2;  //��ȡǰհƫ��
//}
//else if (YUAN < ZHONG && ZHONG >= JIN)//���2
//{
//	ERR = JIN - ZHONG;   //��ȡǰհƫ��
//}
//else if (YUAN >= ZHONG && ZHONG < JIN)//���3
//{
//	ERR = JIN - ZHONG;   //��ȡǰհƫ��
//}
//else //���4
//{
//	ERR = ((ZHONG - YUAN) + (JIN - ZHONG)) / 2;  //��ȡǰհƫ��
//}
//YERR = JIN - 39; //��ȡ�������ƫ��

/************************************************************************/

/*!
  * @brief    ͼ�����ݴ���
  * @param    ��
  * @return
  * @note     ��
  * @date     2020/4/28
  */
  //void ImageProcess(void)
  //{
  //	/* ��ȡһ֡ͼ��  ����ͼ���� */
  //	if (Field_Over_Flag)
  //	{
  //		uint32_t nowtime = systime.get_time_us();
  //
  //		/* �Ӳɼ�ͼ��������ȡ���Լ���Ҫʹ�õĴ�С */
  //		Get_Use_Image();
  //
  //		/* ������һ��ͼ����  �����Ϊ0�����ж��в��Ὺ���µ�DMA���䣬����õ��µ�ͼ�� */
  //		Field_Over_Flag = 0;
  //
  //		/* ������ȡ ��ȡͼ������б�����Ϣ �����Pixle������ */
  //		Get_01_Value(3);
  //
  //		if (KEY_Read(1) == 0)
  //		{
  //			/* �Ա�����Ϣ�����˲����� ȥ����һ��� */
  //			ImagePortFilter(Pixle, Image_Use);
  //
  //			/* ��ȡ�������� */
  //			g_ucIsNoSide = ImageGetSide(Image_Use, ImageSide);
  //
  //
  //			if (g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagZebra == 0)
  //			{
  //				/* ��⻷�� */
  //				RoadIsRoundabout(ImageSide, &g_ucFlagRoundabout);
  //			}
  //
  //
  //			if (g_ucFlagRoundabout)
  //			{
  //				/* �������� */
  //				RoundaboutProcess(Image_Use, ImageSide, &g_ucFlagRoundabout);
  //
  //			}
  //
  //			if (g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagZebra == 0)
  //			{
  //				/* ���ʮ�� */
  //				RoadIsCross(ImageSide, &g_ucFlagCross);
  //			}
  //
  //			if (g_ucFlagCross)
  //			{
  //				/* ʮ�ִ��� */
  //				CrossProcess(Image_Use, ImageSide, &g_ucFlagCross);
  //			}
  //
  //			/* ���������У���ȡ���ƫ�� */
  //			g_sSteeringError = RoadGetSteeringError(ImageSide, ROAD_MAIN_ROW);
  //
  //			nowtime = systime.get_time_us() - nowtime;
  //
  //#if 0       /* ����ʱ���Դ����� */   
  //			TFTSPI_BinRoad(0, 0, MT9V03X_H, MT9V03X_W, (uint8_t*)Image_Use);   //ͼ����ʾ
  //			TFTSPI_BinRoadSide(ImageSide);
  //			TFTSPI_Draw_Line(0, ROAD_MAIN_ROW, 187, ROAD_MAIN_ROW, u16RED);
  //			char txt[61];
  //			sprintf(txt, "%05d", g_sSteeringError);
  //			TFTSPI_P6X8Str(0, 15, txt, u16RED, u16BLUE);
  //
  //			sprintf(txt, "%06d", nowtime);
  //			TFTSPI_P6X8Str(10, 15, txt, u16RED, u16BLUE);
  //
  //			sprintf(txt, "%02d", g_ucFlagRoundabout);
  //			TFTSPI_P6X8Str(18, 15, txt, u16RED, u16BLUE);
  //
  //			sprintf(txt, "%02d", g_ucFlagCross);
  //			TFTSPI_P6X8Str(22, 15, txt, u16RED, u16BLUE);
  //#endif
  //		}
  //
  //		/* �������� ��ʾԭʼͼ�� */
  //		else
  //		{
  //			TFTSPI_Road(0, 0, MT9V03X_H, MT9V03X_W, (uint8_t*)Image_Use);      //ͼ����ʾ
  //		}
  //
  //	}
  //}





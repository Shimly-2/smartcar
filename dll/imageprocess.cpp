#include "stdafx.h"
#include<stdio.h>
#include "test.h"
/*!
  * @file     LQ_ImageProcess.c
  *
  * @brief    ͼ����
  *
  * @company  �����������ܿƼ�
  *
  * @author   LQ-005
  *
  * @note     Tab�� 4���ո�
  *
  * @version  V1.0
  *
  * @par URL  http://shop36265907.taobao.com
  *           http://www.lqist.cn
  *
  * @date     2020��6��9��
  */


/**  @brief    ������  */
#define ROAD_MAIN_ROW      40

/**  @brief    ʹ����ʼ��120  */
#define ROAD_START_ROW     119

/**  @brief    ʹ�ý�����10  */
#define ROAD_END_ROW       1

#define turn_big           40  //����������ֵ���40�ж�Ϊǰ���д���


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


//motor[0]:���ת�ǣ�motor[1]:�������
float* control(float omega, uint8_t cha,float motor[2], uint8_t mode)
{
	if (mode == 0)  //ֱ��ģʽ�����ת���������в�ֵ���������Ϊ0
	{
		motor[0] = atan(cha / 1.5 / ROAD_MAIN_ROW) * 180.0 / 3.1416;
		motor[1] = 1;
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
uint8_t Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W],unsigned char mode)
{
	int i = 0, j = 0;
	int Threshold = 0;
	unsigned long  tv = 0;
	char txt[16];

	if (mode == 0)
	{
		Threshold = GetOSTU(image_in);//�����ֵ
		return Threshold;
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
		return Threshold;
	}
	else if (mode == 2)
	{
		Threshold = 50;
		//�ֶ�������ֵ
		SobelThreshold(image_in, (uint8_t)Threshold);
		return 1;

	}
	else if (mode == 3)
	{
		SobelAutoThreshold(image_in);  //��̬������ֵ
		return 1;
	}
	else if (mode == 4)
	{
		Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
		//return Threshold ;
	}
	/* ��ֵ�� */
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = 0; j < MT9V03X_W; j++)
		{
			if (image_in[i][j] > Threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
			{
				image_in[i][j] = 255;
			}
			else
			{
				image_in[i][j] = 1;
			}
		}
	}
	return 1;
}

// TODO
//uint8_t* Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W], uint8_t image_out[MT9V03X_H][MT9V03X_W], unsigned char mode)
//{
//	int i = 0, j = 0;
//	int Threshold = 0;
//	unsigned long  tv = 0;
//	char txt[16];
//
//	if (mode == 0)
//	{
//		Threshold = GetOSTU(image_in);//�����ֵ
//		//return Threshold;
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
//		//return Threshold;
//	}
//	else if (mode == 2)
//	{
//		Threshold = 50;
//		//�ֶ�������ֵ
//		SobelThreshold(image_in, (uint8_t)Threshold);
//		//return 1;
//
//	}
//	else if (mode == 3)
//	{
//		SobelAutoThreshold(image_in);  //��̬������ֵ
//		//return 1;
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
//			printf("%d��", image_in[i][j]);
//		}
//		printf("||||");
//	}
//	printf("\n\n");
//	for (i = 0; i < MT9V03X_H; i++)
//	{
//		for (j = 0; j < MT9V03X_W; j++)
//		{
//			if (image_in[i][j] > 130) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
//			{
//				//	//
//				image_out[i][j] = 255; printf("%d��", image_out[i][j]);
//			}
//			else
//			{
//				//
//				image_out[i][j] = 1; printf("%d��", image_out[i][j]);
//			}
//		}
//		printf("||||");
//	}
//	return image_out[0];
//}

int add(int num1, int num2)
{
	return num1 + num2;
}

/*!
  * @brief    ������
  * @param
  * @return
  * @date     2020/6/28 ������
  */
void TFTSPI_BinRoadSide(uint8_t imageOut[MT9V03X_H][2])
{
	int i = 0;

	for (i = 0; i < ROAD_START_ROW; i++)
	{
		//TFTSPI_Draw_Dot(imageOut[i][0], i, u16RED);
		imageOut[i][0] = 100;
		//TFTSPI_Draw_Dot(imageOut[i][1], i, u16GREEN);
		imageOut[i][1] = 200;

	}

}



/*!
  * @brief    �ж��Ƿ���ֱ��
  * @param    image �� ��ֵͼ����Ϣ
  * @return   0������ֱ���� 1��ֱ��
  * @note     ˼·�����߱��߶�����
  * @date     2020/6/23 ���ڶ�
  */
uint8_t RoadIsStraight(uint8_t imageSide[MT9V03X_H][2])
{
	uint8_t i = 0;
	uint8_t leftState = 0, rightState = 0;

	/* ������Ƿ񵥵� */
	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		if (imageSide[i][0] + 5 < imageSide[i + 1][0])
		{
			leftState = 1;
			break;
		}
	}

	/* �ұ����Ƿ񵥵� */
	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		if (imageSide[i][1] - 5 > imageSide[i + 1][1])
		{
			rightState = 1;
			break;
		}
	}

	if (leftState == 1 && rightState == 1)
	{
		return 1;
	}

	return 0;
}


/*!
  * @brief    �ж��Ƿ��ǰ�����
  * @param    image �� ��ֵͼ����Ϣ
  * @return   0�����ǣ� 1����
  * @note     ˼·��
  * @date     2020/6/23 ���ڶ�
  */
uint8_t RoadIsZebra(uint8_t image[MT9V03X_H][MT9V03X_W], uint8_t* flag)
{
	int i = 0, j = 0;
	int count = 0;

	for (i = ROAD_MAIN_ROW - 10; i > ROAD_MAIN_ROW + 10; i++)
	{
		for (j = 1; j < MT9V03X_W; j++)
		{
			if (image[i][j] == 1 && image[i][j - 1] == 0)
			{
				count++;
			}
		}
		if (count > 8)
		{
			*flag = 1;
			return 1;
		}
	}


	return 0;
}

/*!
  * @brief    �ж��Ƿ���ʮ��
  * @param    imageSide �� ͼ�������Ϣ
  * @param    flag      �� ʮ��״̬��Ϣ
  * @return   0�����ǣ� 1����
  * @note     ˼·���������߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� ��֤����ʮ��
  * @date     2020/6/23 ���ڶ�
  */
uint8_t RoadIsCross(uint8_t imageSide[MT9V03X_H][2], uint8_t* flag)
{
	int i = 0;
	uint8_t  rightState = 0;
	int start[5] = { 0, 0, 0, 0, 0 }, end[5] = { 0, 0, 0, 0, 0 };
	uint8_t count = 0;
	uint8_t index = 0;


	/* ����Ҳ���߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� */
	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		if (imageSide[i][1] == 187)
		{
			count++;
		}
		else
		{
			if (count > 10 && index < 5)
			{
				start[index] = i + count;
				end[index] = i;
				index++;
			}
			count = 0;
		}

	}

	if (index > 1)
	{
		if (end[0] - start[1] > 10)
		{
			rightState = 1;
		}
	}

	index = 0;

	/* ��������߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� */
	if (rightState == 1)
	{
		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
		{
			if (imageSide[i][0] == 0)
			{
				count++;
			}
			else
			{
				if (count > 10 && index < 5)
				{
					start[index] = i + count;
					end[index] = i;
					index++;
				}
				count = 0;
			}

		}

		if (index > 1)
		{
			if (end[0] - start[1] > 10)
			{
				*flag = 1;
				return 1;
			}
		}
	}

	return 0;

}




/*!
  * @brief    �ж��Ƿ��ǻ���
  * @param    image �� ������Ϣ
  * @param    flag  �� ����״̬��Ϣ
  * @return   0�����ǣ� 1���󻷵�  2���һ���
  * @note     ˼·��һ�������ϸ񵥵�����һ�����߾��복ͷ�����ж��� -- Ȼ��һ�����в�����  --  �����ֶ��� ��֤���л���
  * @date     2020/6/23 ���ڶ�
  */
uint8_t RoadIsRoundabout(uint8_t image[MT9V03X_H][2], uint8_t* flag)
{
	int i = 0;
	uint8_t leftState = 0, rightState = 0;
	int start[5] = { 0, 0, 0, 0, 0 }, end[5] = { 0, 0, 0, 0, 0 };
	uint8_t count = 0;
	uint8_t index = 0;

	/* ������Ƿ񵥵� */
	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		if (image[i][0] + 5 < image[i + 1][0])
		{
			leftState = 1;
			break;
		}
	}

	/* �ұ����Ƿ񵥵� */
	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		if (image[i][1] - 5 > image[i + 1][1])
		{
			rightState = 1;
			break;
		}
	}

	/* ��ߵ����� ����Ҳ��Ƿ��ǻ��� */
	if (leftState == 0)
	{
		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
		{
			if (image[i][1] == 187)
			{
				count++;
			}
			else
			{
				if (count > 10 && index < 5)
				{
					start[index] = i + count;
					end[index] = i;
					index++;
				}
				count = 0;
			}

		}

		if (index > 1)
		{
			if (end[0] - start[1] > 10)
			{
				*flag = 2;
				return 2;
			}
		}
	}

	/* �ұߵ����� �������Ƿ��ǻ��� */
	if (rightState == 0)
	{
		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
		{
			if (image[i][0] == 0)
			{
				count++;
			}
			else
			{
				if (count > 10 && index < 5)
				{
					start[index] = i + count;
					end[index] = i;
					index++;
				}
				count = 0;
			}

		}

		if (index > 1)
		{
			if (end[0] - start[1] > 10)
			{
				*flag = 1;
				return 1;
			}
		}
	}

	return 0;
}


/*!
  * @brief    ��ȡ��������
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    status     �� 1���󻷵�  2���һ���
  * @return
  * @note     ˼·������һ�߱����ϸ񵥵�������һ�߱��ߣ���ȡ��һ����
  * @date     2020/6/23 ���ڶ�
  */
void RoundaboutGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t status)
{
	int i = 0, j = 0;

	switch (status)
	{

		/* �󻷵� */
	case 1:
	{
		/* ����ȷ����߽� */
		for (i = ROAD_START_ROW; i > ROAD_END_ROW; i--)
		{
			for (j = MT9V03X_W / 2; j > 0; j--)
			{
				if (imageInput[i][j])
				{
					imageSide[i][0] = j;
					break;
				}
			}
		}
		break;
	}

	case 2:
	{
		/* ����ȷ���ұ߽� */
		for (i = ROAD_START_ROW; i > ROAD_END_ROW; i--)
		{
			for (j = MT9V03X_W / 2; j < MT9V03X_W; j++)
			{
				if (imageInput[i][j])
				{
					imageSide[i][1] = j;
					break;
				}
			}
		}
		break;
	}
	}


}

/*!
  * @brief    �жϻ��������Ƿ���ڻ���
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    status     �� 1���󻷵�  2���һ���
  * @return
  * @note
  * @date     2020/6/23 ���ڶ�
  */
uint8_t RoundaboutGetArc(uint8_t imageSide[MT9V03X_H][2], uint8_t status, uint8_t* index)
{
	int i = 0;
	uint8_t inc = 0, dec = 0;
	switch (status)
	{
	case 1:
		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
		{
			if (imageSide[i][0] != 0 && imageSide[i + 1][0] != 0)
			{
				if (imageSide[i][0] >= imageSide[i + 1][0])
				{
					inc++;
				}
				else
				{
					dec++;
				}

				/* �л��� */
				if (inc > 5 && dec > 5)
				{
					*index = i + 5;
					return 1;
				}
			}
			else
			{
				inc = 0;
				dec = 0;
			}
		}

		break;

	case 2:
		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
		{
			if (imageSide[i][1] != 187 && imageSide[i + 1][1] != 187)
			{
				if (imageSide[i][1] > imageSide[i + 1][1])
				{
					inc++;
				}
				else
				{
					dec++;
				}

				/* �л��� */
				if (inc > 5 && dec > 5)
				{
					*index = i + 5;
					return 1;
				}
			}
			else
			{
				inc = 0;
				dec = 0;
			}
		}

		break;
	}

	return 0;
}

/*!
  * @brief    ���ߴ���
  * @param    imageSide  : ����
  * @param    status     : 1������߲���   2���ұ��߲���
  * @param    startX     : ��ʼ�� ����
  * @param    startY     : ��ʼ�� ����
  * @param    endX       : ������ ����
  * @param    endY       : ������ ����
  * @return
  * @note     endY һ��Ҫ���� startY
  * @date     2020/6/24 ������
  */
void ImageAddingLine(uint8_t imageSide[MT9V03X_H][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
{
	int i = 0;

	/* ֱ�� x = ky + b*/
	float k = 0.0f, b = 0.0f;
	switch (status)
	{
	case 1:
	{
		k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
		b = (float)startX - (float)startY * k;

		for (i = startY; i < endY; i++)
		{
			imageSide[i][0] = (uint8_t)(k * i + b);
		}
		break;
	}

	case 2:
	{
		k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
		b = (float)startX - (float)startY * k;

		for (i = startY; i < endY; i++)
		{
			imageSide[i][1] = (uint8_t)(k * i + b);
		}
		break;
	}

	}
}

/*!
  * @brief    Ѱ�������
  * @param    imageSide   �� ��������
  * @param    status      ��1����߽�   2���ұ߽�
  * @return
  * @note
  * @date     2020/6/24 ������
  */
uint8_t ImageGetHop(uint8_t imageSide[MT9V03X_H][2], uint8_t state, uint8_t* x, uint8_t* y)
{
	int i = 0;
	uint8_t px = 0, py = 0;
	uint8_t count = 0;
	switch (state)
	{
	case 1:
		/* Ѱ������� */
		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
		{
			if (imageSide[i][0] == 0 && i > (ROAD_END_ROW + 5))
			{
				count++;

				if (count > 5)
				{
					if (imageSide[i - 1][0] > (imageSide[i][0] + 20))
					{
						py = i - 1;
						px = imageSide[py][0];
						break;
					}
					if (imageSide[i - 2][0] > (imageSide[i - 1][0] + 20))
					{
						py = i - 2;
						px = imageSide[py][0];
						break;
					}
					if (imageSide[i - 3][0] > (imageSide[i - 2][0] + 20))
					{
						py = i - 3;
						px = imageSide[py][0];
						break;
					}
					if (imageSide[i - 4][0] > (imageSide[i - 3][0] + 20))
					{
						py = i - 4;
						px = imageSide[py][0];
						break;
					}

				}

			}
			else
			{
				count = 0;
			}
		}

		if (py != 0)
		{
			*x = px;
			*y = py;
			return 1;
		}

		break;


	case 2:
		/* Ѱ������� */
		for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
		{
			if (imageSide[i][1] == 187 && i > (ROAD_END_ROW + 5))
			{
				count++;

				if (count > 5)
				{
					if (imageSide[i - 1][1] < (imageSide[i][1] - 20))
					{
						py = i - 1;
						px = imageSide[py][1];
						break;
					}
					if (imageSide[i - 2][1] < (imageSide[i - 1][1] - 20))
					{
						py = i - 2;
						px = imageSide[py][1];
						break;
					}
					if (imageSide[i - 3][1] < (imageSide[i - 2][1] - 20))
					{
						py = i - 3;
						px = imageSide[py][1];
						break;
					}
					if (imageSide[i - 4][1] < (imageSide[i - 3][1] - 20))
					{
						py = i - 4;
						px = imageSide[py][1];
						break;
					}

				}

			}
			else
			{
				count = 0;
			}
		}

		if (py != 0)
		{
			*x = px;
			*y = py;
			return 1;
		}

		break;
	}

	return 0;

}


/*!
  * @brief    �������ߴ���
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageSide  �� ��������
  * @param    status     ��������־λ   1�������󻷵�   2�������һ���   3���󻷵������뻷  4���һ��������뻷  5���󻷵���������  6���һ�����������
  * @return
  * @note     ����ֻд���󻷵����һ�����ҿ��Բο��󻷵��Լ�����
  * @date     2020/6/24 ������
  */
void RoundaboutProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t* state)
{
	int i = 0;
	uint8_t pointX = 0, pointY = 0;
	uint8_t err = 0;
	static uint8_t cnt = 0;
	switch (*state)
	{
		/* �����󻷵� �������ڴ����� */
	case 1:

		/* ����ȷ����߽� */
		RoundaboutGetSide(imageInput, imageSide, 1);

		/* ��黡�� */
		err = RoundaboutGetArc(imageSide, 1, &pointY);

		/* �л��� ���в��� ���ӻ������ҵ� �� ͼ�����½� */
		if (err)
		{
			pointX = imageSide[pointY][0];

			/* ׼���뻷�� */
			if ((pointY + 10) > ROAD_MAIN_ROW)
			{
				*state = 3;
			}
		}
		else
		{
			pointY = ROAD_START_ROW - 1;

			/* û�л��� ���в��� ���ӱ������ҵ� �� ͼ�����½� */
			for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
			{
				if (imageSide[i][0] > imageSide[pointY][0])
				{
					pointY = i;
				}
			}

			pointX = imageSide[pointY][0];
		}

		/* ���� */
		ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);

		break;

		/* �����󻷵� �������ڴ����� */
	case 2:

		*state = 0;
		break;


		/* ׼�����뻷���� ֱ�����ֲ��� */
	case 3:
		pointY = ROAD_START_ROW - 1;

		/* ����ȷ����߽� */
		RoundaboutGetSide(imageInput, imageSide, 1);

		/* ��黡�� */
		err = RoundaboutGetArc(imageSide, 1, &pointY);

		/* �л��� ���в��� ���ӻ������ҵ� �� ͼ�����½� */
		if (err)
		{
			pointX = imageSide[pointY][0];

			if ((pointY + 10) > ROAD_MAIN_ROW)
			{
				/* �������ڲ��� */
				ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
			}
		}

		/* ������ڲ��� */
		pointX = MT9V03X_W / 3;
		pointY = ROAD_START_ROW - 1;

		/* Ѱ������� */
		ImageGetHop(imageSide, 1, &pointX, &pointY);

		if (pointY >= ROAD_MAIN_ROW && pointY != ROAD_START_ROW - 1)
		{
			imageSide[ROAD_MAIN_ROW][0] = 0;
			*state = 5;
		}

		/* ���� */
		ImageAddingLine(imageSide, 2, pointX + 30, pointY, (MT9V03X_W - 1), ROAD_START_ROW);

		break;

	case 4:

		break;

		/* �������� ֱ�������� */
	case 5:


		/* ��黡�� */
		err = RoundaboutGetArc(imageSide, 2, &pointY);

		if (err || cnt)
		{
			cnt++;

			imageSide[ROAD_MAIN_ROW][0] = 0;
			imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W / 3;
		}

		if (cnt > 20)
		{
			cnt = 0;
			*state = 0;

		}

		break;

		/* �������� ֱ�������� */
	case 6:

		break;

	}
}

/*!
  * @brief    ��ȡʮ�ֱ���
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @return
  * @note     ˼·�����м�����������
  * @date     2020/6/23 ���ڶ�
  */
void CrossGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2])
{
	int i = 0, j = 0;

	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		for (j = 78; j > 1; j--)
		{
			if (imageInput[i][j])
			{
				imageSide[i][0] = j;
				break;
			}
		}

		for (j = 78; j < 187; j++)
		{
			if (imageInput[i][j])
			{
				imageSide[i][1] = j;
				break;
			}
		}
	}

}

/*!
  * @brief    ʮ�ֲ��ߴ���
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageSide  �� ��������
  * @param    status     ��ʮ�ֱ�־λ   1������ʮ��    2������ʮ��   3����ʮ��
  * @return
  * @note
  * @date     2020/6/24 ������
  */
void CrossProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t* state)
{

	uint8_t pointX = 0, pointY = 0;
	uint8_t leftIndex = 0;
	static uint8_t count = 0;
	switch (*state)
	{
	case 1:
	{
		/* ���»�ȡ���� */
		CrossGetSide(imageInput, imageSide);

		/* Ѱ������� */
		if (ImageGetHop(imageSide, 1, &pointX, &pointY))
		{
			/* ���� */
			ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
		}

		leftIndex = pointY;
		pointX = 0;
		pointY = 0;

		/* Ѱ������� */
		if (ImageGetHop(imageSide, 2, &pointX, &pointY))
		{
			/* ���� */
			ImageAddingLine(imageSide, 2, pointX, pointY, (MT9V03X_W - 1), ROAD_START_ROW);
		}

		if (leftIndex != 0 && pointY != 0 && leftIndex >= ROAD_MAIN_ROW && pointY >= ROAD_MAIN_ROW)
		{
			*state = 2;
			count = 0;
		}

		if (count++ > 20)
		{
			*state = 2;
			count = 0;
		}

		break;
	}

	case 2:
	{

		/* ��黡�� */
		if (RoundaboutGetArc(imageSide, 1, &leftIndex))
		{
			/* ����ȷ����߽� */
			RoundaboutGetSide(imageInput, imageSide, 1);

			if (ImageGetHop(imageSide, 1, &pointX, &pointY))
			{
				/* ���� */
				ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);

				*state = 3;

				count = 0;
			}
			else
			{
				imageSide[ROAD_MAIN_ROW][0] = MT9V03X_W / 2;
				imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W - 1;
			}
		}

		break;
	}

	case 3:
	{

		/* ����ȷ����߽� */
		RoundaboutGetSide(imageInput, imageSide, 1);


		if (ImageGetHop(imageSide, 1, &pointX, &pointY))
		{
			/* ��黡�� */
			if (RoundaboutGetArc(imageSide, 1, &leftIndex))
			{
				/* ���� */
				ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);
			}
			else
			{
				/* ���� */
				ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
			}

			if (pointY >= ROAD_MAIN_ROW)
			{
				*state = 0;
				count = 0;
			}
		}
		else
		{
			imageSide[ROAD_MAIN_ROW][0] = 120;
			imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W - 1;
		}

		if (count++ > 10)
		{
			*state = 0;
			count = 0;
		}

		break;
	}
	}

}

/*!
  * @brief    ͣ���ߴ���
  * @param    imageSide  �� ��������
  * @param    state      �� ͣ��״̬  1�����������   2���������Ҳ�
  * @param    speed      �� �ٶ�
  * @return
  * @note
  * @date     2020/6/24 ������
  */
void ZebraProcess(uint8_t imageSide[MT9V03X_H][2], uint8_t state, int16_t* speed)
{
	static uint16_t count = 0;

	count++;

	if (state == 1)
	{
		imageSide[ROAD_MAIN_ROW][0] = 0;
		imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W / 2;
	}
	else
	{
		imageSide[ROAD_MAIN_ROW][0] = MT9V03X_W / 2;
		imageSide[ROAD_MAIN_ROW][1] = MT9V03X_W - 1;
	}

	if (count > 100)
	{
		*speed = 0;
		while (1);
	}

}

/*!
  * @brief    ���������У���ȡ���ƫ��
  * @param
  * @return
  * @note
  * @date     2020/6/24 ������
  */
int16_t RoadGetSteeringError(uint8_t imageSide[MT9V03X_H][2], uint8_t lineIndex)
{

	return imageSide[lineIndex][0] + imageSide[lineIndex][1] - 158;

}

typedef struct Side
{
	uint8_t arr_left[120];
	uint8_t arr_right[120];
}Side, * Sidepoint;
Sidepoint p = (Sidepoint)malloc(sizeof(Side));
/*!
  * @brief    �ж��Ƿ���
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    lineIndex  �� ��
  * @return   0��û�ж���   1:��߶���  2���ұ߶���  3�� ���Ҷ�����   4������
  * @note
  * @date     2020/6/24 ������
  */
uint8_t RoadIsNoSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2], uint8_t lineIndex)
{
	uint8_t state = 0;
	int i = 0;
	static uint8_t last = 78;

	imageOut[lineIndex][0] = 0;
	imageOut[lineIndex][1] = 187;
	/* �þ���С���ȽϽ����У� �ж��Ƿ��� */
	for (i = last; i > 1; i--)
	{
		if (imageInput[lineIndex][i])
		{
			imageOut[lineIndex][0] = i;
			break;
		}
	}

	if (i == 1)
	{
		/* ��߽綪�� */
		state = 1;
	}


	for (i = last; i < 187; i++)
	{
		if (imageInput[lineIndex][i])
		{
			imageOut[lineIndex][1] = i;
			break;
		}
	}

	if (i == 187)
	{
		/* ���ұ߽綪�� */
		if (state == 1)
		{
			state = 3;
		}

		/* �ұ߽綪�� */
		else
		{
			state = 2;
		}

	}

	//    if(imageOut[lineIndex][0] > last - 50)
	//    {
	//        if(last + 50 < MT9V03X_W)
	//        {
	//            last += 50;
	//        }
	//        else
	//        {
	//            last = MT9V03X_W - 5;
	//        }
	//    }
	//    else if(imageOut[lineIndex][1] < last + 50)
	//    {
	//        if(last - 50 > 0)
	//        {
	//            last -= 50;
	//        }
	//        else
	//        {
	//            last = 5;
	//        }
	//    }
	//    else
	//    {
	//        last = 78;
	//    }

	if (imageOut[lineIndex][1] <= imageOut[lineIndex][0])
	{
		state = 4;
	}
	return state;

}


/*!
  * @brief    ���ߴ���
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    mode       �� �Ǳ߶��ߣ�   1����߶���  2���ұ߶���
  * @param    lineIndex  �� ��������
  * @return
  * @note
  * @date     2020/6/24 ������
  */
void RoadNoSideProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2], uint8_t mode, int lineIndex)
{
	int i = 0, j = 0, count = 0;

	switch (mode)
	{
	case 1:
		for (i = imageOut[lineIndex][1]; i > 1; i--)
		{
			count++;
			for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
			{
				if (imageInput[j][i])
				{
					imageOut[lineIndex - count][0] = 0;
					imageOut[lineIndex - count][1] = i;
					break;
				}

			}
		}
		break;


	case 2:
		for (i = imageOut[lineIndex][0]; i < 187; i++)
		{
			count++;
			for (j = lineIndex; j > ROAD_END_ROW && lineIndex > count; j--)
			{
				if (imageInput[j][i])
				{
					imageOut[lineIndex - count][0] = i;
					imageOut[lineIndex - count][1] = 187;
					break;
				}

			}
		}
		break;

	}

}


uint8_t* test(uint8_t image_in[MT9V03X_H][MT9V03X_W],uint8_t image_out[MT9V03X_H][MT9V03X_W])
{
	for (int i = 0; i < MT9V03X_H; i++)
	{
		for (int j = 0; j < MT9V03X_W; j++)
		{
			image_out[i][j] = 100;
		}
	}
	return image_out[0];
}


/*!
  * @brief    ��ȡ����
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @return   �Ƿ���
  * @note     ˼·���Ӿ��복ͷ�Ͻ����п�ʼ���м�����������
  * @date     2020/6/23 ���ڶ�
  */
  uint8_t* ImageGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2])
  {
  	int i = 0, j = 0;
	//for (int i = 0; i < 120; i++)
	//{
	//	imageOut[i][0] = 1;   //0
	//	imageOut[i][1] = 1;
	//}
  
  	RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);
  
  	/* �복ͷ����40�У�Ѱ�ұ��� */
  	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
  	{
  		imageOut[i][0] = 0;   //0
  		imageOut[i][1] = 187;
  
  		/* ���ݱ߽��������� Ѱ�ұ߽� */
  		for (j = imageOut[i + 1][0] + 10; j > 0; j--)
  		{
  			if (imageInput[i][j])
  			{
  				imageOut[i][0] = j;
  				break;
  			}
  		}
  
  		for (j = imageOut[i + 1][1] - 10; j < 160; j++)
  		{
  			if (imageInput[i][j])
  			{
  				imageOut[i][1] = j;
  				break;
  			}
  		}
  
  		/* �����߽� �����������ߣ� �����Ƿ��Ҷ��� */
  		if (imageOut[i][0] > (MT9V03X_W / 2 - 10) && imageOut[i][1] > (MT9V03X_W - 5))
  		{
  			/* �Ҷ��ߴ��� */
  			RoadNoSideProcess(imageInput, imageOut, 2, i);
  
  			if (i > 70)
  			{
  				imageOut[i][0] += 50;
  			}
  			//return 1;
  		}
  
  		/* ����ұ߽� �����������ߣ� �����Ƿ����� */
  		if (imageOut[i][1] < (MT9V03X_W / 2 + 10) && imageOut[i][0] < (5))
  		{
  			/* ���ߴ��� */
  			RoadNoSideProcess(imageInput, imageOut, 1, i);
  
  			if (i > 70)
  			{
  				imageOut[i][1] -= 50;
  			}
  			//return 2;
  
  		}
  	}
	for (int i = 0; i < 120; i++)
	{
		if (imageOut[i][0] == 0) {
			imageOut[i][0] = 1;
		}
		if (imageOut[i][0] >= 187) {
			imageOut[i][0] = 187;
		}
		if (imageOut[i][1] == 0) {
			imageOut[i][1] = 1;
		}
		if (imageOut[i][1] >= 187) {
			imageOut[i][1] = 187;
		}
		//imageOut[i][1] = 110;
	}
  	return imageOut[0];
  }

//uint8_t ImageGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2])
//{
//	int i = 0, j = 0;
//
//	RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);
//
//	/* �복ͷ����40�У�Ѱ�ұ��� */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		imageOut[i][0] = 0;
//		imageOut[i][1] = 187;
//
//		/* ���ݱ߽��������� Ѱ�ұ߽� */
//		for (j = imageOut[i + 1][0] + 10; j > 0; j--)
//		{
//			if (imageInput[i][j])
//			{
//				imageOut[i][0] = j;
//				break;
//			}
//		}
//
//		for (j = imageOut[i + 1][1] - 10; j < 160; j++)
//		{
//			if (imageInput[i][j])
//			{
//				imageOut[i][1] = j;
//				break;
//			}
//		}
//
//		/* �����߽� �����������ߣ� �����Ƿ��Ҷ��� */
//		if (imageOut[i][0] > (MT9V03X_W / 2 - 10) && imageOut[i][1] > (MT9V03X_W - 5))
//		{
//			/* �Ҷ��ߴ��� */
//			RoadNoSideProcess(imageInput, imageOut, 2, i);
//
//			if (i > 70)
//			{
//				imageOut[i][0] += 50;
//			}
//			return 1;
//		}
//
//		/* ����ұ߽� �����������ߣ� �����Ƿ����� */
//		if (imageOut[i][1] < (MT9V03X_W / 2 + 10) && imageOut[i][0] < (5))
//		{
//			/* ���ߴ��� */
//			RoadNoSideProcess(imageInput, imageOut, 1, i);
//
//			if (i > 70)
//			{
//				imageOut[i][1] -= 50;
//			}
//			return 2;
//
//		}
//	}
//
//	return 0;
//}


/*!
  * @brief    ����һ�����
  * @param
  * @return
  * @note     ˼·�� �����������ڵ�9���㣬�����������ֵ�������õ�
  * @date     2020/6/24 ������
  */
void ImagePortFilter(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][MT9V03X_W])
{
	uint8_t temp = 0;

	for (int i = 1; i < MT9V03X_H - 1; i++)
	{
		for (int j = 1; j < MT9V03X_W - 1; j++)
		{
			temp = imageInput[i - 1][j - 1] + imageInput[i - 1][j] + imageInput[i - 1][j + 1] +
				imageInput[i][j - 1] + imageInput[i][j] + imageInput[i][j + 1] +
				imageInput[i + 1][j - 1] + imageInput[i + 1][j] + imageInput[i + 1][j + 1];

			/* ������5�����Ǳ��� �����õ� ���Ե��������Ż��˲�Ч�� */
			if (temp > 4)
			{
				imageOut[i][j] = 1;
			}
			else
			{
				imageOut[i][j] = 0;
			}

		}
	}
}


/*!
  * @brief    ��������
  * @note
  */
uint8_t ImageSide[MT9V03X_H][2];


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
  //			char txt[32];
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





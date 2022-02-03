#include "stdafx.h"
#include<stdio.h>
#include "test.h"
/*!
  * @file     LQ_ImageProcess.c
  *
  * @brief    图像处理
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


/**  @brief    主跑行  */
#define ROAD_MAIN_ROW      40

/**  @brief    使用起始行120  */
#define ROAD_START_ROW     119

/**  @brief    使用结束行10  */
#define ROAD_END_ROW       1

#define turn_big           40  //主跑行于中值相差40判定为前方有大弯


/**  @brief    环岛标志位  */
uint8_t g_ucFlagRoundabout = 0;

/**  @brief    十字标志位  */
uint8_t g_ucFlagCross = 0;

/**  @brief    斑马线标志位  */
uint8_t g_ucFlagZebra = 0;

const motorbike motor_contorl = {
	0.189,      //车轮距 m
	0.03375,    //车轮半径 m
	0.07,       //质心高度 m
	0.09425,    //质心投影与P1距离 m
	0.7,        //车体质量 kg
	0.5,        //跑道环路与转弯半径 m
	9.794,      //重力加速度 m/s^2
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
 * @brief 开发版参数
 */
const xlwParamStructure paramTrace = {
	5,          /* 被识别为信标灯的最小像素个数 */
	28,         /* 不计入像素的上方画面的高度 */
	1,			/* 连续丢失目标导致判断为丢失信标灯的次数上限 */
	60,			/* 正常车速的最大信标灯尺寸（最远，像素最少） */
	80,			/* 第一档车速的最大信标灯尺寸 */
	100,		/* 第二档... */
	500,        /* 正常车速 */
	450,        /* 第一档车速 */
	300,        /* 第二档车速 */
	200,        /* 第三档车速 */
	10.3f,      /* 发车电压阈值 */
	-50,		/* 寻灯速度左值 */
	200,		/* 寻灯速度右值 */
	0          /* 二值化阈值 */
};


//motor[0]:舵机转角，motor[1]:车体倾角
float* control(float omega, uint8_t cha,float motor[2], uint8_t mode)
{
	if (mode == 0)  //直道模式，舵机转角由主跑行差值决定，倾角为0
	{
		motor[0] = atan(cha / 1.5 / ROAD_MAIN_ROW) * 180.0 / 3.1416;
		motor[1] = 1;
	}
	if (mode == 1)  //检测到环岛的处理方式
	{
		float op1 = 0.0, Va = 0.0;
		op1 = sqrt(motor_contorl.R * motor_contorl.R - motor_contorl.b * motor_contorl.b);
		Va = motor_contorl.R * omega;  //Va=V ?
		motor[0] = atan(motor_contorl.w / op1) * 180.0 / 3.1416;
		motor[1] = atan(motor_contorl.R * motor_contorl.g / (Va * Va)) * 180.0 / 3.1416;
	}
	if (mode == 2)  //大弯模式，主跑行差值大于一定条件实现
	{
		float op1 = 0.0, Va = 0.0;
		op1 = sqrt(motor_contorl.R * motor_contorl.R - motor_contorl.b * motor_contorl.b);
		Va = motor_contorl.R * omega;  //Va=V ?
		motor[0] = atan(motor_contorl.w / op1) * 180.0 / 3.1416;
		motor[1] = atan(motor_contorl.R * motor_contorl.g / (Va * Va)) * 180.0 / 3.1416;
		//退出条件不同
	}
	if (mode == 3)  //入库模式
	{
		motor[0] = 30;  //最大值
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
 * @brief OTSU大津法二值化计算阈值
 * @param image (uint8_t*)图像起始指针
 * @param col   (uint16_t)列数
 * @param row   (uint16_t)行数
 * @return (uint8_t)计算得出的阈值
 */
uint8_t XLW_otsuThreshold(uint8_t image[MT9V03X_H][MT9V03X_W], uint16_t col, uint16_t row)
{
	/* 定义灰度调整等级：128档 */
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

	//统计灰度级中每个像素在整幅图像中的个数
	for (i = paramTrace.upperCutLine; i < height; i += 2)
	{
		for (j = 0; j < width; j += 2)
		{
			pixelCount[(int)data[i * width + j] / 2]++; //将像素值作为计数数组的下标
		}
	}

	//计算每个像素在整幅图像中的比例
	float maxPro = 0.0;
	for (i = 0; i < GrayScale; i++)
	{
		pixelPro[i] = (float)pixelCount[i] / pixelSum;
		if (pixelPro[i] > maxPro)
		{
			maxPro = pixelPro[i];
		}
	}

	//遍历灰度级[0,255]
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (i = 0; i < GrayScale; i++) // i作为阈值
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
		for (j = 0; j < GrayScale; j++)
		{
			if (j <= i) //背景部分
			{
				w0 += pixelPro[j];
				u0tmp += j * pixelPro[j];
			}
			else //前景部分
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
  * @brief    大津法求阈值大小(龙邱)
  * @param    tmImage ： 图像数据
  * @return   阈值
 * @note     Ostu方法又名最大类间差方法，通过统计整个图像的直方图特性来实现全局阈值T的自动选取，其算法步骤为：
  * @note     1) 先计算图像的直方图，即将图像所有的像素点按照0~255共256个bin，统计落在每个bin的像素点数量
  * @note     2) 归一化直方图，也即将每个bin中像素点数量除以总的像素点
  * @note     3) i表示分类的阈值，也即一个灰度级，从0开始迭代	1
  * @note     4) 通过归一化的直方图，统计0~i 灰度级的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例w0，并统计前景像素的平均灰度u0；统计i~255灰度级的像素(假设像素值在此范围的像素叫做背景像素) 所占整幅图像的比例w1，并统计背景像素的平均灰度u1；
  * @note     5) 计算前景像素和背景像素的方差 g = w0*w1*(u0-u1) (u0-u1)
  * @note     6) i++；转到4)，直到i为256时结束迭代
  * @note     7) 将最大g相应的i值作为图像的全局阈值
  * @note     缺陷:OSTU算法在处理光照不均匀的图像的时候，效果会明显不好，因为利用的是全局像素信息。
  * @note     解决光照不均匀  https://blog.csdn.net/kk55guang2/article/details/78475414
  * @note     https://blog.csdn.net/kk55guang2/article/details/78490069
  * @note     https://wenku.baidu.com/view/84e5eb271a37f111f0855b2d.html
  * @see      GetOSTU(Image_Use);//大津法阈值
  * @date     2019/6/25 星期二
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
	float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差; 
	signed short MinValue, MaxValue;
	signed short Threshold = 0;
	unsigned char HistoGram[256];              //

	for (j = 0; j < 256; j++)  HistoGram[j] = 0; //初始化灰度直方图 

	for (j = 0; j < MT9V03X_H; j++)
	{
		for (i = 0; i < MT9V03X_W; i++)
		{
			HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
		}
	}

	for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
	for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

	if (MaxValue == MinValue)     return MaxValue;         // 图像中只有一个颜色    
	if (MinValue + 1 == MaxValue)  return MinValue;        // 图像中只有二个颜色

	for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  像素总数

	PixelIntegral = 0;
	for (j = MinValue; j <= MaxValue; j++)
	{
		PixelIntegral += HistoGram[j] * j;//灰度值总数
	}
	SigmaB = -1;
	for (j = MinValue; j < MaxValue; j++)
	{
		PixelBack = PixelBack + HistoGram[j];   //前景像素点数
		PixelFore = Amount - PixelBack;         //背景像素点数
		OmegaBack = (float)PixelBack / Amount;//前景像素百分比
		OmegaFore = (float)PixelFore / Amount;//背景像素百分比
		PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
		MicroBack = (float)PixelIntegralBack / PixelBack;   //前景灰度百分比
		MicroFore = (float)PixelIntegralFore / PixelFore;   //背景灰度百分比
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
		if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
		{
			SigmaB = Sigma;
			Threshold = j;
		}
	}
	return Threshold;                        //返回最佳阈值;
}


/**
  * @brief    基于soble边沿检测算子的一种边沿检测
  * @param    imageIn    输入数组
  * @param    imageOut   输出数组      保存的二值化后的边沿信息
  * @param    Threshold  阈值
  * @return
  * @note
  * @date     2020/5/15
  */
void SobelThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W], uint8_t Threshold)
{
	/** 卷积核大小 */
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
			/* 计算不同方向梯度幅值  */
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

			/* 找出梯度幅值最大值  */
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
  * @brief    基于soble边沿检测算子的一种自动阈值边沿检测
  * @param    imageIn    输入数组
  * @param    imageOut   输出数组      保存的二值化后的边沿信息
  * @return
  * @note
  * @date     2020/5/15
  */
void SobelAutoThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W])
{
	/** 卷积核大小 */
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
			/* 计算不同方向梯度幅值  */
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

			/* 找出梯度幅值最大值  */
			for (k = 1; k < 4; k++)
			{
				if (temp[0] < temp[k])
				{
					temp[0] = temp[k];
				}
			}

			/* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
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
  * @brief    二值化
  * @param    mode ：0：使用龙邱大津法阈值,1：使用平均阈值,2: sobel 算子改进型-手动阈值,3：sobel 算子改进型-动态阈值,4：OTSU
  * @return   无
  * @note     无
  * @see      Get_01_Value(0); //使用大津法二值化
  * @date     2020/5/15 星期二
  */
uint8_t Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W],unsigned char mode)
{
	int i = 0, j = 0;
	int Threshold = 0;
	unsigned long  tv = 0;
	char txt[16];

	if (mode == 0)
	{
		Threshold = GetOSTU(image_in);//大津法阈值
		return Threshold;
	}
	if (mode == 1)
	{
		//累加
		for (i = 0; i < MT9V03X_H; i++)
		{
			for (j = 0; j < MT9V03X_W; j++)
			{
				tv += image_in[i][j];   //累加
			}
		}
		Threshold = tv / MT9V03X_H / MT9V03X_W;        //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
		Threshold = Threshold + 20;      //此处阈值设置，根据环境的光线来设定
		return Threshold;
	}
	else if (mode == 2)
	{
		Threshold = 50;
		//手动调节阈值
		SobelThreshold(image_in, (uint8_t)Threshold);
		return 1;

	}
	else if (mode == 3)
	{
		SobelAutoThreshold(image_in);  //动态调节阈值
		return 1;
	}
	else if (mode == 4)
	{
		Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
		//return Threshold ;
	}
	/* 二值化 */
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = 0; j < MT9V03X_W; j++)
		{
			if (image_in[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
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
//		Threshold = GetOSTU(image_in);//大津法阈值
//		//return Threshold;
//	}
//	if (mode == 1)
//	{
//		//累加
//		for (i = 0; i < MT9V03X_H; i++)
//		{
//			for (j = 0; j < MT9V03X_W; j++)
//			{
//				tv += image_in[i][j];   //累加
//			}
//		}
//		Threshold = tv / MT9V03X_H / MT9V03X_W;        //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
//		Threshold = Threshold + 20;      //此处阈值设置，根据环境的光线来设定
//		//return Threshold;
//	}
//	else if (mode == 2)
//	{
//		Threshold = 50;
//		//手动调节阈值
//		SobelThreshold(image_in, (uint8_t)Threshold);
//		//return 1;
//
//	}
//	else if (mode == 3)
//	{
//		SobelAutoThreshold(image_in);  //动态调节阈值
//		//return 1;
//	}
//	else if (mode == 4)
//	{
//		Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
//		//return Threshold ;
//	}
//	/* 二值化 */
//	for (i = 0; i < MT9V03X_H; i++)
//	{
//		for (j = 0; j < MT9V03X_W; j++)
//		{
//			printf("%d，", image_in[i][j]);
//		}
//		printf("||||");
//	}
//	printf("\n\n");
//	for (i = 0; i < MT9V03X_H; i++)
//	{
//		for (j = 0; j < MT9V03X_W; j++)
//		{
//			if (image_in[i][j] > 130) //数值越大，显示的内容越多，较浅的图像也能显示出来
//			{
//				//	//
//				image_out[i][j] = 255; printf("%d，", image_out[i][j]);
//			}
//			else
//			{
//				//
//				image_out[i][j] = 1; printf("%d，", image_out[i][j]);
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
  * @brief    画边线
  * @param
  * @return
  * @date     2020/6/28 星期日
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
  * @brief    判断是否是直道
  * @param    image ： 二值图像信息
  * @return   0：不是直道， 1：直道
  * @note     思路：两边边线都单调
  * @date     2020/6/23 星期二
  */
uint8_t RoadIsStraight(uint8_t imageSide[MT9V03X_H][2])
{
	uint8_t i = 0;
	uint8_t leftState = 0, rightState = 0;

	/* 左边线是否单调 */
	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		if (imageSide[i][0] + 5 < imageSide[i + 1][0])
		{
			leftState = 1;
			break;
		}
	}

	/* 右边线是否单调 */
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
  * @brief    判断是否是斑马线
  * @param    image ： 二值图像信息
  * @return   0：不是， 1：是
  * @note     思路：
  * @date     2020/6/23 星期二
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
  * @brief    判断是否是十字
  * @param    imageSide ： 图像边线信息
  * @param    flag      ： 十字状态信息
  * @return   0：不是， 1：是
  * @note     思路：两条边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 则证明有十字
  * @date     2020/6/23 星期二
  */
uint8_t RoadIsCross(uint8_t imageSide[MT9V03X_H][2], uint8_t* flag)
{
	int i = 0;
	uint8_t  rightState = 0;
	int start[5] = { 0, 0, 0, 0, 0 }, end[5] = { 0, 0, 0, 0, 0 };
	uint8_t count = 0;
	uint8_t index = 0;


	/* 检测右侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
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

	/* 检测左侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
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
  * @brief    判断是否是环岛
  * @param    image ： 边线信息
  * @param    flag  ： 环岛状态信息
  * @return   0：不是， 1：左环岛  2：右环岛
  * @note     思路：一条边线严格单调，另一条边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 则证明有环岛
  * @date     2020/6/23 星期二
  */
uint8_t RoadIsRoundabout(uint8_t image[MT9V03X_H][2], uint8_t* flag)
{
	int i = 0;
	uint8_t leftState = 0, rightState = 0;
	int start[5] = { 0, 0, 0, 0, 0 }, end[5] = { 0, 0, 0, 0, 0 };
	uint8_t count = 0;
	uint8_t index = 0;

	/* 左边线是否单调 */
	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		if (image[i][0] + 5 < image[i + 1][0])
		{
			leftState = 1;
			break;
		}
	}

	/* 右边线是否单调 */
	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
	{
		if (image[i][1] - 5 > image[i + 1][1])
		{
			rightState = 1;
			break;
		}
	}

	/* 左边单调， 检测右侧是否是环岛 */
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

	/* 右边单调， 检测左侧是否是环岛 */
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
  * @brief    获取环岛边线
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    status     ： 1：左环岛  2：右环岛
  * @return
  * @note     思路：环岛一边边线严格单调，根据一边边线，获取另一边线
  * @date     2020/6/23 星期二
  */
void RoundaboutGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t status)
{
	int i = 0, j = 0;

	switch (status)
	{

		/* 左环岛 */
	case 1:
	{
		/* 重新确定左边界 */
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
		/* 重新确定右边界 */
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
  * @brief    判断环岛边线是否存在弧形
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    status     ： 1：左环岛  2：右环岛
  * @return
  * @note
  * @date     2020/6/23 星期二
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

				/* 有弧线 */
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

				/* 有弧线 */
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
  * @brief    补线处理
  * @param    imageSide  : 边线
  * @param    status     : 1：左边线补线   2：右边线补线
  * @param    startX     : 起始点 列数
  * @param    startY     : 起始点 行数
  * @param    endX       : 结束点 列数
  * @param    endY       : 结束点 行数
  * @return
  * @note     endY 一定要大于 startY
  * @date     2020/6/24 星期三
  */
void ImageAddingLine(uint8_t imageSide[MT9V03X_H][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
{
	int i = 0;

	/* 直线 x = ky + b*/
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
  * @brief    寻找跳变点
  * @param    imageSide   ： 边线数组
  * @param    status      ：1：左边界   2：右边界
  * @return
  * @note
  * @date     2020/6/24 星期三
  */
uint8_t ImageGetHop(uint8_t imageSide[MT9V03X_H][2], uint8_t state, uint8_t* x, uint8_t* y)
{
	int i = 0;
	uint8_t px = 0, py = 0;
	uint8_t count = 0;
	switch (state)
	{
	case 1:
		/* 寻找跳变点 */
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
		/* 寻找跳变点 */
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
  * @brief    环岛补线处理
  * @param    imageInput ： 二值图像信息
  * @param    imageSide  ： 边线数组
  * @param    status     ：环岛标志位   1：发现左环岛   2：发现右环岛   3：左环岛即将入环  4：右环岛即将入环  5：左环岛即将出环  6：右环岛即将出环
  * @return
  * @note     这里只写了左环岛，右环岛大家可以参考左环岛自己完善
  * @date     2020/6/24 星期三
  */
void RoundaboutProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t* state)
{
	int i = 0;
	uint8_t pointX = 0, pointY = 0;
	uint8_t err = 0;
	static uint8_t cnt = 0;
	switch (*state)
	{
		/* 发现左环岛 环岛出口处补线 */
	case 1:

		/* 重新确定左边界 */
		RoundaboutGetSide(imageInput, imageSide, 1);

		/* 检查弧线 */
		err = RoundaboutGetArc(imageSide, 1, &pointY);

		/* 有弧线 进行补线 连接弧线最右点 和 图像左下角 */
		if (err)
		{
			pointX = imageSide[pointY][0];

			/* 准备入环岛 */
			if ((pointY + 10) > ROAD_MAIN_ROW)
			{
				*state = 3;
			}
		}
		else
		{
			pointY = ROAD_START_ROW - 1;

			/* 没有弧线 进行补线 连接边线最右点 和 图像左下角 */
			for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
			{
				if (imageSide[i][0] > imageSide[pointY][0])
				{
					pointY = i;
				}
			}

			pointX = imageSide[pointY][0];
		}

		/* 补线 */
		ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);

		break;

		/* 发现左环岛 环岛出口处补线 */
	case 2:

		*state = 0;
		break;


		/* 准备进入环岛， 直道部分补线 */
	case 3:
		pointY = ROAD_START_ROW - 1;

		/* 重新确定左边界 */
		RoundaboutGetSide(imageInput, imageSide, 1);

		/* 检查弧线 */
		err = RoundaboutGetArc(imageSide, 1, &pointY);

		/* 有弧线 进行补线 连接弧线最右点 和 图像左下角 */
		if (err)
		{
			pointX = imageSide[pointY][0];

			if ((pointY + 10) > ROAD_MAIN_ROW)
			{
				/* 环岛出口补线 */
				ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
			}
		}

		/* 环岛入口补线 */
		pointX = MT9V03X_W / 3;
		pointY = ROAD_START_ROW - 1;

		/* 寻找跳变点 */
		ImageGetHop(imageSide, 1, &pointX, &pointY);

		if (pointY >= ROAD_MAIN_ROW && pointY != ROAD_START_ROW - 1)
		{
			imageSide[ROAD_MAIN_ROW][0] = 0;
			*state = 5;
		}

		/* 补线 */
		ImageAddingLine(imageSide, 2, pointX + 30, pointY, (MT9V03X_W - 1), ROAD_START_ROW);

		break;

	case 4:

		break;

		/* 出环岛， 直道处补线 */
	case 5:


		/* 检查弧线 */
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

		/* 出环岛， 直道处补线 */
	case 6:

		break;

	}
}

/*!
  * @brief    获取十字边线
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @return
  * @note     思路：从中间向两边搜线
  * @date     2020/6/23 星期二
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
  * @brief    十字补线处理
  * @param    imageInput ： 二值图像信息
  * @param    imageSide  ： 边线数组
  * @param    status     ：十字标志位   1：发现十字    2：进入十字   3：出十字
  * @return
  * @note
  * @date     2020/6/24 星期三
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
		/* 重新获取边线 */
		CrossGetSide(imageInput, imageSide);

		/* 寻找跳变点 */
		if (ImageGetHop(imageSide, 1, &pointX, &pointY))
		{
			/* 补线 */
			ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
		}

		leftIndex = pointY;
		pointX = 0;
		pointY = 0;

		/* 寻找跳变点 */
		if (ImageGetHop(imageSide, 2, &pointX, &pointY))
		{
			/* 补线 */
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

		/* 检查弧线 */
		if (RoundaboutGetArc(imageSide, 1, &leftIndex))
		{
			/* 重新确定左边界 */
			RoundaboutGetSide(imageInput, imageSide, 1);

			if (ImageGetHop(imageSide, 1, &pointX, &pointY))
			{
				/* 补线 */
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

		/* 重新确定左边界 */
		RoundaboutGetSide(imageInput, imageSide, 1);


		if (ImageGetHop(imageSide, 1, &pointX, &pointY))
		{
			/* 检查弧线 */
			if (RoundaboutGetArc(imageSide, 1, &leftIndex))
			{
				/* 补线 */
				ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);
			}
			else
			{
				/* 补线 */
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
  * @brief    停车线处理
  * @param    imageSide  ： 边线数组
  * @param    state      ： 停车状态  1：车库在左侧   2：车库在右侧
  * @param    speed      ： 速度
  * @return
  * @note
  * @date     2020/6/24 星期三
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
  * @brief    根据主跑行，求取舵机偏差
  * @param
  * @return
  * @note
  * @date     2020/6/24 星期三
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
  * @brief    判断是否丢线
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    lineIndex  ： 行
  * @return   0：没有丢线   1:左边丢线  2：右边丢线  3： 左右都丢线   4：错误
  * @note
  * @date     2020/6/24 星期三
  */
uint8_t RoadIsNoSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2], uint8_t lineIndex)
{
	uint8_t state = 0;
	int i = 0;
	static uint8_t last = 78;

	imageOut[lineIndex][0] = 0;
	imageOut[lineIndex][1] = 187;
	/* 用距离小车比较近的行， 判断是否丢线 */
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
		/* 左边界丢线 */
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
		/* 左右边界丢线 */
		if (state == 1)
		{
			state = 3;
		}

		/* 右边界丢线 */
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
  * @brief    丢线处理
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    mode       ： 那边丢线？   1：左边丢线  2：右边丢线
  * @param    lineIndex  ： 丢线行数
  * @return
  * @note
  * @date     2020/6/24 星期三
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
  * @brief    获取边线
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @return   是否丢线
  * @note     思路：从距离车头较近的行开始从中间向两边搜线
  * @date     2020/6/23 星期二
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
  
  	/* 离车头近的40行，寻找边线 */
  	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
  	{
  		imageOut[i][0] = 0;   //0
  		imageOut[i][1] = 187;
  
  		/* 根据边界连续特性 寻找边界 */
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
  
  		/* 如果左边界 即将超出中线， 则检查是否右丢线 */
  		if (imageOut[i][0] > (MT9V03X_W / 2 - 10) && imageOut[i][1] > (MT9V03X_W - 5))
  		{
  			/* 右丢线处理 */
  			RoadNoSideProcess(imageInput, imageOut, 2, i);
  
  			if (i > 70)
  			{
  				imageOut[i][0] += 50;
  			}
  			//return 1;
  		}
  
  		/* 如果右边界 即将超出中线， 则检查是否左丢线 */
  		if (imageOut[i][1] < (MT9V03X_W / 2 + 10) && imageOut[i][0] < (5))
  		{
  			/* 左丢线处理 */
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
//	/* 离车头近的40行，寻找边线 */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		imageOut[i][0] = 0;
//		imageOut[i][1] = 187;
//
//		/* 根据边界连续特性 寻找边界 */
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
//		/* 如果左边界 即将超出中线， 则检查是否右丢线 */
//		if (imageOut[i][0] > (MT9V03X_W / 2 - 10) && imageOut[i][1] > (MT9V03X_W - 5))
//		{
//			/* 右丢线处理 */
//			RoadNoSideProcess(imageInput, imageOut, 2, i);
//
//			if (i > 70)
//			{
//				imageOut[i][0] += 50;
//			}
//			return 1;
//		}
//
//		/* 如果右边界 即将超出中线， 则检查是否左丢线 */
//		if (imageOut[i][1] < (MT9V03X_W / 2 + 10) && imageOut[i][0] < (5))
//		{
//			/* 左丢线处理 */
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
  * @brief    除单一的噪点
  * @param
  * @return
  * @note     思路： 检查边沿邻域内的9个点，如果大于设置值，则保留该点
  * @date     2020/6/24 星期三
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

			/* 邻域内5个点是边沿 则保留该点 可以调节这里优化滤波效果 */
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
  * @brief    边线数组
  * @note
  */
uint8_t ImageSide[MT9V03X_H][2];


/*!
  * @brief    图像数据处理
  * @param    无
  * @return
  * @note     无
  * @date     2020/4/28
  */
  //void ImageProcess(void)
  //{
  //	/* 获取一帧图像  进行图像处理 */
  //	if (Field_Over_Flag)
  //	{
  //		uint32_t nowtime = systime.get_time_us();
  //
  //		/* 从采集图像数据中取出自己想要使用的大小 */
  //		Get_Use_Image();
  //
  //		/* 开启下一幅图像传输  如果不为0，则场中断中不会开启新的DMA传输，不会得到新的图像 */
  //		Field_Over_Flag = 0;
  //
  //		/* 边沿提取 提取图像的所有边沿信息 存放在Pixle数组中 */
  //		Get_01_Value(3);
  //
  //		if (KEY_Read(1) == 0)
  //		{
  //			/* 对边沿信息进行滤波处理 去除单一噪点 */
  //			ImagePortFilter(Pixle, Image_Use);
  //
  //			/* 提取赛道边线 */
  //			g_ucIsNoSide = ImageGetSide(Image_Use, ImageSide);
  //
  //
  //			if (g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagZebra == 0)
  //			{
  //				/* 检测环岛 */
  //				RoadIsRoundabout(ImageSide, &g_ucFlagRoundabout);
  //			}
  //
  //
  //			if (g_ucFlagRoundabout)
  //			{
  //				/* 环岛处理 */
  //				RoundaboutProcess(Image_Use, ImageSide, &g_ucFlagRoundabout);
  //
  //			}
  //
  //			if (g_ucFlagRoundabout == 0 && g_ucFlagCross == 0 && g_ucFlagZebra == 0)
  //			{
  //				/* 检测十字 */
  //				RoadIsCross(ImageSide, &g_ucFlagCross);
  //			}
  //
  //			if (g_ucFlagCross)
  //			{
  //				/* 十字处理 */
  //				CrossProcess(Image_Use, ImageSide, &g_ucFlagCross);
  //			}
  //
  //			/* 根据主跑行，求取舵机偏差 */
  //			g_sSteeringError = RoadGetSteeringError(ImageSide, ROAD_MAIN_ROW);
  //
  //			nowtime = systime.get_time_us() - nowtime;
  //
  //#if 0       /* 调试时可以打开这里 */   
  //			TFTSPI_BinRoad(0, 0, MT9V03X_H, MT9V03X_W, (uint8_t*)Image_Use);   //图像显示
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
  //		/* 按键按下 显示原始图形 */
  //		else
  //		{
  //			TFTSPI_Road(0, 0, MT9V03X_H, MT9V03X_W, (uint8_t*)Image_Use);      //图像显示
  //		}
  //
  //	}
  //}





#include "stdafx.h"
#include<stdio.h>
#include "test.h"
#include "common.h"
#include "sensor_process.h"

/*!
  * @file     imageprocess.cpp
  * @brief    图像处理
  * @author   YYY
  * @version  V1.2
  * @date     2022/2/7
  */


/**  @brief    主跑行  */
#define ROAD_MAIN_ROW      40

/**  @brief    使用起始行120  */
#define ROAD_START_ROW     119

/**  @brief    使用结束行10  */
#define ROAD_END_ROW       1

#define turn_big           40  //主跑行于中值相差40判定为前方有大弯


/*******************以下是环岛识别所用到的相关全局变量 先放这里***************/
/********************以下是环岛识别所用到的相关变量先放这里*********************/
uint8 Island_flag = 0;
uint8 Big_island_flag = 0;
int16 Fir_jump_point_row;
int16 Fir_jump_point_column;
int16 Sec_jump_point_row;
int16 Sec_jump_point_column;

int16 Island_change_time = 400;//环岛状态切换延时 400ms 速度大时减小 速度小时加大

int16 Island_time;
int16 meet_time;

/**********************以下是会车区识别所用到的相关变量************************/
uint8 meet_flag = 0;
uint8 Left_car_flag_mid;
int16 Start_time;
uint8 Left_car_flag_end;
int16 Stop_ready2;


uint8 tesssss[ROW][COL];

/********************以下的全局变量，后期可优化为局部变量*********************/



/*************************以下为用于巡迹的相关全局变量*************************/
coordinate Left_line; //左边线列坐标 枚举 存左边线数组用
coordinate Right_line;//右边线列坐标 枚举 存右边线数组用
coordinate Midd_line; //中线列坐标   枚举 存中线数组用
struct PID Steer;// PID PD枚举 舵机控制用
uint8 binary_img[ROW][COL];//二值化后的图像数组
uint8 image[ROW][COL];
uint8 Left_line_lost[70]; //存储对应行左边线的 搜索情况，1 为丢，未找到                                                //40
uint8 Right_line_lost[70];//存储对应行右边线的 搜索情况，1 为丢，未找到                                                //40
uint8 Found_left_flag, Found_right_flag;//该行左、右边线黑点寻找标志位 1为找到
uint8 Near_flag;//近处处理标志 小车前端车头即将临近赛道边界 即将全丢线(近处一片黑)
uint8 Cross_road_flag;//十字 标志位
uint8 out_flag;//出界标志位
uint8 Highest_speed_flag, Mult_speed_flag, Lowest_speed_flag;//小车高速 中速 标志位
uint8 Ram_flag = 0;//坡道标志位
uint8 Thre = 130;
int16 L_lost_cnt = 0, R_lost_cnt = 0;//左、右丢单边线（点）次数  
int16 L_R_lost_cnt, Cross_road_cnt;//左右同时丢边线（点）次数  十字丢线次数，有别双边丢线次数
int16 Row_begging = 63, Row_end = 0, Last_row_end, Column_end;//扫线 起始行 结束行  上一次扫线结束行 结束列（起始列）   //63
int16 L_last_lost_row = 63, R_last_lost_row = 63;//左、右边最后丢的线的行数                                           //63
int16 L_R_lost_row = 63;//十字行数                                                                                   //63
int16 Last_mid_line = 93;//最后的 中点的行数
int16 Vertical_longest_length_1;//纵向搜索长度（最远端）
int16 old_error;//上次中线偏差
int16 row = 63;//行数                                                                                               //63
int16 Row_start = 63;//扫线起始行;                                                                                   //63
int16 Last_L_column;//上一场图像某一行的左边界列数 暂时未用到
int16 Last_R_column;//上一场图像某一行的右边界列数 暂时未用到
int16 Zebra_first_row;

float Middle_line = 93;//图像中线  可改动
float Left_slope = 0; //左边线  延伸斜率
float Right_slope = 0;//右边线 延伸斜率
float Last_left_slope = 0; //上一场图像的左边线斜率
float Last_right_slope = 0;//上一场图像的右边线斜率

float Add_weight[60] = {
							2.3,2.3,2.3,2.3,2.3,2,2,2,2,2,
							2,2,1.7,1.7,1.7,1.7,1.5,1,1,1,
							1,1,1,1,1,1,1,1,1,1,

							1,1,1,1,1,1,1,1,1,1,//最多三十行  不会用到这里后面的十个
							1,1,1,1,1,1,1,1,1,1,

							1,1,1,1,1,1,1,1,1,1,

};//加权算法 权重 调路径  后期可用分段加权 
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

//压缩二值化图像解压（空间 换 时间 解压）
//srclen 是二值化图像的占用空间大小
//【鹰眼解压】鹰眼图像解压，转为 二维数组 - 智能车资料区 - 山外论坛 http://vcan123.com/forum.php?mod=viewthread&tid=17&ctid=6
//解压的时候，里面有个数组，配置黑、白对应的值是多少。
/*void img_extract(void *dst, void *src, uint32_t srclen)
{
	uint8_t colour[2] = {1, 0}; //0 和 1 分别对应的颜色
	uint8_t * mdst = dst;
	uint8_t * msrc = src;
	//注：山外的摄像头 0 表示 白色，1表示 黑色
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
void img_extract(uint8* img, uint8* buff, uint32 length)    //解压后，解压前，原长度
{
	uint8 colour[2] = { 255, 0 };   //白,黑
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

//motor[0]:舵机转角，motor[1]:车体倾角
float* control(float omega, uint8_t cha,float motor[2], uint8_t mode)
{
	if (mode == 0)  //直道模式，舵机转角由主跑行差值决定，倾角为0
	{
		motor[0] = atan(cha / 1.5 / ROAD_MAIN_ROW) * 180.0 / 3.1416;
		motor[1] = atan(cha / 2.0 / ROAD_MAIN_ROW) * 180.0 / 3.1416;
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
//uint8_t Get_01_Value(uint8_t image_in[MT9V03X_H][MT9V03X_W],unsigned char mode)
//{
//	int i = 0, j = 0;
//	int Threshold = 0;
//	unsigned long  tv = 0;
//	char txt[16];
//
//	if (mode == 0)
//	{
//		Threshold = GetOSTU(image_in);//大津法阈值
//		return Threshold;
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
//		return Threshold;
//	}
//	else if (mode == 2)
//	{
//		Threshold = 50;
//		//手动调节阈值
//		SobelThreshold(image_in, (uint8_t)Threshold);
//		return 1;
//
//	}
//	else if (mode == 3)
//	{
//		SobelAutoThreshold(image_in);  //动态调节阈值
//		return 1;
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
//			if (image_in[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
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
		Threshold = GetOSTU(image_in);//大津法阈值
		//return Threshold;
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
		//return Threshold;
	}
	else if (mode == 2)
	{
		Threshold = 50;
		//手动调节阈值
		SobelThreshold(image_in, (uint8_t)Threshold);
		//return 1;

	}
	else if (mode == 3)
	{
		SobelAutoThreshold(image_in);  //动态调节阈值
		//return 1;
	}
	else if (mode == 4)
	{
		Threshold = XLW_otsuThreshold(image_in, MT9V03X_H, MT9V03X_W);  //OTSU
		//return Threshold ;
	}
	/* 二值化 */
	int m = 0;
	if (mode == 0 || mode == 1 || mode == 4) 
	{
	    for (i = 0; i < MT9V03X_H; i++)
		{
			for (j = i*188,m=0; j < (i+1)*188; j++,m++)
			{
				if (image_in[i][m] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
				{
					//	//
					image_out[j] = 255; //printf("%d，", image_out[i][j]);
				}
				else
				{
					//
					image_out[j] = 1; //printf("%d，", image_out[i][j]);
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
				image_out[j] = image_in[i][m]; //printf("%d，", image_out[i][j]);
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
Func    Name: 定义图像三条线 坐标 初始化
Descriptions:  Midd_line.x[i]  Left_line.x[i]  Right_line.x[i]  分别为图像的中左右 三条线的x（横W）坐标

Input   para: ~
In&Out  Para: ~
Output  para: ~
Return value: ~
***********************************************************************************/
void Line_coordinates_init()//初始化定义三根线的数组  左初始都为0  右初始都为159  中初始都为79 
{
	int16 i;
	for (i = 0; i < 80; i++)
	{
		Left_line.x[i] = 0;
		Midd_line.x[i] = 93;
		Right_line.x[i] = 187;
		Left_line_lost[i] = 1;//初始化为1  表示未找着
		Right_line_lost[i] = 1;//初始化为1 表示未找着
	}
}
/***************************************************************
* 函数名称：void GetHistGram(uint8 image[Row][Col])
* 功能说明：获取图像的灰度信息，统计图像中每个灰度值的个数
* 参数说明：
* 函数输入：Img_row_start, Img_row_end  要获取的图像信息（行）范围
* 函数返回：void
* 修改时间：2018年5月30日
* 备 注：
***************************************************************/
int16 HistGram[256] = { 0 };
void GetHistGram(int16 Img_row_start, int16 Img_row_end)
{
	int16 X, Y;
	for (Y = 0; Y < 256; Y++)
	{
		HistGram[Y] = 0;//初始化灰度直方图
	}
	for (Y = Img_row_start; Y <= Img_row_end; Y++)
	{
		for (X = 0; X < COL; X++)
		{
			HistGram[image[Y][X]]++;//统计每个灰度值的个数
		}
	}
}

/***************************************************************
*
*
* 函数名称：uint8t OSTUThreshold()
* 功能说明：大津法获取图像阈值
* 参数说明：
* 函数输入：void
* 函数返回：uint8t 阈值
* 修改时间：2018年5月30日
* 备 注：
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
	float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
	int16 MinValue, MaxValue;
	uint8 Threshold = 0;
	for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
	for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

	if (MaxValue == MinValue)
	{
		return MaxValue;          // 图像中只有一个颜色（灰度值）    
	}
	if (MinValue + 1 == MaxValue)
	{
		return MinValue;      // 图像中只有二个颜色（灰度值）
	}

	for (Y = MinValue; Y <= MaxValue; Y++)
	{
		Amount += HistGram[Y];        //  计算像素总数
	}

	PixelIntegral = 0;
	for (Y = MinValue; Y <= MaxValue; Y++)
	{
		PixelIntegral += HistGram[Y] * Y;//灰度值总数
	}
	SigmaB = -1;
	for (Y = MinValue; Y < MaxValue; Y++)
	{
		PixelBack = PixelBack + HistGram[Y];    //前景像素点数
		PixelFore = Amount - PixelBack;         //背景像素点数
		OmegaBack = (float)PixelBack / Amount;//前景像素百分比
		OmegaFore = (float)PixelFore / Amount;//背景像素百分比
		PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
		PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
		MicroBack = (float)PixelIntegralBack / PixelBack;//前景灰度百分比
		MicroFore = (float)PixelIntegralFore / PixelFore;//背景灰度百分比
		Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
		if (Sigma > SigmaB)//遍历最大的类间方差g
		{
			SigmaB = Sigma;
			Threshold = Y;
		}
	}
	return Threshold;
}
/**********************************************************************************
Func    Name: 图像静态阈值二值化
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
	//uint8 th1 = OSTU_Threshold();//75;//66  远处（0~6行）图像的阈值
	//GetHistGram(10,63);
	//uint8 th2 = OSTU_Threshold();//75;//66  近处（7~20行）图像的阈值
   // GetHistGram(21,63);
	//uint8 th3 = OSTU_Threshold();//75;//66  近处（21~63行）图像的阈值
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
Func    Name: 边线、中线求取
Descriptions:

Input   para: ~
In&Out  Para: ~
Output  para: ~
Return value: ~
***********************************************************************************/
uint8* Get_line_LMR(uint8 imagein[ROW][COL],uint8 side[ROW*3])
{
	uint8 Get_L_near_slope_flag = 0;//本场图像是否有尝试进行近处边线斜率的计算获取  以丢线前面几行为基准求斜率
	uint8 Get_R_near_slope_flag = 0;//本场图像是否有尝试进行近处边线斜率的计算获取  以丢线前面几行为基准求斜率
	uint8 Vertical_search_flag = 0;//纵向搜索标志位
	uint8 Fill_line_flag = 0;//是否可以进行单边或十字补线标志位  
	uint8 L_line_lost_flag = 0;//右 丢线标志位  辅助记录最先丢线的行
	uint8 R_line_lost_flag = 0;//左 丢线标志位  辅助记录最先丢线的行
	uint8 L_fir_line_flag = 0;//左 丢线标志位  辅助记录最先找到的行
	uint8 R_fir_line_flag = 0;//右 丢线标志位  辅助记录最先找到的行
	uint8 L_R_line_lost_flag = 0;//左右边线同时丢线标志位  辅助记录最先丢线的行
	uint8 Vertical_find_left_flag = 0;//纵向搜索往前推几行后是否有找着左边界 默认0为未找着，0未找着则纵向搜索斜率沿用上一场图像斜率
	uint8 Vertical_find_right_flag = 0;//纵向搜索往前推几行后是否有找着右边界 默认0为未找着，0未找着则纵向搜索斜率沿用上一场图像斜率
	uint8 out_point = 0;//用于判断小车最前端一行是否接近全黑 辅助判断是否出界
	uint8 out_point1 = 0;//用于判断小车最前端一行是否接近全黑 辅助判断是否出界
	uint8 out_point2 = 0;
	uint8 L_near_slope_flag = 0;//是否有根据小车前面几行边界线求出左边界斜率 1为有
	uint8 R_near_slope_flag = 0;//是否有根据小车前面几行边界线求出右边界斜率 1为有
	int16 L_earliest_lost_row = 0, R_earliest_lost_row = 0, L_R_earliest_lost_row = 0;//左、右边最早丢的行数
	//int16 L_earliest_find_row = 63,R_earliest_find_row = 63;
	int16 Left_end = 0;//每场图像扫线时 左边界限制列数
	int16 Right_end = 187;//每场图像扫线时 右边界限制列数
	int16 i = 63, j = (int16)Middle_line;                                                                          //63
	int16 Left_start = Last_mid_line, Right_start = Last_mid_line;//初始化左右扫线起始列 起始为上一场图像中线
	int16 Vertical_row_1 = 63;//纵向搜索最远端黑点所在的行                                                                         //63
	int16 Vertical_longest_length_1 = 0;//纵向搜索长度（最远端）
	int16 Vertical_length = 63;//纵向搜索长度                                                                          //63
	int16 Vertical_longest_length_2 = 63;//纵向搜索长度  用来比较                                                                         //63
	int16 Vertical_column_1 = 93;//纵向搜索记录到到最远端的 行数 列数
	int16 Vertical_column_2 = 93;
	int16 Vertical_column = 93;//最终记录的纵向搜索记录到到最远端的 行数 列数
	int16 Vertical_search_row_left = 0, Vertical_search_row_right = 0;//纵向搜索时 从最远端往前面加n行 即为该变量，动态 用于搜索最远端前面(靠近小车)n行左右两边的黑点 用于丢线行前一行所在列做斜率用
	int16 Aver_mid_line;//中线平均值  求取偏差用
	float Miline_weight_sum = 0;//加权算法  中线加权总和
	float Weight_sum = 0;//权重总和 中线加权总和除以权重总和 即可得到该场图像加权后的赛道中线(点)所在列  与图像中线(79)相减即可得到用于舵机PD用的偏差


	Left_slope = 0, Right_slope = 0;//左右边线 延伸  斜率  这里可考虑不清零
	Cross_road_flag = 0;//十字丢线标志位  不一定为真正的十字
	L_lost_cnt = 0;//左边界丢线次数
	R_lost_cnt = 0;//右边界丢线次数
	L_R_lost_cnt = 0;//左右边界同时丢线次数
	Cross_road_cnt = 0;//十字丢线次数
	Row_start = 63;//扫线起始行;                                                                         //63

/********************以下是环岛识别所用到的相关变量先放这里*********************/
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
	/********************以上是环岛识别所用到的相关变量先放这里*********************/

	/**********************以下是会车区识别所用到的相关变量************************/
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
		 *  @brief        边界限制，防止越界
		 *  @param        ~
		 *  Sample usage  ~
		 */
		Found_left_flag = 0;                                         //左边界丢线标记
		Found_right_flag = 0;                                        //右边界丢线标记
		Left_line.x[row] = Left_line.x[row + 1] - 4;                     //扫线边界限制，根据本行的数值，预判下一行扫线位置，减小处理行数
		Right_line.x[row] = Right_line.x[row + 1] + 4;
		if (Left_line.x[row] < 0)      Left_line.x[row] = 0;            //限制坐标平面，放置越界
		if (Right_line.x[row] > 187)   Right_line.x[row] = 187;
		Left_end = (int16)Left_line.x[row];                     //扫描列结束限制
		Right_end = (int16)Right_line.x[row];
		if (Left_end < 0)      Left_end = 0;
		if (Right_end > 187)   Right_end = 187;
		/*
		 *  @brief        寻找扫描边界黑线，找到则记录黑点坐标并做标志
		 *  @param        start       扫描起始行
		 *  @param        end         扫描终止行
		 *  Sample usage
		 */

		for (j = Left_start; j > Left_end; j--)
		{
			if (binary_img[row][j] == 0 && binary_img[row][j - 1] == 0)                     //黑黑  易受噪点或者赛道上黑点干扰  后期考虑加滤波 如果控制周期允许的话  最原始滤波：两白点中间出现黑点，则该黑点为白
			{
				Left_line.x[row] = j;                                //记录黑点所在行的列数
				Found_left_flag = 1;                                 //黑点捕捉成功标记
				Left_line_lost[row] = 0;                            //记录所在点颜色，为黑色
				break;
			}
		}
		for (j = Right_start; j < Right_end; j++)
		{
			if (binary_img[row][j] == 0 && binary_img[row][j + 1] == 0)                     //黑黑
			{
				Right_line.x[row] = j;
				Found_right_flag = 1;
				Right_line_lost[row] = 0;
				break;
			}
		}
		/*
		 *  @brief        图像远处是否有斑马线（有的话停止往后巡线，防止干扰）
		 *  @param
		 *  @param
		 *  Sample usage
		 */


		 /*
		  *  @brief        防畸变干扰
		  *  @param
		  *  @param
		  *  Sample usage
		  */
		if (Found_left_flag && Left_line.x[row] - Left_line.x[row + 1] < 0)//正常左线趋势  偏右，此处左线偏左  如果偏左则为左转  或者靠近十字导致的图像畸变  此时判断是右转还是图像畸变所找到的边界
		{
			if (!Found_right_flag || Right_line.x[row] - Right_line.x[row + 1] > 0)//右边未找到或者右边线减去上一行的右边线大于0，则认为是图像畸变
			{
				Found_left_flag = 0;                                 //标记本行未找到真正的边界，所找到的边界为图像畸变导致横向黑线所变成的纵向边界
				Left_line_lost[row] = 1;                            //标记本行丢线
			}
		}
		if (Found_right_flag && Right_line.x[row] - Right_line.x[row + 1] > 0)//正常右线趋势  偏左，此处右线偏右  如果偏右则为左转  或者靠近十字导致的图像畸变  此时判断是左转还是图像畸变所找到的边界
		{
			if (!Found_left_flag || Left_line.x[row] - Left_line.x[row + 1] < 0)//左边未找到或者左边线减去上一行的右边线小于0，则认为是图像畸变
			{
				Found_right_flag = 0;                                 //标记本行未找到真正的边界，所找到的边界为图像畸变导致横向黑线所变成的纵向边界
				Left_line_lost[row] = 1;                            //标记本行丢线
			}
		}
		/*
		*  @brief        防噪点干扰  防左右线由于噪点干扰产生跳变
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
		 *  @brief          记录左右边界最先丢线所在的行
		 *  @param         ~null
		 *  @param         ~null
		 *  Sample usage   ~null
		 */
		if (!Found_left_flag && Found_right_flag && !L_line_lost_flag)  //记录第一次左边界单边丢线所在的行
		{
			L_line_lost_flag = 1;
			L_earliest_lost_row = row;
		}
		if (!Found_right_flag && Found_left_flag && !R_line_lost_flag)//记录第一次右界单边丢线所在的行
		{
			R_line_lost_flag = 1;
			R_earliest_lost_row = row;
		}
		if (!Found_left_flag && !Found_right_flag && !L_R_line_lost_flag)//记录第一次左右边界同时丢线所在的行
		{
			L_R_line_lost_flag = 1;
			L_R_earliest_lost_row = row;
		}
		if (!L_fir_line_flag && !Left_line_lost[row])//记录第一次左边界没丢线的所在行
		{
			L_fir_line_flag = 1;
			//L_earliest_find_row = row;
		}
		if (!R_fir_line_flag && !Right_line_lost[row])//记录第一次右边界没丢线的所在行
		{
			R_fir_line_flag = 1;
			//R_earliest_find_row = row;
		}
		/*
		 *  @brief        记录各边丢线次数  及最后丢线的 行
		 *  @param         ~null
		 *  @param         ~null
		 *  Sample usage   ~null
		 */
		if (row > 5)
		{
			if (!Found_left_flag || !Found_right_flag)//丢线的三种情况 做此判断非多此一举，如果两边都未丢，则无需再进行判断
			{
				if (!Found_left_flag && !Found_right_flag)//十字丢线 两边全丢
				{
					L_R_lost_cnt++;//十字丢线次数(也可以说是双边丢线次数，下面所写的Cross_road  即为近处真正的十字丢线)
					L_R_lost_row = row;//记录一场图像最后一次两边同时丢线的行 
					if (row > 10)
						Cross_road_cnt++;
				}
				if (!Found_left_flag && Found_right_flag)
				{
					L_lost_cnt++;//单左边丢线次数
					L_last_lost_row = row;//记录一场图像左边最后一次丢线的所在行
				}
				if (Found_left_flag && !Found_right_flag)
				{
					R_lost_cnt++;//单右边丢线次数
					R_last_lost_row = row;//记录一场图像右边最后一次丢线的所在行
				}
			}
		}
		/*
	   *  @brief 利用丢线行前面的有效行求斜率   这样就无需进行纵向求斜率   近处斜率比远处斜率准确稳定，近处斜率无法求再采用远端斜率
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
		 *  @brief          十字 环岛 补线
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
					Left_line.x[row] = Left_line.x[row + 1] + Left_slope;//若左线斜率为k>0(k<0) 则每单位右(左)移|k|，以此类推
					//gpio_set(PTC10,0);
				}
				else if ((Island_flag == 0 || Island_flag == 2) && Right_slope < 0 && (((Right_line.x[row + 2] - Right_line.x[row + 1] > 0 || Right_line.x[row + 3] - Right_line.x[row + 2] > 0 || R_near_slope_flag)) && Found_left_flag && !Found_right_flag))
				{
					Right_line.x[row] = Right_line.x[row + 1] + Right_slope;//若右线斜率为k>0(k<0) 则每单位右(左)移|k|，以此类推

				}
			}
		}

		if (!Island_flag)
		{
			/*if(meet_flag)
			{
				Midd_line.x[row] = (Right_line.x[row] - Single_R_line[row]) + 93 + 25;//靠边行驶， 不采用此种方式，直接将最终的中值平移即可
			}
			else*/ Midd_line.x[row] = (Left_line.x[row] + Right_line.x[row]) / 2;//每行中线列坐标
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
	   *  @brief        若进行纵向搜索，则计算斜率，斜率相当于  每单位纵坐标 左右 移动几单位 上一次的左右黑线
	   *  @param         ~null
	   *  @param         ~null
	   *  Sample usage   ~null
	   */

	   /*
	   *  @brief        纵向搜索，记录最远端边缘黑点坐标，
	   *  @param         ~null
	   *  @param         ~null
	   *  Sample usage   ~null
	   */
		if ((!Found_left_flag || !Found_right_flag || Island_flag == 3 || Island_flag == 4 || meet_flag) && !Vertical_search_flag)  //单边丢线 或者上一场图像确定为环岛开启纵向搜索
		{
			Vertical_search_flag = 1;                                     //纵向搜索标志位
			if (!Found_left_flag || !Found_right_flag || Island_flag == 3 || Island_flag == 4 || meet_flag)
			{
				for (j = 1; j < 187; j++)//49~110  搜索列范围过大可能会受到临近赛道干扰
				{
					for (i = row; i > 0; i--)
					{


						if (((binary_img[i][j] == 0 && binary_img[i - 1][j] == 0) || (binary_img[i + 1][j] != 0 && binary_img[i][j] == 0)) || (i == 1))
						{
							Vertical_length = row - i;//本行row到纵向最远处的长度
							if (Vertical_length > Vertical_longest_length_1)
							{
								Vertical_longest_length_1 = Vertical_length;//记录本行row到纵向最远处黑点的最长 长度
								Vertical_row_1 = i;//记录本行row纵向 最远处黑点的W坐标（行数）
								Vertical_column_1 = j;//记录本行row纵向 最远处黑点的H坐标（列数）
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
		 *  @brief        若进行纵向搜索，则计算斜率，斜率相当于  每单位纵坐标 左右 移动几单位 上一次的左右黑线
		 *  @param         ~null  此处用纵向搜索最远端所在行的前面n行往左右两边搜索黑点  并与丢线行做斜率 后面可再加一种求斜率方法
		 *  @param         ~null  当某一边界丢线时，其前面的有效边界数大于某个值时，即可利用前面搜索到的有效边界求斜率，而不用进行纵向搜索
		 *  Sample usage   ~null
		 */
		if (Vertical_search_flag && !Fill_line_flag && (!L_near_slope_flag || !R_near_slope_flag))                 //纵向搜索结束
		{
			Fill_line_flag = 1;
			if (!L_near_slope_flag)
			{
				Vertical_search_row_left = Vertical_row_1 + 3;  //最远端图像赛道所在行 前面五行
				while (!Vertical_find_left_flag)
				{
					for (j = Vertical_column; j > 30; j--)//此处限制范围考虑放宽 减小
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
							break;//当最远端前面十行均未搜索到边界点时，则退出不再搜索 斜率沿用上一场图像的边界斜率
						}
					}
				}
			}
			if (!R_near_slope_flag)
			{
				Vertical_search_row_right = Vertical_row_1 + 3;
				while (!Vertical_find_right_flag)
				{
					for (j = Vertical_column; j < 139; j++)//此处限制范围考虑放宽 增大
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
							break;//当最远端前面十行均未搜索到边界点时，则退出不再搜索 斜率沿用上一场图像的边界斜率
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
		 *  @brief        补线限制   后续需要修改
		 *  @param         Near_flag  近处处理 标志
		 *  @param         ~null
		 *  Sample usage   ~null
		 */
		if (!meet_flag && row < Row_start)
		{
			if ((fabs)(Midd_line.x[row] - Midd_line.x[row + 1]) > 20)//相邻行中线值相差超过某个值，则可能为反光导致远处赛道中线值跳变，可考虑直接break退出后面的搜索
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
			row = row + 1;//该判断语句 需要重新定义  连续两个中点为黑色则退出扫线
			break;
		}
		else if ((Island_flag == 3 && Midd_line.x[row] - Midd_line.x[row + 1] > 0) || (Island_flag == 4 && Midd_line.x[row] - Midd_line.x[row + 1] < 0))
		{
			row = row + 1;   //绕环时 ： 若左环中线往右则退出扫线  若右环中线往左则退出扫线
			break;
		}

		Left_start = (int16)Midd_line.x[row];//下一行扫左边线时，起始列为本行中线所在列
		Right_start = (int16)Midd_line.x[row];//下一行扫右边线时，起始列为本行中线所在列
	}

	if (Cross_road_cnt > 8)
	{
		Cross_road_flag = 1;//近处超过八行左右边界同时丢线 时  确认为十字
	}

	Row_end = row + 1;//记录本场图像所处理到的最远端的所在行
	Column_end = (int16)Midd_line.x[Row_end];//记录本场图像所处理到的最远端的所在行 的中线所在列坐标

	if (Row_end > Row_start - 2)     Near_flag = 1;//本场图像所处理到的最远端的所在行 只比扫线起始行小1，即处理的行数只有一行，则认为小车靠近赛道边界 （即将丢线）
	else                Near_flag = 0;
	Row_begging = Row_start;
	Last_row_end = Row_end;
	/*
	 *  @brief        环岛判断
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
		if (L1_R0_cnt < 1 && L0_R1 && R_lost_cnt < 1 && L_R_lost_cnt < 1 && L0_R1_row - L1_R1_row>5) //区分十字  数字可修改    改变条件严格程度
		{
			if (Left_line.x[L1_R1_row - 1] - Left_line.x[L1_R1_row] > 3 && ((!Left_line_lost[63] && L0_R1_cnt > 9 && Left_line.x[L1_R1_row + 1] - Left_line.x[L1_R1_row + 2] > 0) || (Left_line_lost[63] && L0_R1_cnt > 12 && Left_line.x[L1_R1_row] - Left_line.x[L1_R1_row + 1] > 15 && Left_line.x[L1_R1_row - 2] - Left_line.x[L1_R1_row - 1] > 3)))                                                                         //63
			{
				if (fabs((Right_line.x[L1_R1_row + 5] - Right_line.x[L1_R1_row]) - (Right_line.x[L0_R1_row] - Right_line.x[L0_R1_row - 5])) < 5)
				{
					Island_flag = 1;
				}
			}
		}
		else if (L0_R1_cnt < 1 && L1_R0 && L_lost_cnt < 1 && L_R_lost_cnt < 1 && L1_R0_row - L1_R1_row>5) //区分十字  数字可修改    改变条件严格程度
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
				Big_island_error3++;//相邻两行的列坐标相差大于三的次数  用于区分大小环
			}
			else if (Left_line.x[i] - Left_line.x[i + 1] > 2)
			{
				Big_island_error2++;//相邻两行的列坐标相差大于二的次数  用于区分大小环
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
		if (Big_island_error3 > 12 || (Big_island_error3 + Big_island_error2 > 15))//大环岛半径较小时  减小12 15
		{
			Big_island_flag = 1;
		}
		if (Island_inflexion_row > (Big_island_flag ? 10 : 15))//大环岛半径较小时  减小15
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
				Big_island_error3++;//相邻两行的列坐标相差大于三的次数  用于区分大小环
			}
			else if (Right_line.x[i + 1] - Right_line.x[i] > 2)
			{
				Big_island_error2++;//相邻两行的列坐标相差大于二的次数  用于区分大小环
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
		if (Big_island_error3 > 12 || (Big_island_error3 + Big_island_error2 > 15))//大环岛半径较小时  减小12 15
		{
			Big_island_flag = 1;
		}
		if (Island_inflexion_row > (Big_island_flag ? 10 : 15))//大环岛半径较小时  减小15
		{
			Island_flag = 4;
		}
	}
	else if (Island_flag == 3 && Island_time > Island_change_time)
	{
		if (Right_line_lost[63] && Right_line_lost[62] && Right_line_lost[61])//出右岛时前面五行左边线全丢
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
		if (Left_line_lost[63] && Left_line_lost[62] && Left_line_lost[61])//出右岛时前面五行左边线全丢
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
				Left_line_swerve_R++;//左线往右趋势
			else if (Left_line.x[i] - Left_line.x[i - 1] > 0)
				Left_line_swerve_L++;//左线往左趋势
			if (Right_line.x[i] - Right_line.x[i - 1] > 0)
				Right_line_swerve_L++;//右线趋势往左
			else if (Right_line.x[i] - Right_line.x[i - 1] < 0)
				Right_line_swerve_R++;//右线趋势往右
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
				Left_line_swerve_R++;//左线往右趋势
			else if (Left_line.x[i] - Left_line.x[i - 1] > 0)
				Left_line_swerve_L++;//左线往左趋势
			if (Right_line.x[i] - Right_line.x[i - 1] > 0)
				Right_line_swerve_L++;//右线趋势往左
			else if (Right_line.x[i] - Right_line.x[i - 1] < 0)
				Right_line_swerve_R++;//右线趋势往右
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
	 *  @brief        会车区识别
	 *  @param         Near_flag
	 *  @param         ~null
	 *  Sample usage   ~null
	 */
	if (((meet_time == 0 && Start_time == 2000) || (meet_time == 1500)) && !meet_flag && !Island_flag && !Cross_road_flag && L_lost_cnt < 1 && R_lost_cnt < 1)//直道会车区
	{
		for (i = 63; i > Row_end; i--)
		{
			if (!L_dent_flag && !Left_line_lost[i] && !Left_line_lost[i - 1] && Left_line.x[i] - Left_line.x[i - 1] > 0)
			{
				L_dent_flag = 1;//发现有凹陷
				L_dent_row = i;//记录凹陷的所在行的前一行（行数大为前，小为后）
			}
			else if (!L_dent_flag && !Left_line_lost[i] && !Left_line_lost[i - 1] && Left_line.x[i - 1] - Left_line.x[i] > 3)
			{
				L_cripling_flag = 1;//发现有凸出的
				L_cripling_row = i;//记录凸出的所在行的前一行
				L_cripling++;//凸出的次数
			}
			else if (L_dent_flag && ((!Left_line_lost[i] && !Left_line_lost[i - 1] && Left_line.x[i - 1] - Left_line.x[i] > 3) || L_cripling_flag))
			{
				if (L_cripling_flag && L_cripling_row - L_dent_row < 10 && L_cripling_row - L_dent_row>2)
				{
					L_dotted_line++;//（出现凹陷并且往后搜索到相邻行列坐标差值大于3 或者有凹陷又有凸出）的次数  即搜索到一段标准的虚线
					L_cripling_flag = 0;//只清凸出的标志 不清凹陷的标志
				}
				else if (L_dent_row - i < 10 && L_dent_row - i>0)
				{
					L_dotted_line++;//（出现凹陷并且往后搜索到相邻行列坐标差值大于3 或者有凹陷又有凸出）的次数  即搜索到一段标准的虚线
					L_cripling_flag = 0;//清凸出的标志 清凹陷的标志
					L_dent_flag = 0;//清凹陷的标志
				}
			}
			if (!Left_line_lost[i] && !Left_line_lost[i - 1] && Left_line.x[i - 1] - Left_line.x[i] > 2)
			{
				L_cripling2++;
			}


			if (!R_dent_flag && !Right_line_lost[i] && !Right_line_lost[i - 1] && Right_line.x[i - 1] - Right_line.x[i] > 0)
			{
				R_dent_flag = 1;//发现有凹陷
				R_dent_row = i;//记录凹陷的所在行的前一行（行数大为前，小为后）
			}
			else if (!R_dent_flag && !Right_line_lost[i] && !Right_line_lost[i - 1] && Right_line.x[i - 1] - Right_line.x[i] > 3)
			{
				R_cripling_flag = 1;//发现有凸出的
				R_cripling_row = i;//记录凸出的所在行的前一行
				R_cripling++;//凸出的次数
			}
			else if (R_dent_flag && (!Right_line_lost[i] && !Right_line_lost[i - 1] && Right_line.x[i] - Right_line.x[i - 1] > 3 || R_cripling_flag))
			{
				if (R_cripling_flag && R_cripling_row - R_dent_row < 10 && R_cripling_row - R_dent_row>2)
				{
					R_dotted_line++;//（出现凹陷并且往后搜索到相邻行列坐标差值大于3 或者有凹陷又有凸出）的次数  即可能搜索到一段标准的虚线
					R_cripling_flag = 0;//只清凸出的标志 不清凹陷的标志
				}
				else if (R_dent_row - i < 10 && R_dent_row - i>0)
				{
					R_dotted_line++;//（出现凹陷并且往后搜索到相邻行列坐标差值大于3 或者有凹陷又有凸出）的次数  即可能搜索到一段标准的虚线
					R_cripling_flag = 0;//清凸出的标志 清凹陷的标志
					R_dent_flag = 0;//清凹陷的标志
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
	 *  @brief        权重加权累加  先判断中点是否为白 若不为白就舍弃
	 *  @param         Near_flag
	 *  @param         ~null
	 *  Sample usage   ~null
	 */
	for (i = Row_end; i <= Row_start; i++)  //扫线起始行;
	{
		if (binary_img[i][(int16)Midd_line.x[i]] != 0)
		{
			Miline_weight_sum += Midd_line.x[i] * Add_weight[i - Row_end];//计算中线综总和
			Weight_sum += Add_weight[i - Row_end];//权值总数
		}
	}

	if (meet_flag == 1)
		Aver_mid_line = (int16)(Miline_weight_sum / Weight_sum) + 20;//中线最佳期望值
	else if (meet_flag == 2 && !Stop_ready2)
		Aver_mid_line = (int16)(Miline_weight_sum / Weight_sum) + 24;//中线最佳期望值
	else
		Aver_mid_line = (int16)(Miline_weight_sum / Weight_sum);

	if (meet_flag && (Left_car_flag_mid || Left_car_flag_end))
		Last_mid_line = Aver_mid_line;


	Steer.error = Aver_mid_line - 93;//偏差
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
Func    Name: 定义图像三条线 坐标 初始化
Descriptions:  Mid.x[i]  Left.x[i]  Right.x[i]  分别为图像的中左右 三条线的x（横W）坐标

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
 *  @brief        转弯趋势   利用相邻中线差值计算
 *  @param         Swerve_trend 转弯趋势
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
	Swerve_trend = abs(L_turn_row - R_turn_row);//转弯趋势
/*
 *  @brief        坡道检测
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
				Ram_flag = 1;//上坡
			}
			/*else if(L_R_lost_cnt<1&&L_lost_cnt<8&&R_lost_cnt<8&&Right_line.x[15]-Left_line.x[15]>95&&Swerve_trend<5)
			{
				Ram_flag = 2;//下坡  此处因为实验室原因  若坡道前后非急弯 则可用第一个直接判断为上坡或者下坡
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
 *  @brief        直道检测
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
//  * @brief    画边线
//  * @param
//  * @return
//  * @date     2020/6/28 星期日
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
//  * @brief    判断是否是直道
//  * @param    image ： 二值图像信息
//  * @return   0：不是直道， 1：直道
//  * @note     思路：两边边线都单调
//  * @date     2020/6/23 星期二
//  */
//uint8_t RoadIsStraight(uint8_t imageSide[MT9V03X_H][2])
//{
//	uint8_t i = 0;
//	uint8_t leftState = 0, rightState = 0;
//
//	/* 左边线是否单调 */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		if (imageSide[i][0] + 5 < imageSide[i + 1][0])
//		{
//			leftState = 1;
//			break;
//		}
//	}
//
//	/* 右边线是否单调 */
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
//  * @brief    判断是否是斑马线
//  * @param    image ： 二值图像信息
//  * @return   0：不是， 1：是
//  * @note     思路：
//  * @date     2020/6/23 星期二
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
//  * @brief    判断是否是十字
//  * @param    imageSide ： 图像边线信息
//  * @param    flag      ： 十字状态信息
//  * @return   0：不是， 1：是
//  * @note     思路：两条边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 则证明有十字
//  * @date     2020/6/23 星期二
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
//	/* 检测右侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
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
//	/* 检测左侧边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 */
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
//  * @brief    判断是否是环岛
//  * @param    image ： 边线信息
//  * @param    flag  ： 环岛状态信息
//  * @return   0：不是， 1：左环岛  2：右环岛
//  * @note     思路：一条边线严格单调，另一条边线距离车头近的行丢线 -- 然后一部分行不丢线  --  接着又丢线 则证明有环岛
//  * @date     2020/6/23 星期二
//  */
//uint8_t RoadIsRoundabout(uint8_t image[MT9V03X_H][2], uint8_t* flag)
//{
//	int i = 0;
//	uint8_t leftState = 0, rightState = 0;
//	int start[5] = { 0, 0, 0, 0, 0 }, end[5] = { 0, 0, 0, 0, 0 };
//	uint8_t count = 0;
//	uint8_t index = 0;
//
//	/* 左边线是否单调 */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		if (image[i][0] + 5 < image[i + 1][0])
//		{
//			leftState = 1;
//			break;
//		}
//	}
//
//	/* 右边线是否单调 */
//	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//	{
//		if (image[i][1] - 5 > image[i + 1][1])
//		{
//			rightState = 1;
//			break;
//		}
//	}
//
//	/* 左边单调， 检测右侧是否是环岛 */
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
//	/* 右边单调， 检测左侧是否是环岛 */
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
//  * @brief    获取环岛边线
//  * @param    imageInput ： 二值图像信息
//  * @param    imageOut   ： 边线数组
//  * @param    status     ： 1：左环岛  2：右环岛
//  * @return
//  * @note     思路：环岛一边边线严格单调，根据一边边线，获取另一边线
//  * @date     2020/6/23 星期二
//  */
//void RoundaboutGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t status)
//{
//	int i = 0, j = 0;
//
//	switch (status)
//	{
//
//		/* 左环岛 */
//	case 1:
//	{
//		/* 重新确定左边界 */
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
//		/* 重新确定右边界 */
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
//  * @brief    判断环岛边线是否存在弧形
//  * @param    imageInput ： 二值图像信息
//  * @param    imageOut   ： 边线数组
//  * @param    status     ： 1：左环岛  2：右环岛
//  * @return
//  * @note
//  * @date     2020/6/23 星期二
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
//				/* 有弧线 */
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
//				/* 有弧线 */
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
//  * @brief    补线处理
//  * @param    imageSide  : 边线
//  * @param    status     : 1：左边线补线   2：右边线补线
//  * @param    startX     : 起始点 列数
//  * @param    startY     : 起始点 行数
//  * @param    endX       : 结束点 列数
//  * @param    endY       : 结束点 行数
//  * @return
//  * @note     endY 一定要大于 startY
//  * @date     2020/6/24 星期三
//  */
//void ImageAddingLine(uint8_t imageSide[MT9V03X_H][2], uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
//{
//	int i = 0;
//
//	/* 直线 x = ky + b*/
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
//  * @brief    寻找跳变点
//  * @param    imageSide   ： 边线数组
//  * @param    status      ：1：左边界   2：右边界
//  * @return
//  * @note
//  * @date     2020/6/24 星期三
//  */
//uint8_t ImageGetHop(uint8_t imageSide[MT9V03X_H][2], uint8_t state, uint8_t* x, uint8_t* y)
//{
//	int i = 0;
//	uint8_t px = 0, py = 0;
//	uint8_t count = 0;
//	switch (state)
//	{
//	case 1:
//		/* 寻找跳变点 */
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
//		/* 寻找跳变点 */
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
//  * @brief    环岛补线处理
//  * @param    imageInput ： 二值图像信息
//  * @param    imageSide  ： 边线数组
//  * @param    status     ：环岛标志位   1：发现左环岛   2：发现右环岛   3：左环岛即将入环  4：右环岛即将入环  5：左环岛即将出环  6：右环岛即将出环
//  * @return
//  * @note     这里只写了左环岛，右环岛大家可以参考左环岛自己完善
//  * @date     2020/6/24 星期三
//  */
//uint8_t* RoundaboutProcess(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageSide[MT9V03X_H][2], uint8_t* state)
//{
//	int i = 0;
//	uint8_t pointX = 0, pointY = 0;
//	uint8_t err = 0;
//	static uint8_t cnt = 0;
//	switch (*state)
//	{
//		/* 发现左环岛 环岛出口处补线 */
//	case 1:
//
//		/* 重新确定左边界 */
//		RoundaboutGetSide(imageInput, imageSide, 1);
//
//		/* 检查弧线 */
//		err = RoundaboutGetArc(imageSide, 1, &pointY);
//
//		/* 有弧线 进行补线 连接弧线最右点 和 图像左下角 */
//		if (err)
//		{
//			pointX = imageSide[pointY][0];
//
//			/* 准备入环岛 */
//			if ((pointY + 10) > ROAD_MAIN_ROW)
//			{
//				*state = 3;
//			}
//		}
//		else
//		{
//			pointY = ROAD_START_ROW - 1;
//
//			/* 没有弧线 进行补线 连接边线最右点 和 图像左下角 */
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
//		/* 补线 */
//		ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
//
//		break;
//
//		/* 发现左环岛 环岛出口处补线 */
//	case 2:
//
//		*state = 0;
//		break;
//
//
//		/* 准备进入环岛， 直道部分补线 */
//	case 3:
//		pointY = ROAD_START_ROW - 1;
//
//		/* 重新确定左边界 */
//		RoundaboutGetSide(imageInput, imageSide, 1);
//
//		/* 检查弧线 */
//		err = RoundaboutGetArc(imageSide, 1, &pointY);
//
//		/* 有弧线 进行补线 连接弧线最右点 和 图像左下角 */
//		if (err)
//		{
//			pointX = imageSide[pointY][0];
//
//			if ((pointY + 10) > ROAD_MAIN_ROW)
//			{
//				/* 环岛出口补线 */
//				ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
//			}
//		}
//
//		/* 环岛入口补线 */
//		pointX = MT9V03X_W / 3;
//		pointY = ROAD_START_ROW - 1;
//
//		/* 寻找跳变点 */
//		ImageGetHop(imageSide, 1, &pointX, &pointY);
//
//		if (pointY >= ROAD_MAIN_ROW && pointY != ROAD_START_ROW - 1)
//		{
//			imageSide[ROAD_MAIN_ROW][0] = 0;
//			*state = 5;
//		}
//
//		/* 补线 */
//		ImageAddingLine(imageSide, 2, pointX + 30, pointY, (MT9V03X_W - 1), ROAD_START_ROW);
//
//		break;
//
//	case 4:
//
//		break;
//
//		/* 出环岛， 直道处补线 */
//	case 5:
//
//
//		/* 检查弧线 */
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
//		/* 出环岛， 直道处补线 */
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
//  * @brief    获取十字边线
//  * @param    imageInput ： 二值图像信息
//  * @param    imageOut   ： 边线数组
//  * @return
//  * @note     思路：从中间向两边搜线
//  * @date     2020/6/23 星期二
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
//  * @brief    十字补线处理
//  * @param    imageInput ： 二值图像信息
//  * @param    imageSide  ： 边线数组
//  * @param    status     ：十字标志位   1：发现十字    2：进入十字   3：出十字
//  * @return
//  * @note
//  * @date     2020/6/24 星期三
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
//		/* 重新获取边线 */
//		CrossGetSide(imageInput, imageSide);
//
//		/* 寻找跳变点 */
//		if (ImageGetHop(imageSide, 1, &pointX, &pointY))
//		{
//			/* 补线 */
//			ImageAddingLine(imageSide, 1, pointX, pointY, 0, ROAD_START_ROW);
//		}
//
//		leftIndex = pointY;
//		pointX = 0;
//		pointY = 0;
//
//		/* 寻找跳变点 */
//		if (ImageGetHop(imageSide, 2, &pointX, &pointY))
//		{
//			/* 补线 */
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
//		/* 检查弧线 */
//		if (RoundaboutGetArc(imageSide, 1, &leftIndex))
//		{
//			/* 重新确定左边界 */
//			RoundaboutGetSide(imageInput, imageSide, 1);
//
//			if (ImageGetHop(imageSide, 1, &pointX, &pointY))
//			{
//				/* 补线 */
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
//		/* 重新确定左边界 */
//		RoundaboutGetSide(imageInput, imageSide, 1);
//
//
//		if (ImageGetHop(imageSide, 1, &pointX, &pointY))
//		{
//			/* 检查弧线 */
//			if (RoundaboutGetArc(imageSide, 1, &leftIndex))
//			{
//				/* 补线 */
//				ImageAddingLine(imageSide, 1, pointX, pointY, imageSide[leftIndex][0], leftIndex);
//			}
//			else
//			{
//				/* 补线 */
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
//  * @brief    停车线处理
//  * @param    imageSide  ： 边线数组
//  * @param    state      ： 停车状态  1：车库在左侧   2：车库在右侧
//  * @param    speed      ： 速度
//  * @return
//  * @note
//  * @date     2020/6/24 星期三
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
//  * @brief    根据主跑行，求取舵机偏差
//  * @param
//  * @return
//  * @note
//  * @date     2020/6/24 星期三
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
//  * @brief    判断是否丢线
//  * @param    imageInput ： 二值图像信息
//  * @param    imageOut   ： 边线数组
//  * @param    lineIndex  ： 行
//  * @return   0：没有丢线   1:左边丢线  2：右边丢线  3： 左右都丢线   4：错误
//  * @note
//  * @date     2020/6/24 星期三
//  */
//uint8_t RoadIsNoSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2], uint8_t lineIndex)
//{
//	uint8_t state = 0;
//	int i = 0;
//	static uint8_t last = 78;
//
//	imageOut[lineIndex][0] = 0;
//	imageOut[lineIndex][1] = 187;
//	/* 用距离小车比较近的行， 判断是否丢线 */
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
//		/* 左边界丢线 */
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
//		/* 左右边界丢线 */
//		if (state == 1)
//		{
//			state = 3;
//		}
//
//		/* 右边界丢线 */
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
//  * @brief    丢线处理
//  * @param    imageInput ： 二值图像信息
//  * @param    imageOut   ： 边线数组
//  * @param    mode       ： 那边丢线？   1：左边丢线  2：右边丢线
//  * @param    lineIndex  ： 丢线行数
//  * @return
//  * @note
//  * @date     2020/6/24 星期三
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
//  * @brief    获取边线
//  * @param    imageInput ： 二值图像信息
//  * @param    imageOut   ： 边线数组
//  * @return   是否丢线
//  * @note     思路：从距离车头较近的行开始从中间向两边搜线
//  * @date     2020/6/23 星期二
//  */
//  uint8_t* ImageGetSide(uint8_t imageInput[MT9V03X_H][MT9V03X_W], uint8_t imageOut[MT9V03X_H][2])
//  {
//  	int i = 0, j = 0;
//  	RoadIsNoSide(imageInput, imageOut, ROAD_START_ROW);
//  
//  	/* 离车头近的40行，寻找边线 */
//  	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
//  	{
//  		imageOut[i][0] = 0;   //0
//  		imageOut[i][1] = 187;
//  
//  		/* 根据边界连续特性 寻找边界 */
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
//  		/* 如果左边界 即将超出中线， 则检查是否右丢线 */
//  		if (imageOut[i][0] > (MT9V03X_W / 2 - 10) && imageOut[i][1] > (MT9V03X_W - 5))
//  		{
//  			/* 右丢线处理 */
//  			RoadNoSideProcess(imageInput, imageOut, 2, i);
//  
//  			if (i > 70)
//  			{
//  				imageOut[i][0] += 50;
//  			}
//  			//return 1;
//  		}
//  
//  		/* 如果右边界 即将超出中线， 则检查是否左丢线 */
//  		if (imageOut[i][1] < (MT9V03X_W / 2 + 10) && imageOut[i][0] < (5))
//  		{
//  			/* 左丢线处理 */
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
////	/* 离车头近的40行，寻找边线 */
////	for (i = ROAD_START_ROW - 1; i > ROAD_END_ROW; i--)
////	{
////		imageOut[i][0] = 0;
////		imageOut[i][1] = 187;
////
////		/* 根据边界连续特性 寻找边界 */
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
////		/* 如果左边界 即将超出中线， 则检查是否右丢线 */
////		if (imageOut[i][0] > (MT9V03X_W / 2 - 10) && imageOut[i][1] > (MT9V03X_W - 5))
////		{
////			/* 右丢线处理 */
////			RoadNoSideProcess(imageInput, imageOut, 2, i);
////
////			if (i > 70)
////			{
////				imageOut[i][0] += 50;
////			}
////			return 1;
////		}
////
////		/* 如果右边界 即将超出中线， 则检查是否左丢线 */
////		if (imageOut[i][1] < (MT9V03X_W / 2 + 10) && imageOut[i][0] < (5))
////		{
////			/* 左丢线处理 */
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
//  * @brief    除单一的噪点
//  * @param
//  * @return
//  * @note     思路： 检查边沿邻域内的9个点，如果大于设置值，则保留该点
//  * @date     2020/6/24 星期三
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
//			/* 邻域内5个点是边沿 则保留该点 可以调节这里优化滤波效果 */
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
//  * @brief    边线数组
//  * @note
//  */
//uint8_t ImageSide[MT9V03X_H][2];


/*************************图像处理（灵魂）********************/
//void xunxian(uint8_t imagein[MT9V03X_H][MT9V03X_W])
//{
//	uint8_t AX, AY; //A点
//	uint8_t BX, BY; //B点
//	uint8_t CX, CY; //C点
//	uint8_t DX, DY; //D点
//
//	AY = 59; //获取AB点
//	BY = 59;
//	//先找最底下一行中心线
//	for (int i = 39; i >= 1; i--) //从中间向左找上升沿
//	{
//		if (imagein[AY][i - 1] - imagein[AY][i] == 1) //找到上升沿
//		{
//			AX = i; //A横坐标
//		}
//	}
//	for (int i = 39; i < 79; i++) //从中间向右找上升沿
//	{
//		if (imagein[BY][i + 1] - imagein[BY][i] == 1) //找到上升沿
//		{
//			BX = i; //B横坐标
//		}
//	}
//
//	CY = AY - 1; //迭代C点
//	CX = AX - 1;
//	for (int i = CY; i > 0; i--) //由近及远
//	{
//		for (int j = CX; j < 80; i++) //由左向右
//		{
//			if (imagein[i][j] == 0) //找到白点
//			{
//				CX = j - 1; //得到上一行C点X位置
//				break;
//			}
//		}
//		if (imagein[i - 1][CX] == 1) //判断上方还有没有黑点
//		{
//			CY = i;   //得到C点Y位置
//			break;
//		}
//	}
//
//	DY = BY - 1; //迭代D点
//	DX = BX - 1;
//	for (int i = DY; i > 0; i--) //由近及远
//	{
//		for (int j = DX; j > 0; i--) //由左向右
//		{
//			if (imagein[i][j] == 0) //找到白点
//			{
//				DX = j + 1; //得到上一行D点X位置
//				break; //跳出此循环
//			}
//		}
//		if (imagein[i - 1][DX] == 1) //判断上方还有没有黑点
//		{
//			DY = i;   //得到C点Y位置
//			break;//跳出此循环
//		}
//	}
//
//	if (abs(CY - DY) < 10 && CY > 30 && DY > 30)  //初级判断十字路口
//	{
//		uint8_t Y = min(CY, DY);   //取得CD高度较小值
//		uint8_t HEI = 0;          //十字路口上方区域黑点数量
//		for (i = Y; i > Y - 10; i -= 2)  //Y抽点轮询
//		{
//			for (j = 10; j < 70; j += 5) //X抽点轮询
//			{
//				if (imagein[i][j] == 1)  //有黑点
//				{
//					HEI++;  //计数变量++
//				}
//			}
//		}
//
//		if (HEI < 10) //最终判断十字路口，并补线
//		{
//			float K;    //补线斜率
//			K = (CX - AX) / (CY - AY); //计算AC点斜率
//
//			for (i = CY; i > CY - 20; i--)  //补AC延长2像素宽线
//			{
//				imagein[i][CX + (CY - i) * K] = 1;     //把图像对应点涂黑
//				imagein[i][(CX + (CY - i) * K) - 1] = 1; //把图像对应点涂黑
//			}
//
//			K = (DX - BX) / (DY - BY); //计算BD点斜率
//
//			for (i = DY; i > DY - 20; i--)  //补BD延长2像素宽线
//			{
//				imagein[i][DX + (DY - i) * K] = 1;     //把图像对应点涂黑
//				imagein[i][(DX + (DY - i) * K) - 1] = 1; //把图像对应点涂黑
//			}
//		}
//	}
//}
//
///*************************************************************/
//
///*************************找中线******************************/
//uint8_t ZHONGJIAN[60] = [39]; //中线位置
//uint8_t ZUO[60] = { 0 }; //左线位置
//uint8_t YOU[60] = { 79 }; //右线位置
//
//ZHONGJIAN[59] = (AX + BX) / 2; //最底下一行中心线位置找到
//
//for (i = 58; i > 0; i--) //向上迭代求剩下中心线
//{
//	for (j = ZHONGJIAN[i + 1]; j >= 1; j--) //从中间向左找上升沿
//	{
//		if (imagein[i][j - 1] - imagein[i][j] == 1) //找到上升沿
//		{
//			ZUO[i] = j; //左线
//		}
//	}
//	for (j = ZHONGJIAN[i + 1]; j < 79; j++) //从中间向左找上升沿
//	{
//		if (imagein[i][j + 1] - imagein[i][j] == 1) //找到上升沿
//		{
//			ZUO[i] = j; //左线
//		}
//	}
//	ZHONGJIAN[i] = (ZUO[i] + YOU[i]) / 2; //计算当前行中心点
//}
///*************************************************************************/
//
///***************************求车身偏差************************************/
//uint8_t QIANZHAN = 15;  //摄像头前瞻
//uint8_t YUAN, ZHONG, JIN;
//
//char ERR = 0; //前瞻偏差
//char YERR = 0; //车身横向偏差
//
//JIN = ZHONGJIAN[59];
//ZHONG = ZHONGJIAN[59 - QIANZHAN];
//YUAN = ZHONGJIAN[59 - QIANZHAN * 2];
//
//if (YUAN < ZHONG && ZHONG < JIN)   //情况1
//{
//	ERR = ((ZHONG - YUAN) + (JIN - ZHONG)) / 2;  //获取前瞻偏差
//}
//else if (YUAN < ZHONG && ZHONG >= JIN)//情况2
//{
//	ERR = JIN - ZHONG;   //获取前瞻偏差
//}
//else if (YUAN >= ZHONG && ZHONG < JIN)//情况3
//{
//	ERR = JIN - ZHONG;   //获取前瞻偏差
//}
//else //情况4
//{
//	ERR = ((ZHONG - YUAN) + (JIN - ZHONG)) / 2;  //获取前瞻偏差
//}
//YERR = JIN - 39; //获取车身横向偏差

/************************************************************************/

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
  //		/* 按键按下 显示原始图形 */
  //		else
  //		{
  //			TFTSPI_Road(0, 0, MT9V03X_H, MT9V03X_W, (uint8_t*)Image_Use);      //图像显示
  //		}
  //
  //	}
  //}





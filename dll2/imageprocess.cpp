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


#include "nosaydie_camera.h"
//#include "headfile.h"
#include "common.h"
#include "nosaydie_camera_bianliang.h"
  //����ȫ�ֱ���-------------------------------------------
  //  ͼ��
  //      ͼ��Ԥ����
uint8 threshold;                  //��ֵ����ֵ
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
uint8 image[MT9V03X_H][MT9V03X_W];        //ʹ�ö�άָ��image������Ϣͼ�񣬱��ں��ڶ�ֵ��ͼ������ͼ���ת��
uint8 binimage_flag = 1;          //��ֵ����־��Ĭ��û�ж�ֵ��
struct LEFT_EDGE L_edge[150];     //��߽�ṹ��
struct LEFT_EDGE L_edge2[150];
struct LEFT_EDGE R_edge[150];    //�ұ߽�ṹ��
struct LEFT_EDGE M_Line[150];     //���߽ṹ�壬����ѡ��2
struct LEFT_EDGE Last_M_Line[150];//�ϴ����߽ṹ��
struct LEFT_EDGE MID_LINE[150];   //���߽ṹ�壬����ѡ��1

struct LEFT_EDGE L_edge_use[120];     //��߽�ṹ�� ���ʹ��
struct LEFT_EDGE R_edge_use[120];    //�ұ߽�ṹ�� ���ʹ��

struct LEFT_EDGE* Mid_Line= M_Line;       //���߽ṹ�塾ʵ��ʹ�á�
//      �������ߴ���
uint8 searchpoint = 1;            //��������������ߵ��־�����ڽṹ��.flag�����ں��ڴ���
uint8 nihepoint = 2;              //�������������ߵ��־�����ڽṹ��.flag�����ں��ڴ���
extern uint8 Road_Width[MT9V03X_H];//�����أ����ڶ���
//      ͼ����ʾ
uint8 l_display_cnt = 0;
uint8 r_display_cnt = 0;

/*--------------------------------------------------------------------------
* ���������ܡ�������gui����
* ����    ��������
* ���� �� ֵ������
*--------------------------------------------------------------------------*/
void git_gui_value(uint8 value_in[30])
{
	for (int i = 0; i < 30; i++)
	{
		value3[i] = value_in[i];
	}
}
/*--------------------------------------------------------------------------
* ���������ܡ���ͼ�������ߡ�Ԫ��ʶ��_����������Խ��
* ����    ��������
* ���� �� ֵ������
*--------------------------------------------------------------------------*/
int x1, x2, x3, x4;
uint8* Image_process(uint8 imagein[LCDH][LCDW],uint8 sidee[150*2*3+100+240*2])
{
	//�������
	//  ȫ�֡��ⲿ����--------------------------
	//      ȫ�ֱ���
	//      �ⲿ����
	//  �����ڲ�����-------------------
	//      һ��ͼ����
	uint8 Bottom2 = Image_H - 40;   //�����ڶ���
	uint8 max_col = Image_W - 5, min_col = 5;         //���/С�����꣬���߽�ֵ
	uint8 L_search_amount = 150, R_search_amount = 150;  //���ұ߽��ѵ�ʱ�������ĵ�
	uint8 jilu_row_l = 0, jilu_col_l = 0, jilu_row_r = 0, jilu_col_r = 0;  //��¼�������Ļ����߽������ֵ
   //ͼ����-----------------------
   //  һ��ͼ����
   //      ��ʼ����ر���
	enable_check_l_r_edge_same = 0;    //ʹ�ܼ�����ұ����Ƿ����غϣ�Ĭ�ϲ���������������ʼ�����ʱ����
	left_findflag = 0;                 //��߽���ڱ�־��1�ҵ���߽磬0û�ҵ���߽�        Ĭ��û���ҵ���߽�
	right_findflag = 0;                //�ұ߽���ڱ�־��1�ҵ��б߽磬0û�ҵ��ұ߽�        Ĭ��û���ҵ��ұ߽�
	L_basic_row_start = Image_H - 2;    //��ʼ���ߵ�
	R_basic_row_start = Image_H - 2;    //�ҿ�ʼ���ߵ�
   //      ��ȡͼ���ֵ����ֵ��ͼ���ֵ��
	//get_deal_image();

	/* ��ȡgui���ݲ��� */
	for (int i = 0; i < LCDH; i++)
	{
		for (int j = 0; j < LCDW; j++)
		{
			image[i][j] = imagein[i][j];
			//printf("%d", image[i][j]);
		}
	}

	L_basic_row_start = value3[0];
	R_basic_row_start = value3[1];
	L_edge_start_col = value3[2];
	R_edge_start_col = value3[3];
	L_search_amount = value3[4];
	R_search_amount = value3[5];
	min_col = value3[6];
	max_col = value3[7];
	L_lost_ = value3[8];
	R_lost_ = value3[9];
	L_edge_lost_start_col = value3[10];
	R_edge_lost_start_col = value3[11];
	dist = value3[12];

	clear_point();//�߽��ʼ��

	//average_filter();
	//threshold = OSTU_bin(Image_W,Image_H,mt9v03x_image);//��ȡ��̬��ֵ
	//threshold = GetOSTUThreshold(image, 0, Image_H - 1, 0, Image_W - 1);
	//get_binImage(threshold);//��ȡ��ֵ��ͼ��
	//      ��ʼ������
	enable_balinyu = 1;
	if (enable_balinyu)//���ʹ�ܰ��������߼�һ�㴦��
	{
		//��ر���
		line_lose_center_left = 0;
		line_lose_center_right = 0;
		line_point_count_left = 0;
		line_point_count_right = 0;
		L_edge_count = 0;//��ߵ������0
		R_edge_count = 0;//�ұߵ������0
		int exist_edge_size = 0;   //�ж��Ƿ������/�ұ߽�

		/* Ѱ��������ʼ�� */
		//Ѱ����/���߿�ʼ�㣬���ж��Ƿ���ڵ�ǰ��
		clear_point();
		exist_edge_size = edge_point_ornot(L_basic_row_start, 0);
		if (exist_edge_size >= 0) { jilu_row_l = L_basic_row_start; jilu_col_l = exist_edge_size; left_findflag = 1; }
		//printf("�������ʼ�㣺(x=%d,y=%d)\n", jilu_row_l, jilu_col_l);

		exist_edge_size = edge_point_ornot(R_basic_row_start, (uint8)(1));
		if (exist_edge_size >= 0) { jilu_row_r = R_basic_row_start; jilu_col_r = exist_edge_size; right_findflag = 1; }
		//printf("�ұ�����ʼ�㣺(x=%d,y=%d)\n", jilu_row_r, jilu_col_r);
		
		/* ���а���������  */
		if (left_findflag)//�����߽����ڲ��ҵ�,��ʼ����
		{
			//��������
			L_edge[0].row = jilu_row_l;
			L_edge[0].col = jilu_col_l;
			L_edge[0].flag = 1;
			uint8 curr_row = jilu_row_l;//��ʼ��������
			uint8 curr_col = jilu_col_l;//��ʼ��������
			dire_left = 0; //��ʼ���ϸ��߽�������
			center_turn_flag = 1;//��ʼ����δ�궨״̬
			center_lost_flag_l = 0;//�м����߱�־λ
			//��ʼ���ߣ����ȡ150���㣬���������ѣ���7����λ
			findline_lefthand_adaptive(jilu_row_l, jilu_col_l, L_search_amount);
			//for (int i = 1; i < L_search_amount; i++)    //�������70����
			//{
			//	
			//	////Խ���˳� ��Խ�����Խ�磨���������������ң�
			//	if (curr_row < L_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < L_edge_end_row)  break;
			//	if (curr_col > max_col || curr_col < min_col && !center_lost_flag_l)   //max=160-5 min=5
			//	{
			//		if (++L_search_edge_count == 3)//����3���������߽磬�˳�      //�����������߽������Ҫ����  5��155��Ҫ����
			//		{
			//			curr_col = L_edge_start_col;
			//			
			//			for (uint8 rowi = curr_row; rowi > 0; rowi--)
			//			{
			//				//printf("row=%d", rowi);
			//				if (black_(image[rowi -3][curr_col]) && black_(image[rowi -2][curr_col]) && white_(image[rowi -1][curr_col]) && white_(image[rowi][curr_col]))
			//				{
			//					printf("�м�����������������ʼ�㣺(x=%d,y=%d)\n", rowi - 2, L_edge_start_col);
			//					curr_row = rowi - 2;
			//					curr_col = L_edge_start_col;
			//					center_lost_flag_l = 1;
			//					center_lost_row_l = curr_row;     //�м����߿�ʼ������
			//					center_lost_col_l = curr_col + 1; //�м����߿�ʼ������
			//					dire_left = 0; //��ʼ���ϸ��߽�������
			//					L_top_corner_start = i;//���Ϲյ㿪ʼ���
			//					break;
			//				}
			//			}
			//		}
			//	}
			//	else
			//		L_search_edge_count = 0;
			//	//���߹���
			//	//printf("dir:%d ", dire_left);
			//	if (dire_left != 2 && black_(image[curr_row - 1][curr_col - 1]) && white_(image[curr_row - 1][curr_col]))   //���Ϻڣ�2���ұ߰�
			//	{   
			//		//printf("stg:%d \n", 1);
			//		curr_row = curr_row - 1;
			//		curr_col = curr_col - 1;
			//		L_edge_count = L_edge_count + 1;
			//		dire_left = 7;
			//		L_edge[i].row = curr_row;
			//		L_edge[i].col = curr_col;
			//		L_edge[i].flag = 1;
			//	}
			//	else if (dire_left != 3 && black_(image[curr_row - 1][curr_col + 1]) && white_(image[curr_row][curr_col + 1]))    //���Ϻڣ�3���±߰�
			//	{
			//		//printf("stg:%d \n", 2);
			//		curr_row = curr_row - 1;
			//		curr_col = curr_col + 1;
			//		L_edge_count = L_edge_count + 1;
			//		dire_left = 6;
			//		L_edge[i].row = curr_row;
			//		L_edge[i].col = curr_col;
			//		L_edge[i].flag = 1;
			//	}
			//	else if (black_(image[curr_row - 1][curr_col]) && white_(image[curr_row - 1][curr_col + 1]))                  //���Ϻڣ�1���Ұ�
			//	{
			//		//printf("stg:%d \n", 3);
			//		curr_row = curr_row - 1;
			//		L_edge_count = L_edge_count + 1;
			//		dire_left = 0;
			//		L_edge[i].row = curr_row;
			//		L_edge[i].col = curr_col;
			//		L_edge[i].flag = 1;
			//	}
			//	else if (dire_left != 5 && black_(image[curr_row][curr_col - 1]) && white_(image[curr_row - 1][curr_col - 1]))     //����ڣ�5���ϰ�
			//	{
			//		//printf("stg:%d \n", 4);
			//		curr_col = curr_col - 1;
			//		L_edge_count = L_edge_count + 1;
			//		dire_left = 4;
			//		L_edge[i].row = curr_row;
			//		L_edge[i].col = curr_col;
			//		L_edge[i].flag = 1;
			//	}
			//	else if (dire_left != 4 && black_(image[curr_row][curr_col + 1]) && white_(image[curr_row + 1][curr_col + 1]))  //���Һڣ�4���°�
			//	{
			//		//printf("stg:%d \n", 5);
			//		curr_col = curr_col + 1;
			//		L_edge_count = L_edge_count + 1;
			//		dire_left = 5;
			//		L_edge[i].row = curr_row;
			//		L_edge[i].col = curr_col;
			//		L_edge[i].flag = 1;
			//	}
			//	else if (dire_left != 6 && black_(image[curr_row + 1][curr_col - 1]) && white_(image[curr_row][curr_col - 1]))    //���ºڣ�6���ϰ�
			//	{
			//		//printf("stg:%d \n", 6);
			//		curr_row = curr_row + 1;
			//		curr_col = curr_col - 1;
			//		L_edge_count = L_edge_count + 1;
			//		dire_left = 3;
			//		L_edge[i].row = curr_row;
			//		L_edge[i].col = curr_col;
			//		L_edge[i].flag = 1;
			//	}
			//	else if (dire_left != 7 && black_(image[curr_row + 1][curr_col + 1]) && white_(image[curr_row + 1][curr_col]))    //���ºڣ�7�����
			//	{
			//		//printf("stg:%d \n", 7);
			//		curr_row = curr_row + 1;
			//		curr_col = curr_col + 1;
			//		L_edge_count = L_edge_count + 1;
			//		dire_left = 2;
			//		L_edge[i].row = curr_row;
			//		L_edge[i].col = curr_col;
			//		L_edge[i].flag = 1;
			//	}
			//	else
			//		break;
			//}
		}
		if (right_findflag)//����ұ߽���ڲ��ѵ�
		{
			R_edge[0].row = jilu_row_r;
			R_edge[0].col = jilu_col_r;
			R_edge[0].flag = 1;
			uint8 curr_row = jilu_row_r;
			uint8 curr_col = jilu_col_r;
			dire_right = 0;
			center_lost_flag_r = 0; //�м��Ҷ��߱�־λ
			findline_righthand_adaptive(jilu_row_r, jilu_col_r, R_search_amount);
			//for (int i = 1; i < R_search_amount; i++)
			//{
			//	//Խ���˳� ��Խ�����Խ�磨���������������ң�
			//	if (curr_row < R_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < R_edge_end_row)  break;
			//	if (curr_col > max_col || curr_col < min_col && !center_lost_flag_r)   //���������������߽磬�˳�
			//	{
			//		if (++R_search_edge_count == 3)
			//		{
			//			curr_col = R_edge_start_col;
			//			for (uint8 rowi = curr_row; rowi > 0; rowi--)
			//			{
			//				
			//				if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
			//				{
			//					printf("�м��Ҷ���������ұ�����ʼ�㣺(x=%d,y=%d)\n", rowi - 2, R_edge_start_col);
			//					curr_row = rowi - 2;
			//					curr_col = R_edge_start_col;
			//					center_lost_flag_r = 1;
			//					center_lost_row_r = curr_row;     //�м��Ҷ��߿�ʼ������
			//					center_lost_col_r = curr_col - 1; //�м��Ҷ��߿�ʼ������
			//					dire_right = 0;
			//					R_top_corner_start = i;//���Ϲյ㿪ʼ���
			//					break;
			//				}
			//			}
			//		}
			//	}
			//	else   R_search_edge_count = 0;
			//	//���߹���
			//	if (curr_col < Image_W && dire_right != 3 && black_(image[curr_row - 1][curr_col + 1]) && white_(image[curr_row - 1][curr_col]))    //���Ϻڣ�3�����
			//	{
			//		curr_row = curr_row - 1;
			//		curr_col = curr_col + 1;
			//		R_edge_count = R_edge_count + 1;
			//		dire_right = 6;
			//		R_edge[i].row = curr_row;
			//		R_edge[i].col = curr_col;
			//		R_edge[i].flag = 1;
			//	}
			//	else if (dire_right != 2 && black_(image[curr_row - 1][curr_col - 1]) && white_(image[curr_row][curr_col - 1])) //���Ϻڣ�2���°�
			//	{
			//		curr_row = curr_row - 1;
			//		curr_col = curr_col - 1;
			//		R_edge_count = R_edge_count + 1;
			//		dire_right = 7;
			//		R_edge[i].row = curr_row;
			//		R_edge[i].col = curr_col;
			//		R_edge[i].flag = 1;
			//	}
			//	else if (black_(image[curr_row - 1][curr_col]) && white_(image[curr_row - 1][curr_col - 1]))                  //���Ϻڣ�1�����
			//	{
			//		curr_row = curr_row - 1;
			//		R_edge_count = R_edge_count + 1;
			//		dire_right = 0;
			//		R_edge[i].row = curr_row;
			//		R_edge[i].col = curr_col;
			//		R_edge[i].flag = 1;
			//	}
			//	else if (dire_right != 5 && black_(image[curr_row][curr_col - 1]) && white_(image[curr_row + 1][curr_col - 1]))   //����ڣ�5���°�
			//	{
			//		curr_col = curr_col - 1;
			//		R_edge_count = R_edge_count + 1;
			//		dire_right = 4;
			//		R_edge[i].row = curr_row;
			//		R_edge[i].col = curr_col;
			//		R_edge[i].flag = 1;
			//	}
			//	else if (dire_right != 4 && black_(image[curr_row][curr_col + 1]) && white_(image[curr_row - 1][curr_col + 1]))   //���Һڣ�4���ϰ�
			//	{
			//		curr_col = curr_col + 1;
			//		R_edge_count = R_edge_count + 1;
			//		dire_right = 5;
			//		R_edge[i].row = curr_row;
			//		R_edge[i].col = curr_col;
			//		R_edge[i].flag = 1;
			//	}
			//	else if (dire_right != 6 && black_(image[curr_row + 1][curr_col - 1]) && white_(image[curr_row + 1][curr_col]))   //���ºڣ�6���Ұ�
			//	{
			//		curr_row = curr_row + 1;
			//		curr_col = curr_col - 1;
			//		R_edge_count = R_edge_count + 1;
			//		dire_right = 3;
			//		R_edge[i].row = curr_row;
			//		R_edge[i].col = curr_col;
			//		R_edge[i].flag = 1;
			//	}
			//	else if (dire_right != 7 && black_(image[curr_row + 1][curr_col + 1]) && white_(image[curr_row][curr_col + 1]))   //���ºڣ�7���ϰ�
			//	{
			//		curr_row = curr_row + 1;
			//		curr_col = curr_col + 1;
			//		R_edge_count = R_edge_count + 1;
			//		dire_right = 2;
			//		R_edge[i].row = curr_row;
			//		R_edge[i].col = curr_col;
			//		R_edge[i].flag = 1;
			//	}
			//	else
			//		break;
			//}
		}

		/* �߽�Ԥ����  */
		/* ���Ķα߽�ֱ���� */
		// ��ر�����ʼ��
		pre_L_edge_count = 0;
		pre_R_edge_count = 0;
		// ��߽紦��
		if (left_findflag)//�����߽���ڲ��ҵ�
		{
			//blur_points(L_edge,L_edge2, L_edge_count, (int)round(line_blur_kernel));
		
			//ƽ���߽�
			//for (int i = 1; i < L_edge_count - 1; i++)
			//{
			//	L_edge[i].row = (L_edge[i].row + L_edge[i + 1].row) / 2;
			//	L_edge[i].col = (L_edge[i].col + L_edge[i + 1].col) / 2;
			//}
			//for (int i = L_edge_count - 1; i > 1; i--)
			//{
			//	L_edge[i].row = (L_edge[i].row + L_edge[i - 1].row) / 2;
			//	L_edge[i].col = (L_edge[i].col + L_edge[i - 1].col) / 2;
			//}
			
			edge_process_flag = 0; //�����־

			//�ض�����ˮƽ�ı߽�
			//if (L_edge_count >= 70)
			//{
			//	num_cnt = 0;//��¼����ˮƽ��ĸ���
			//	L_count = L_edge_count / 2;
			//	while (L_count < L_edge_count)
			//	{
			//		if (L_edge[L_count].row == L_edge[L_count + 1].row)
			//			num_cnt = num_cnt + 1;
			//		else
			//			num_cnt = 0;
			//		if (num_cnt > 5)//����5����ˮƽ
			//			break;
			//		L_count = L_count + 1;
			//	}
			//	L_edge_count = L_count;//�ض���5��ˮƽ�㴦
			//}
		}
		//�ұ߽紦��
		if (right_findflag)//�ұ߽�������ҵ�ʱ��ʼ����
		{
			//ƽ���߽�
			//for (int i = 1; i < R_edge_count - 1; i++)
			//{
			//	R_edge[i].row = (R_edge[i].row + R_edge[i + 1].row) / 2;
			//	R_edge[i].col = (R_edge[i].col + R_edge[i - 1].col) / 2;
			//}
			//for (int i = R_edge_count - 1; i > 1; i--)
			//{
			//	R_edge[i].row = (R_edge[i].row + R_edge[i + 1].row) / 2;
			//	R_edge[i].col = (R_edge[i].col + R_edge[i - 1].col) / 2;
			//}

			//�ض�����ˮƽ�ı߽�
			//if (R_edge_count >= 70)   //�߽繻��ʱ�ſ�ʼ����
			//{
			//	num_cnt = 0;
			//	R_count = R_edge_count / 2;
			//	while (R_count < R_edge_count)
			//	{
			//		if (R_edge[R_count].row == R_edge[R_count + 1].row)
			//			num_cnt = num_cnt + 1;
			//		else
			//			num_cnt = 0;
			//		if (num_cnt > 5)
			//			break;
			//		R_count = R_count + 1;
			//	}
			//	R_edge_count = R_count;
			//}
		}
		//����쳣�߽�
		// ���1�����ұ��ߺ��ͼ�������Width/2���������ұ��������غϵ����⣬ʹ���غϼ���
		//if (fabs(L_edge[0].col - L_edge[L_edge_count].col) > Image_W / 2 && fabs(L_edge[0].col - L_edge[L_edge_count].col) > Image_W / 2)
		//	enable_check_l_r_edge_same = 1;
		//if (enable_check_l_r_edge_same)
		//{
		//	uint8 i_left = L_edge_count - 1;
		//	uint8 chongdie_cnt = 0;
		//	for (int i = 0; i < R_edge_count; i++)
		//	{
		//		if (fabs(R_edge[i].row - L_edge[i_left].row) < 5 && fabs(R_edge[i].col - L_edge[i_left].col) < 5)
		//		{
		//			chongdie_cnt = chongdie_cnt + 1;
		//			i_left = i_left - 1;
		//			if (chongdie_cnt > 3) break;
		//			if (i_left < 0)   break;
		//		}
		//	}
		//	if (chongdie_cnt >= 2)
		//	{
		//		x2 = fabs(L_edge[0].row - R_edge[0].row);
		//		//���1�������-���������ұ߽�������ʼ����ֵ���
		//		if (fabs(L_edge[0].row - R_edge[0].row) >= 15)
		//		{
		//			if (L_edge[0].row > R_edge[0].row)        //�����ߵ�ߣ���������߶��ߣ��������
		//				left_findflag = 0;
		//			else if (R_edge[0].row > L_edge[0].row)   //�����ߵ�ߣ������ұ��߶��ߣ��������
		//				right_findflag = 0;
		//		}
		//		//���2����������/�ջ�ʮ�֡��������������޹յ㣨����·�ڱ����غ��йյ㡢�ջ�ʮ���޹յ㣩
		//		else if (fabs(L_edge[0].row - R_edge[0].row) < 15)
		//		{
		//		}
		//	}
		//}

		////�߽��̫�٣�ȥ��
		//if (L_edge_count < 10)
		//	left_findflag = 0;
		//if (R_edge_count < 10)
		//	right_findflag = 0;
		////�������ʼ������ұ���
		//if (left_findflag && right_findflag)
		//{
		//	if (jilu_col_l > jilu_col_r)
		//	{
		//		if (jilu_row_l > jilu_row_r)
		//			left_findflag = 0;
		//		else if (jilu_row_l < jilu_row_r)
		//			right_findflag = 0;
		//	}
		//}
		//if (L_edge_count - R_edge_count > 30 && right_findflag)  //���ұ߽��������̫����>>��
		//{
		//	right_findflag = 0;
		//	R_edge_count = 0;
		//}
		//if (R_edge_count - L_edge_count > 30 && left_findflag)   //��>>��
		//{
		//	left_findflag = 0;
		//	L_edge_count = 0;
		//}
		////�����/�ұ߽�̫�����ҵ������ϴ�ɾ������
		//if (left_findflag && right_findflag)
		//{
		//	if (jilu_row_r - jilu_row_l > Image_H / 2 && L_edge_count - R_edge_count > 10)
		//	{
		//		right_findflag = 0;
		//		R_edge_count = 0;
		//	}
		//	else if (jilu_row_l - jilu_row_r > Image_H / 2 && R_edge_count - L_edge_count > 10)
		//	{
		//		left_findflag = 0;
		//		L_edge_count = 0;
		//	}
		//}
		//if (L_edge[L_edge_count - 1].row - L_edge[1].row > -10)  //���߼���ˮƽ ��
		//	left_findflag = 0;
		//if (R_edge[R_edge_count - 1].row - R_edge[1].row > -10)  //��
		//	right_findflag = 0;
		////����ĳһ�߽磬�����������
		//if (left_findflag || right_findflag)
		//	enable_midline = 1;
		//else
		//	enable_midline = 0;
		//���յ�
		/* Ѱ�����¹յ� */
		L_corner_flag = 0;// ��ʼ������
		L_corner_row = 0;
		L_corner_col = 0;
		L_corner_angle = 0;
		if (enable_L_corner /*&& !L_start_lost*/) //���ʹ��������յ�,�ҿ�ʼδ����
		{
			if (L_edge_count > 2 * dist + 1)
			{
				//printf("\nL_ed:%d ", L_top_corner_start - 9);
				//for (int i = 0; i < L_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
				//{
				//	//printf("L_ed2:%d ", L_edge[i].row);
				//	if (L_edge[i + dist].row > dist+1)
				//	{
				//		int ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
				//			(L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
				//		//printf("ang3:%d ", ang);
				//		if ( ang<= 0) //����ȷ��Ϊ��ǻ���ֱ�� ������
				//		{
				//			//printf("L_ed3:%d ", L_edge[i].row);
				//			L_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //��Ƕ�
				//			L_corner_angle = 180 - L_corner_angle;
				//			if (L_edge[i + dist].col > L_edge[i + 2 * dist].col)    //ȷ���սǳ�����յ�û�г�������
				//			{
				//				L_corner_flag = 1;//�쳣�յ�
				//				L_corner_row = L_edge[i + dist].row;
				//				L_corner_col = L_edge[i + dist].col;
				//				//printf("��յ㣺(x=%d,y=%d)���Ƕ�=%d\n", L_corner_row, L_corner_col, L_corner_angle);
				//				break;
				//			}
				//		}
				//	}
				//}
				int max = 0;
				for (int i = 0; i < L_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
				{
					//printf("L_ed2:%d ", L_edge[i].row);
					if (L_edge[i + dist].row > dist + 1)
					{
						L_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //��Ƕ�
						L_corner_angle = 180 - L_corner_angle;
						//printf("L_ang:%d ", L_corner_angle);
						if (L_corner_angle > max)
						{
							max = L_corner_angle;
							L_corner_row = L_edge[i + dist].row;
							L_corner_col = L_edge[i + dist].col;
							L_beh_i = i + dist;
						}
					}
				}
				L_corner_angle = max;
			}
		}
		if(L_start_lost)
		{
			int min = 999;
			for (int i = 1; i < L_edge_count - (2 * dist + 1); i++)
			{
				//printf("ang:%d \n", Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col));
				if (L_edge[i + dist].row > dist + 1)
				{
					L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col);
					if (L_center_lost_corner_angle < min)
					{
						min = L_center_lost_corner_angle;
						center_lost_corner_row_l = L_edge[i + dist].row;
						center_lost_corner_col_l = L_edge[i + dist].col;
						L_top_i = i + dist;
					}
					//int ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
					//	(L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
					//if (ang <= 0) //����ȷ��Ϊ��ǻ���ֱ�� ������
					//{
					//	L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //��Ƕ�
					//	if (L_edge[i + dist].col < L_edge[i + 2 * dist].col && L_center_lost_corner_angle != 180)    //ȷ���սǳ�����յ�û�г�������
					//	{
					//		L_corner_flag = 1;//�쳣�յ�
					//		center_lost_corner_row_l = L_edge[i + dist].row;
					//		center_lost_corner_col_l = L_edge[i + dist].col;
					//		break;
					//	}
					//}
				}
			}
			L_center_lost_corner_angle = min;
		}
		if (center_lost_flag_l /*|| L_start_lost*/)
		{
			int min = 999;
			for (int i = L_top_corner_start; i < L_edge_count - (2 * dist + 1); i++)
			{
				//printf("ang:%d \n", Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col));
				if (L_edge[i + dist].row > dist+1)
				{
					L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col);
					if (L_center_lost_corner_angle < min)
					{
						min = L_center_lost_corner_angle;
						center_lost_corner_row_l = L_edge[i + dist].row;
						center_lost_corner_col_l = L_edge[i + dist].col;
						L_top_i = i + dist;
					}
					//int ang = (L_edge[i].col - L_edge[i + dist].col) * (L_edge[i + 2 * dist].col - L_edge[i + dist].col) +
					//	(L_edge[i].row - L_edge[i + dist].row) * (L_edge[i + 2 * dist].row - L_edge[i + dist].row);
					//if (ang <= 0 ) //����ȷ��Ϊ��ǻ���ֱ�� ������
					//{
					//	L_center_lost_corner_angle = Get_angle(L_edge[i].row, L_edge[i].col, L_edge[i + dist].row, L_edge[i + dist].col, L_edge[i + 2 * dist].row, L_edge[i + 2 * dist].col); //��Ƕ�
					//	if (L_edge[i + dist].col < L_edge[i + 2 * dist].col && L_center_lost_corner_angle!=180)    //ȷ���սǳ�����յ�û�г�������
					//	{
					//		L_corner_flag = 1;//�쳣�յ�
					//		center_lost_corner_row_l = L_edge[i + dist].row;
					//		center_lost_corner_col_l = L_edge[i + dist].col;
					//		break;
					//	}
					//}
				}
			}
			L_center_lost_corner_angle = min;
		}
		printf("���¹յ㣺(x=%d,y=%d)���Ƕ�=%d\n", L_corner_row, L_corner_col, L_corner_angle);
		printf("���Ϲյ㣺(x=%d,y=%d)���Ƕ�=%d\n", center_lost_corner_row_l, center_lost_corner_col_l, L_center_lost_corner_angle);
		R_corner_flag = 0;//��ʼ������
		R_corner_row = 0;
		R_corner_col = 0;
		R_corner_angle = 0;
		if (enable_R_corner /*&& !R_start_lost*/)    //���ʹ�������ҹյ�
		{
			if (R_edge_count > (2 * dist + 1))
			{
				//for (int i = 0; i < R_top_corner_start - (2 * dist + 1); i++)//R_edge_count - 9
				//{
				//	if (R_edge[i + dist].row > dist+1)
				//	{
				//		int ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
				//			(R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
				//		//printf("ang=%d ", ang);
				//		if ( ang<= 0) //����ȷ��Ϊ��ǻ���ֱ�� ������
				//		{
				//			R_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //��Ƕ�
				//			R_corner_angle = 180 - R_corner_angle;
				//			if (R_edge[i + 2 * dist].col > R_edge[i + dist].col)    //ȷ���սǳ�����յ�û�г�������
				//			{
				//				R_corner_flag = 1;//�쳣�յ�
				//				R_corner_row = R_edge[i + dist].row;
				//				R_corner_col = R_edge[i + dist].col;
				//				break;
				//			}
				//		}
				//	}
				//}
				int max = 0;
				for (int i = 0; i < R_top_corner_start - (2 * dist + 1); i++)//L_edge_count - 9
				{
					//printf("L_ed2:%d ", L_edge[i].row);
					if (R_edge[i + dist].row > dist + 1)
					{
						R_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //��Ƕ�
						R_corner_angle = 180 - R_corner_angle;
						//printf("R_ang:%d ", R_corner_angle);
						if (R_corner_angle > max)
						{
							max = R_corner_angle;
							R_corner_row = R_edge[i + dist].row;
							R_corner_col = R_edge[i + dist].col;
							R_beh_i = i + dist;
						}
					}
				}
				R_corner_angle = max;
			}
		}
		if (R_start_lost)
		{
			int min = 999;
			//printf("LLL%d", R_edge_count);
			//������70������  ��bug��Ҫ��
			for (int i = 0; i < 70 - (2 * dist + 1); i++)
			{
				if (R_edge[i + dist].row > dist + 1)
				{
					R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //��Ƕ�
					if (R_center_lost_corner_angle < min && R_center_lost_corner_angle>90)
					{
						min = R_center_lost_corner_angle;
						center_lost_corner_row_r = R_edge[i + dist].row;
						center_lost_corner_col_r = R_edge[i + dist].col;
						R_top_i = i + dist;
					}
					//int ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
					//	(R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
					////printf("ang2=%d ", ang);
					//if (ang <= 0) //����ȷ��Ϊ��ǻ���ֱ�� ������
					//{
					//	R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //��Ƕ�
					//	if (R_edge[i + dist].col > R_edge[i + 2 * dist].col && R_center_lost_corner_angle != 180)    //ȷ���սǳ�����յ�û�г�������
					//	{
					//		R_corner_flag = 1;//�쳣�յ�
					//		center_lost_corner_row_r = R_edge[i + dist].row;
					//		center_lost_corner_col_r = R_edge[i + dist].col;
					//		break;
					//	}
					//}
				}
			}
			R_center_lost_corner_angle = min;
		}
		if (center_lost_flag_r /*|| R_start_lost*/)
		{
			int min = 999;
			for (int i = R_top_corner_start; i < R_edge_count - (2 * dist + 1); i++)
			{
				if (R_edge[i + dist].row > dist+1)
				{
					R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //��Ƕ�
					if (R_center_lost_corner_angle < min)
					{
						min = R_center_lost_corner_angle;
						center_lost_corner_row_r = R_edge[i + dist].row;
						center_lost_corner_col_r = R_edge[i + dist].col;
						R_top_i = i + dist;
					}																																									  //int ang = (R_edge[i].col - R_edge[i + dist].col) * (R_edge[i + 2 * dist].col - R_edge[i + dist].col) +
					//	(R_edge[i].row - R_edge[i + dist].row) * (R_edge[i + 2 * dist].row - R_edge[i + dist].row);
					////printf("ang2=%d ", ang);
					//if (ang <= 0) //����ȷ��Ϊ��ǻ���ֱ�� ������
					//{
					//	R_center_lost_corner_angle = Get_angle(R_edge[i].row, R_edge[i].col, R_edge[i + dist].row, R_edge[i + dist].col, R_edge[i + 2 * dist].row, R_edge[i + 2 * dist].col); //��Ƕ�
					//	if (R_edge[i + dist].col > R_edge[i + 2 * dist].col && R_center_lost_corner_angle != 180)    //ȷ���սǳ�����յ�û�г�������
					//	{
					//		R_corner_flag = 1;//�쳣�յ�
					//		center_lost_corner_row_r = R_edge[i + dist].row;
					//		center_lost_corner_col_r = R_edge[i + dist].col;
					//		break;
					//	}
					//}
				}
			}
			R_center_lost_corner_angle = min;
		}
		printf("���¹յ㣺(x=%d,y=%d)���Ƕ�=%d\n", R_corner_row, R_corner_col, R_corner_angle);
		printf("���Ϲյ㣺(x=%d,y=%d)���Ƕ�=%d\n", center_lost_corner_row_r, center_lost_corner_col_r, R_center_lost_corner_angle);
		//�յ㴦��
		//���򣨲����ã�������ɾ��

	}
	edge_truncation(0);
	for (int i = 0; i < Image_H; i++)
	{
		L_edge_use[i].row = i;
		L_edge_use[i].col = (int)(i * k_l + b_l);
		//printf("hh(%d,%d)\n", L_edge_use[i].row, L_edge_use[i].col);
		if (L_edge_use[i].col < 0)
		{
			L_edge_use[i].col = 0;
		}
		if (L_edge_use[i].col > 159)
		{
			L_edge_use[i].col = 159;
		}
	}
	edge_truncation(1);
	
	for (int i = 0; i < Image_H; i++)
	{
		R_edge_use[i].row = i;
		R_edge_use[i].col = (int)(i * k_r + b_r);
		if (R_edge_use[i].col > 159)
		{
			R_edge_use[i].col = 159;
		}
		if (R_edge_use[i].col < 0)
		{
			R_edge_use[i].col = 0;
		}
	}
	
	//���ʹ�����ߴ�����ʼ��������
	//if (enable_midline)
	//	get_mid();
	

	for (int i = 0,m=0; i < 300; i=i+2,m++)
	{
		sidee[i] = L_edge[m].row;
		sidee[i+1] = L_edge[m].col;
		if (sidee[i] >119) sidee[i] = 119;
		if (sidee[i+1] >160) sidee[i+1] = 159;
		//if (sidee[i] == 0) sidee[i] = 1;
		//if (sidee[i+1] == 0) sidee[i+1] = 1;
		//printf("%d ", sidee[i]);
	}
	for (int i = 300, m = 0; i < 600; i = i + 2, m++)
	{
		sidee[i] = R_edge[m].row;
		sidee[i + 1] = R_edge[m].col;
		if (sidee[i] > 119) sidee[i] = 119;
		if (sidee[i + 1] > 160) sidee[i + 1] = 159;
		//if (sidee[i] == 0) sidee[i] = 1;
		//if (sidee[i + 1] == 0) sidee[i + 1] = 1;
	}
	for (int i = 600, m = 0; i < 900; i = i + 2, m++)
	{
		sidee[i] = Mid_Line[m].row;
		sidee[i + 1] = Mid_Line[m].col;
		if (sidee[i] > 119) sidee[i] = 119;
		if (sidee[i + 1] > 160) sidee[i + 1] = 159;
		//if (sidee[i] == 0) sidee[i] = 1;
		//if (sidee[i + 1] == 0) sidee[i + 1] = 1;
	}
	sidee[900] = jilu_row_l; //�������ʼ��
	sidee[901] = jilu_col_l; 
	sidee[902] = jilu_row_r; //�ұ�����ʼ��
	sidee[903] = jilu_col_r; 
	
	sidee[904] = L_basic_row_start;//����������������ʼ��
	sidee[905] = L_edge_start_col;
	sidee[906] = R_basic_row_start;//����������ұ�����ʼ��
	sidee[907] = R_edge_start_col;

	sidee[908] = center_lost_row_l;//�м�����������������ʼ��
	sidee[909] = center_lost_col_l;
	sidee[910] = center_lost_row_r;//�м��Ҷ���������������ʼ��
	sidee[911] = center_lost_col_r;

	sidee[912] = L_corner_row;//���¹յ�
	sidee[913] = L_corner_col; 
	sidee[914] = L_corner_angle;

	sidee[915] = R_corner_row;//���¹յ�
	sidee[916] = R_corner_col;
	sidee[917] = R_corner_angle;

	sidee[918] = center_lost_corner_row_l;//���Ϲյ�
	sidee[919] = center_lost_corner_col_l;
	sidee[920] = L_center_lost_corner_angle;

	sidee[921] = center_lost_corner_row_r;//���Ϲյ�
	sidee[922] = center_lost_corner_col_r;
	sidee[923] = R_center_lost_corner_angle;

	for (int i = 1000, m = 0; i < 1240; i = i + 2, m++)
	{
		sidee[i] = L_edge_use[m].row;
		sidee[i + 1] = L_edge_use[m].col;
		//printf("hhhh(%d,%d)\n", L_edge_use[m].row, L_edge_use[m].col);
	}
	for (int i = 1240, m = 0; i < 1480; i = i + 2, m++)
	{
		sidee[i] = R_edge_use[m].row;
		sidee[i + 1] = R_edge_use[m].col;
	}

	return sidee;
}
/*--------------------------------------------------------------------------
* ���������ܡ�����ȡ����
* ����    ��������
* ���� �� ֵ������
* ����    ע������
*--------------------------------------------------------------------------*/
int size1 = 0;
void get_mid()
{
	size1 = 1;
	//��ر���
	uint8 mid_cnt = 0;      //����������ϣ����������ϣ������ڼ�¼�µ����ߵ����
	uint8 last_mid_count = 0; //���ڼ�¼��һ�����ߵ����߸�������������ƽ��
	uint8 up_cnt = 15;        //�������������Ƹ���
	//������һ����������
	for (int i = 0; i < Mid_count; i++)
	{
		Last_M_Line[i].row = Mid_Line[i].row;
		Last_M_Line[i].col = Mid_Line[i].col;
	}
	last_mid_count = Mid_count;
	//���1�����Ҿ��ҵ�
	if (left_findflag && right_findflag)
	{
		size1 = 2;
		//��ʼ�����߸���
		Mid_count = 0;
		//��ֹ���߳���
		L_edge[0].row = L_edge[1].row;
		L_edge[0].col = L_edge[1].col;
		R_edge[0].row = R_edge[1].row;
		R_edge[0].col = R_edge[1].col;
		//����߱��ұ��߳�
		if (L_edge_count >= R_edge_count)
		{
			float k = 1.0 * L_edge_count / R_edge_count;
			for (int i = 0; i < R_edge_count; i++)
			{
				Mid_count = Mid_count + 1;
				M_Line[i].row = (uint8)((L_edge[(uint8)(k * i)].row + R_edge[i].row) / 2);
				M_Line[i].col = (uint8)((L_edge[(uint8)(k * i)].col + R_edge[i].col) / 2);
				M_Line[i].flag = searchpoint;
			}
		}
		//�ұ��߱�����߳�
		else if (L_edge_count < R_edge_count)
		{
			float k = 1.0 * R_edge_count / L_edge_count;
			for (int i = 0; i < L_edge_count; i++)
			{
				Mid_count = Mid_count + 1;
				M_Line[i].row = (uint8)((L_edge[i].row + R_edge[(uint8)(k * i)].row) / 2);
				M_Line[i].col = (uint8)((L_edge[i].col + R_edge[(uint8)(k * i)].col) / 2);
				M_Line[i].flag = searchpoint;
			}
		}
	}
	//���2������ȫ����
	//      �ұ߽綪�ߣ���߽粻����||������������
	else if ((left_findflag == 1 && right_findflag == 0) || (left_findflag && L_edge_count - R_edge_count > 70))
	{
		//��ֹ���߳���
		L_edge[0].row = L_edge[1].row;
		L_edge[0].col = L_edge[1].col;
		//��ʼ�����߸���
		Mid_count = 0;
		for (int i = 0; i < L_edge_count; i++)
		{
			int16 col = ((2 * (int16)L_edge[i].col + (int16)Road_Width[Image_H - L_edge[i].row]) / 2);
			if (col > Image_W - 1)    col = Image_W - 1;
			else if (col < 0)  col = 0;
			Mid_count = Mid_count + 1;
			M_Line[i].row = L_edge[i].row;
			M_Line[i].col = (uint8)col;
			M_Line[i].flag = searchpoint;
		}
	}
	//      ����߶��ߣ��ұ��߲�����
	else if ((left_findflag == 0 && right_findflag == 1) || (right_findflag && R_edge_count - L_edge_count > 70))
	{
		//��ֹ���߳���
		R_edge[0].row = R_edge[1].row;
		R_edge[0].col = R_edge[1].col;
		//��ʼ�����߸���
		Mid_count = 0;
		for (int i = 0; i < R_edge_count; i++)
		{
			int16 col = ((2 * (int16)R_edge[i].col - (int16)Road_Width[Image_H - R_edge[i].row]) / 2);
			if (col > Image_W - 1)    col = Image_W - 1;
			else if (col < 0)  col = 0;
			Mid_count = Mid_count + 1;
			M_Line[i].row = R_edge[i].row;
			M_Line[i].col = (uint8)col;
			M_Line[i].flag = searchpoint;
		}
	}
	//���3������ȫ����
	//���4�����ұ߾����ڣ�������̫��
	if (Mid_count > 0)
	{
		uint8 down_cnt = 15;
		//��ͷ��������û�ҵ������������
		if (M_Line[0].row < Image_H - 2 && M_Line[0].row > Image_H / 4 && Mid_count > 2)
		{
			int num = Mid_count / 4;
			int sumX = 0, sumY = 0;
			float sumUP = 0, sumDown = 0, avrX = 0, avrY = 0, K=0, B=0;
			for (int i = 0; i < num; i++)
			{
				sumX = sumX + M_Line[i].row;
				sumY = sumY + M_Line[i].col;
			}
			avrX = 1.0 * sumX / num;
			avrY = 1.0 * sumY / num;
			for (int i = 0; i < num; i++)
			{
				sumUP = sumUP + (M_Line[i].col - avrY) * (M_Line[i].row - avrX);
				sumDown = sumDown + (M_Line[i].row - avrX) * (M_Line[i].row - avrX);
			}
			if (sumDown == 0)
			{
				K = 0;
			}
			else
			{
				K = 1.0 * sumUP / sumDown;
				B = 1.0 * (sumY - K * sumX) / num;
			}
			for (int i = M_Line[0].row; i < Image_H; i++)  //��ʼ�������
			{
				int col = K * i + B;
				if (col > Image_W - 1) col = Image_W - 1;
				else if (col < 0)    col = 0;
				MID_LINE[mid_cnt].row = (uint8)i;
				MID_LINE[mid_cnt].col = (uint8)col;
				mid_cnt = mid_cnt + 1;
				if (--down_cnt == 0) break;
			}
		}
		//���ߵ����̫�٣������������
		if (Mid_count + mid_cnt < 60 && Mid_count >2)
		{
			int num = Mid_count / 4;
			int sumX = 0, sumY = 0;
			float sumUP = 0, sumDown = 0, avrX = 0, avrY = 0, K, B;
			for (int i = Mid_count - num - 1; i < Mid_count; i++)
			{
				sumX = sumX + M_Line[i].row;
				sumY = sumY + M_Line[i].col;
			}
			avrX = 1.0 * sumX / num;
			avrY = 1.0 * sumY / num;
			for (int i = Mid_count - num - 1; i < Mid_count; i++)
			{
				sumUP = sumUP + (M_Line[i].col - avrY) * (M_Line[i].row - avrX);
				sumDown = sumDown + (M_Line[i].row - avrX) * (M_Line[i].row - avrX);
			}
			if (sumDown == 0)
			{
				K = 0;
			}
			else
			{
				K = 1.0 * sumUP / sumDown;
				B = 1.0 * (sumY - K * sumX) / num;
			}
			for (int i = M_Line[Mid_count - 1].row; i > 0; i--)  //��ʼ�������
			{
				int col = K * i + B;
				if (col > Image_W - 1) col = Image_W - 1;
				else if (col < 0)    col = 0;
				M_Line[Mid_count].row = (uint8)i;
				M_Line[Mid_count].col = (uint8)col;
				Mid_count = Mid_count + 1;
				if (--up_cnt == 0) break;
			}
		}
		//�����������ˣ���������ߴ�����
		if (mid_cnt > 0)
		{
			for (int i = 0; i < Mid_count; i++)
			{
				MID_LINE[mid_cnt].row = M_Line[i].row;
				MID_LINE[mid_cnt].col = M_Line[i].col;
				mid_cnt = mid_cnt + 1;
			}
		}
	}
	if (mid_cnt > 0)
	{
		Mid_count = mid_cnt;
		Mid_Line = MID_LINE;
	}
	else
		Mid_Line = M_Line;
	//����ƽ��
	float k = 0.8;
	float k2;
	for (int i = 0; i < Mid_count - 1; i++)
	{
		Mid_Line[i].row = (Mid_Line[i].row + Mid_Line[i + 1].row) / 2;
		Mid_Line[i].col = (Mid_Line[i].col + Mid_Line[i + 1].col) / 2;
	}
	for (int i = Mid_count - 1; i > 1; i--)
	{
		Mid_Line[i].row = (Mid_Line[i].row + Mid_Line[i - 1].row) / 2;
		Mid_Line[i].col = (Mid_Line[i].col + Mid_Line[i - 1].col) / 2;
	}
	if (last_mid_count >= Mid_count)
	{
		k2 = 1.0 * last_mid_count / Mid_count;
		for (int i = 0; i < Mid_count; i++)
		{
			Mid_Line[i].row = (1 - k) * Last_M_Line[(uint8)(k2 * i)].row + k * Mid_Line[i].row;
			Mid_Line[i].col = (1 - k) * Last_M_Line[(uint8)(k2 * i)].col + k * Mid_Line[i].col;
		}
	}
	else if (last_mid_count < Mid_count)
	{
		k2 = 1.0 * Mid_count / last_mid_count;
		for (int i = 0; i < last_mid_count; i++)
		{
			Mid_Line[i].row = (1 - k) * Last_M_Line[i].row + k * Mid_Line[(uint8)(k2 * i)].row;
			Mid_Line[i].col = (1 - k) * Last_M_Line[i].col + k * Mid_Line[(uint8)(k2 * i)].col;
		}
	}
}
/*--------------------------------------------------------------------------
* ���������ܡ�����ʼ����߽��ұ߽�ṹ��
* ����    ��������
* ���� �� ֵ������
*--------------------------------------------------------------------------*/
void clear_point()
{
	for (int i = 0; i < L_edge_count; i++)
	{
		L_edge[i].row = 0;
		L_edge[i].col = 0;
		L_edge[i].flag = 0;
	}
	for (int i = 0; i < R_edge_count; i++)
	{
		R_edge[i].row = 0;
		R_edge[i].col = 0;
		R_edge[i].flag = 0;
	}
}
/*--------------------------------------------------------------------------
* ���������ܡ�����յ�ĽǶ�ֵ
* ����    ���������������������
* ���� �� ֵ�����Ƕ�ֵ
* ����    ע������
*--------------------------------------------------------------------------*/
//int Get_angle(uint8 ax, uint8 ay, uint8 bx, uint8 by, uint8 cx, uint8 cy)
//{
//	int8 abx = ax - bx;
//	int8 aby = ay - by;
//	int8 cbx = cx - bx;
//	int8 cby = cy - by;
//	int8 ab_muti_cb = abx * cbx + aby * cby;
//	int8 dist_ab = sqrt(abx * abx + aby * aby);
//	int8 dist_cb = sqrt(cbx * cbx + cby * cby);
//	int8 cosvalue = ab_muti_cb / (dist_ab * dist_cb);
//	return (int)(acos(cosvalue) * 180 / 3.14159);
//}
int Get_angle(uint8 ax, uint8 ay, uint8 bx, uint8 by, uint8 cx, uint8 cy)
{
	float abx = ax - bx;
	float aby = ay - by;
	float cbx = cx - bx;
	float cby = cy - by;
	float ab_muti_cb = abx * cbx + aby * cby;
	float dist_ab = sqrt(abx * abx + aby * aby);
	float dist_cb = sqrt(cbx * cbx + cby * cby);
	float cosvalue = ab_muti_cb / (dist_ab * dist_cb);
	return (int)(acos(cosvalue) * 180 / 3.14159);
}
/*--------------------------------------------------------------------------
* ���������ܡ����ж��Ƿ������/�ұ߽��
* ����    ���������߻�������ֵ      ��/�ұ߽�ѡ���־(0:��ߣ�1:�ұ�)
* ���� �� ֵ����-1 �� �߽����ֵ��-1��ʾû���ҵ��߽��
* ����    ע������
*--------------------------------------------------------------------------*/
int edge_point_ornot(uint8 row, uint8 side)  //5��155��Ҫ����
{
	//�����
	if (side == 0)
	{
		uint8 find_edge = 0;
		L_lost_count = 0;
		//�ӿ�ʼ��������������
		for (uint8 rowi = row; rowi > 0; rowi--)
		{
	    	//���ͼ�������Ϊ�����⣨δ���ߣ�����ʼ��������
			if (L_lost_count <= L_lost_)
			{
				//��������
				for (int col = L_edge_start_col; col < Image_W/2; col++)
				{
					if (black_(image[rowi][L_edge_start_col]))
					{
						//printf("col:%d ", col);
						//������ֺںڰװף����ж�Ϊ�߽��ߣ��˳�ѭ��
						if (black_(image[rowi][col]) && black_(image[rowi][col + 1]) && white_(image[rowi][col + 2]) && white_(image[rowi][col + 3]))
						{
							L_basic_row_start = rowi;   //��ֵ��ʼ�����У���
							printf("�������ʼ�㣺(x=%d,y=%d)\n", rowi, col + 1);
							if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
							return col + 1;               //������ֵ         //!!!   1->2
						}
					}
					if (col == Image_W / 2 - 1)
					{
						L_lost_count++;
					}
				}
			}
			//printf("lost %d ", L_lost_count);
			if (L_lost_count > L_lost_)//���ߴ��ڴ������ƿ���������
			{
				//������ֺںڰװף����ж�Ϊ�߽��ߣ��˳�ѭ��
				if (black_(image[rowi - 3][L_edge_start_col]) && black_(image[rowi - 2][L_edge_start_col]) && white_(image[rowi - 1][L_edge_start_col]) && white_(image[rowi][L_edge_start_col]))
				{
					L_basic_row_start = rowi - 2;   //��ֵ��ʼ�����У���
					printf("����������������ʼ�㣺(x=%d,y=%d)\n", L_basic_row_start, L_edge_start_col);
					L_start_lost = 1;
					//if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
					return L_edge_start_col;               //������ֵ         //!!!   1->2
				}
			}
			if (find_edge == 2) return -1;
		}
	}
	//�ұ���
	if (side == 1)
	{
		uint8 find_edge = 0;
		R_lost_count = 0;
		for (uint8 rowi = row; rowi > 0; rowi--)
		{
			//���ͼ�����Ҳ�Ϊ�����⣨δ���ߣ�����ʼ��������
			if (R_lost_count <= R_lost_)
			{
				//��������
				for (int col = R_edge_start_col; col > Image_W/2; col--)
				{
					if (black_(image[rowi][R_edge_start_col]))
					{
						//������ֺںڰװף����ж�Ϊ�߽��ߣ��˳�ѭ��
						if (white_(image[rowi][col - 3]) && white_(image[rowi][col - 2]) && black_(image[rowi][col - 1]) && black_(image[rowi][col]))
						{
							R_basic_row_start = rowi;
							printf("�ұ�����ʼ�㣺(x=%d,y=%d)\n", rowi, col + 1);
							if (rowi < Image_H / 2 && col < Image_W / 2) return -1;
							return col - 1;
						}
					}
					if (col == Image_W / 2 + 1)
					{
						R_lost_count++;
					}
				}
			}
			
			if (R_lost_count > R_lost_)//���ߴ��ڴ������ƿ���������
			{
				if (black_(image[rowi - 3][R_edge_start_col]) && black_(image[rowi - 2][R_edge_start_col]) && white_(image[rowi - 1][R_edge_start_col]) && white_(image[rowi][R_edge_start_col]))
				{
					R_basic_row_start = rowi - 2;   //��ֵ��ʼ�����У���
					printf("����������ұ�����ʼ�㣺(x=%d,y=%d)\n", R_basic_row_start, R_edge_start_col);
					R_start_lost = 1;
					//if (rowi<Image_H / 2 && col>Image_W / 2) return -1;
					return R_edge_start_col;               //������ֵ         //!!!   1->2
				}
			}
			if (find_edge == 2) return -1;
		}

	}
	return -1;
}
/*--------------------------------------------------------------------------
* ���������ܡ����ж��Ƿ��Ǻ�����/�����ص㣬Ϊ���ڲ����ö�ֵ��ͼ����׼��
* ����    �������õ������ֵ
* ���� �� ֵ������
* ����    ע������
*--------------------------------------------------------------------------*/
uint8 black_(uint8 x)    //�жϺ�
{
	if (binimage_flag == 0)  //���û�ж�ֵ������ͨ����ֵ�жϺڰ�
	{
		if (x < threshold)
			return 1;
		else if (x >= threshold)
			return 0;
	}
	else if (binimage_flag == 1)
	{
		if (x == 0)
			return 1;
		else if (x == 1)
			return 0;
	}
	return 0;
}

uint8 white_(uint8 x)    //�жϰ�
{
	if (binimage_flag == 0)
	{
		if (x < threshold)
			return 0;
		else if (x >= threshold)
			return 1;
	}
	else if (binimage_flag == 1)
	{
		if (x == 1)
			return 1;
		else if (x == 0)
			return 0;
	}
	return 0;
}
/*--------------------------------------------------------------------------
* ���������ܡ�����ȡ��ֵ��ͼ��
* ����    ��������
* ���� �� ֵ������
* ����    ע������
*--------------------------------------------------------------------------*/
void get_binImage(uint8 thres)
{
	binimage_flag = 1;  //��ֵ����־��Ϊ1
	for (int row = 0; row < Image_H; row++)
	{
		for (int col = 0; col < Image_W; col++)
		{
			if (image[row][col] < thres)
			{
				image[row][col] = 0;    //С����ֵ��ֵΪ0����ʾΪ��ɫ
			}
			else
				image[row][col] = 1;    //������ֵ��ֵΪ1����ʾΪ��ɫ
		}
	}
}
/*--------------------------------------------------------------------------
* ���������ܡ��������̬��ֵ
* ����    ��������
* ���� �� ֵ������
*--------------------------------------------------------------------------*/
#if 0
#define GrayScale 256   //frame

typedef unsigned char uchar;
int pixel[256] = { 0 };
uint8 OSTU_bin(uint8 width, uint8 height, uint8* Image)
{
	int threshold1 = 0;
	int32 sum_gray = 0;
	int32 sum_pix_num = 0;
	int32 pl_pix_num = 0;
	int32 p2_pix_mum = 0;
	int32 p1_sum_gray = 0;
	float m1 = 0;
	float m2 = 0;
	float V = 0;
	float variance = 0;
	int i, j, k = 0;

	for (i = 0; i < 256; i++)
		pixel[i] = 0;


	for (i = 0; i < height; i++)
	{
		for (j = 0; j < width; j++)
		{
			pixel[(int)Image[i * width + j]]++;
		}
	}

	for (k = 0; k < GrayScale; k++)
	{
		sum_gray += k * pixel[k];
		sum_pix_num += pixel[k];
	}

	for (k = 0; k < GrayScale - 1; k++)
	{
		pl_pix_num += pixel[k];
		p2_pix_mum = sum_pix_num - pl_pix_num;
		p1_sum_gray += k * pixel[k];
		m1 = (float)p1_sum_gray / pl_pix_num;
		m2 = (float)(sum_gray - p1_sum_gray) / p2_pix_mum;

		V = pl_pix_num * p2_pix_mum * (m1 - m2) * (m1 - m2);

		if (V > variance)
		{
			variance = V;
			threshold1 = k;
		}
	}
	return threshold1;
}

#endif
/*--------------------------------------------------------------------------
* ���������ܡ��������̬��ֵ
* ����    ��������
* ���� �� ֵ������
*--------------------------------------------------------------------------*/
#if 1
uint8 GetOSTUThreshold(uint8(*img)[MT9V03X_W], uint16 start_row, uint16 end_row, uint16 start_col, uint16 end_col)
{
	int     threshold1 = 0;
	int32 sum_gray = 0;
	int32 sum_pix_num = 0;
	int32 pl_pix_num = 0;
	int32 p2_pix_mum = 0;
	int32 p1_sum_gray = 0;
	float m1 = 0;
	float m2 = 0;
	float V = 0;
	float variance = 0;
	int i, j, k = 0;
	uint16 MinValue = 0, MaxValue = 255;
	uint16 DeleGrayClass1 = 30; //�߻Ҷȼ�
	uint16  HistoGram[256] = { 0 };

	for (i = 0; i < 256; i++)
		HistoGram[i] = 0;

	for (i = start_row; i < end_row; i++)
	{
		for (j = start_col; j < end_col; j++)
		{
			HistoGram[(int)img[i][j]]++;
		}
	}

	//�Ż�--ɾ���Ҷȼ�����<x����ĻҶȼ�  ɾ���Ҷȼ��ײ�<x����ĻҶȼ� x==> 10-25֮��
	//for(k=255;k>0;--k) {if(HistoGram[k]<=DeleGrayClass1)  HistoGram[k] = 0; else break;}
	//for(k=0;k<256;++k) {if(HistoGram[k]<=DeleGrayClass2)  HistoGram[k] = 0; else break;}

	for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; ++MinValue);        //��ȡ��С�Ҷȼ�
	for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; --MaxValue); //��ȡ���Ҷȼ�

	for (k = MinValue; k <= MaxValue; k++)
	{
		sum_gray += k * HistoGram[k];
		sum_pix_num += HistoGram[k];
	}

	for (k = MinValue; k <= MaxValue; k++)
	{
		pl_pix_num += HistoGram[k];
		p2_pix_mum = sum_pix_num - pl_pix_num;
		p1_sum_gray += k * HistoGram[k];
		m1 = (float)p1_sum_gray / pl_pix_num;
		m2 = (float)(sum_gray - p1_sum_gray) / p2_pix_mum;

		V = pl_pix_num * p2_pix_mum * (m1 - m2) * (m1 - m2);

		if (V > variance)
		{
			variance = V;
			threshold1 = k;
		}
	}

#if 1
	uint8 t1 = threshold1 / 2;
	uint8 t2 = threshold1 + 10;  //���췶Χ
	float rate = 1.25;          //�����������
	//uint8 top_y   = 15;            //�������� 0-top_y
	uint8 side_x = 20;            //������� 0-side_x && (COL_1-side_x)-COL


	/* ��ʼ�� */
	for (i = 255; i > 0; i--)
		HistoGram[i] = i;

	/* �ݶȱ任--�Աȶ����� */
	for (i = t1; i < t2; i++)
	{
		HistoGram[i] = (uint16)(i * rate);
		if (HistoGram[i] > t2) HistoGram[i] = t2;
	}

	//���
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = 0; j < side_x; j++)
		{
			mt9v03x_image[i][j] = HistoGram[mt9v03x_image[i][j]];
		}
	}

	//�Ҳ�
	for (i = 0; i < MT9V03X_H; i++)
	{
		for (j = MT9V03X_W - 1 - side_x; j < MT9V03X_W; j++)
		{
			mt9v03x_image[i][j] = HistoGram[mt9v03x_image[i][j]];
		}
	}
#endif
	return threshold1;
}
#endif
/*--------------------------------------------------------------------------
* ���������ܡ�����ֵ�˲�
* ����    ��������
* ���� �� ֵ������
*--------------------------------------------------------------------------*/
int average_value(uint8* c)
{
	int i;
	int k;
	uint16 t_sum;
	t_sum = 0;
	for (i = 0; i <= 8; i++)
	{
		t_sum = t_sum + c[i];
	}
	k = t_sum / 9;
	return k;
}

void average_filter(void)
{
	uint8 i, j;
	uint8  c[9];
	for (i = 1; i < Image_H - 1; i++)
	{
		for (j = 1; j < Image_W - 1; j++)
		{
			if (i >= 8)
			{
				c[0] = image[i - 1][j - 1];
				c[1] = image[i - 1][j];
				c[2] = image[i - 1][j + 1];
				c[3] = image[i][j - 1];
				c[4] = image[i][j];
				c[5] = image[i][j + 1];
				c[6] = image[i + 1][j - 1];
				c[7] = image[i + 1][j];
				c[8] = image[i + 1][j + 1];
				image[i][j] = average_value(c);
			}

		}
	}

}

void get_deal_image()
{
	for (int row = 0; row < Image_H; row++)
		for (int col = 0; col < Image_W; col++)
		{
			image[row][col] = mt9v03x_image[row][col];
		}
}


// �㼯�����˲�
void blur_points(struct LEFT_EDGE pts_in[120], struct LEFT_EDGE pts_out[120], int num, int kernel)
{
	int half = kernel / 2;
	for (int i = 0; i < num; i++) {
		pts_out[i].row = pts_out[i].col = 0;
		for (int j = -half; j <= half; j++) {
			pts_out[i].row += pts_in[clip(i + j, 0, num - 1)].row * (half + 1 - abs(j));
			pts_out[i].col += pts_in[clip(i + j, 0, num - 1)].col * (half + 1 - abs(j));
		}
		pts_out[i].row /= (2 * half + 2) * (half + 1) / 2;
		pts_out[i].col /= (2 * half + 2) * (half + 1) / 2;
		//printf("row:%d col:%d\n", pts_out[i].row);
	}
}

int clip(int x, int low, int up) {
	return x > up ? up : x < low ? low : x;
}

float fclip(float x, float low, float up) {
	return x > up ? up : x < low ? low : x;
}

void edge_start()
{
	for (int i = 0; i < 150; i++)
	{
		L_edge[i].row = 0;
		L_edge[i].col = 0;
		R_edge[i].row = 0;
		R_edge[i].col = 0;
	}
}

//���߽ضϴ���
void edge_truncation(uint8 side)
{
	if (side == 0)
	{
		//��ʼ��������£�����ʽ
		if (L_start_lost)
		{
			FitStraightLine(L_top_i, L_top_i + 10, 0);
			printf("��ʼ����:k=%f,b=%f\n", k_l, b_l);
		}
		else if (center_lost_flag_l) //�м䶪������£�����ʽ
		{
			float k = 0, b = 0;
			FitStraightLine(L_top_i, L_top_i + 10, 0);
			k = k_l;
			b = b_l;

			k_l = (L_corner_col - center_lost_corner_col_l) / (L_corner_row - center_lost_corner_row_l);
			b_l = L_corner_col - k_l * L_corner_row;
			k_l = (k_l + k) / 2.0;
			b_l = (b_l + b) / 2.0;
			//FitStraightLine(L_beh_i - 9, L_beh_i+1, 0);
			//k_l = (k_l + k) / 2.0;
			//b_l = (b_l + b) / 2.0;
			printf("���м䶪��:k=%f,b=%f\n", k_l, b_l);
		}
		else //���δ�������
		{
			FitStraightLine(L_beh_i - 9, L_beh_i+1, 0);
			printf("���δ����:k=%f,b=%f\n", k_l, b_l);
		}
		//if (L_edge_count >= 70)
		//{
		//	num_cnt = 0;//��¼����ˮƽ��ĸ���
		//	L_count = L_edge_count / 2;
		//	while (L_count < L_edge_count)
		//	{
		//		if (L_edge[L_count].row == L_edge[L_count + 1].row)
		//			num_cnt = num_cnt + 1;
		//		else
		//			num_cnt = 0;
		//		if (num_cnt > 5)//����5����ˮƽ
		//			break;
		//		L_count = L_count + 1;
		//	}
		//	L_edge_count = L_count;//�ض���5��ˮƽ�㴦
		//}
	}
	if (side == 1)
	{
		//��ʼ��������£�����ʽ
		if (R_start_lost)
		{
			FitStraightLine(R_top_i, R_top_i + 10, 1);
			printf("�ҿ�ʼ����:k=%f,b=%f\n", k_r, b_r);
		}
		else if (center_lost_flag_r) //�м䶪������£�����ʽ
		{
			float k = 0, b = 0;
			FitStraightLine(R_top_i, R_top_i + 10, 1);
			k = k_r;
			b = b_r;

			k_r = (R_corner_col - center_lost_corner_col_r) / (R_corner_row - center_lost_corner_row_r);
			b_r = R_corner_col - k_r * R_corner_row;
			k_r = (k_r + k) / 2.0;
			b_r = (b_r + b) / 2.0;
			//FitStraightLine(R_beh_i - 9, R_beh_i+1, 1);
			//k_r = (k_r + k) / 2.0;
			//b_r = (b_r + b) / 2.0;
			printf("���м䶪��:k=%f,b=%f\n", k_r, b_r);
		}
		else //���δ�������
		{
			FitStraightLine(R_beh_i - 9, R_beh_i+1, 1);
			printf("�ұ�δ����:k=%f,b=%f\n", k_r, b_r);
		}
		//if (L_edge_count >= 70)
		//{
		//	num_cnt = 0;//��¼����ˮƽ��ĸ���
		//	L_count = L_edge_count / 2;
		//	while (L_count < L_edge_count)
		//	{
		//		if (L_edge[L_count].row == L_edge[L_count + 1].row)
		//			num_cnt = num_cnt + 1;
		//		else
		//			num_cnt = 0;
		//		if (num_cnt > 5)//����5����ˮƽ
		//			break;
		//		L_count = L_count + 1;
		//	}
		//	L_edge_count = L_count;//�ض���5��ˮƽ�㴦
		//}
	}
	
}

//��С�������
void FitStraightLine(int start, int end, uint8 side)
{
	int n = end - start;
	int x_add = 0;
	int y_add = 0;
	int xy_add = 0;
	int xx_add = 0;
	float x_bar = 0.0;
	float y_bar = 0.0;
	float xy_bar = 0.0;
	float xx_bar = 0.0;
	float b_add = 0.0;

	if (side == 0)
	{
		for (int i = start; i < end; i++)
		{
			printf("(%d,%d)\n", L_edge[i].row, L_edge[i].col);
			x_add += L_edge[i].row;
			y_add += L_edge[i].col;
			xx_add += (L_edge[i].row * L_edge[i].row);
			xy_add += (L_edge[i].row * L_edge[i].col);
		}

		x_bar = x_add * 1.0 / n;
		y_bar = y_add * 1.0 / n;
		xx_bar = xx_add * 1.0 / n;
		xy_bar = xy_add * 1.0 / n;
		//printf("kf=%f", (xy_bar - x_bar * y_bar) / (xx_bar - x_bar * x_bar));

		k_l = (xy_bar - x_bar * y_bar) / (xx_bar - x_bar * x_bar); //���ֱ��б��
		b_l = y_bar - k_l * x_bar;//���ֱ�ߵĳ�����
	}
	if (side == 1)
	{
		for (int i = start; i < end; i++)
		{
			//printf("(%d,%d)\n", R_edge[i].row, R_edge[i].col);
			x_add += R_edge[i].row;
			y_add += R_edge[i].col;
			xx_add += (R_edge[i].row * R_edge[i].row);
			xy_add += (R_edge[i].row * R_edge[i].col);
		}

		x_bar = x_add * 1.0 / n;
		y_bar = y_add * 1.0 / n;
		xx_bar = xx_add * 1.0 / n;
		xy_bar = xy_add * 1.0 / n;

		k_r = (xy_bar - x_bar * y_bar) / (xx_bar - x_bar * x_bar); //���ֱ��б��
		b_r = y_bar - k_r * x_bar;//���ֱ�ߵĳ�����
	}
}


// �����Թ�Ѳ��  //y=115,x=30
void findline_lefthand_adaptive(int jilu_row, int jilu_col, int search_amount) 
{
	L_edge[0].row = jilu_row;
	L_edge[0].col = jilu_col;
	L_edge[0].flag = 1;
	uint8 curr_row = jilu_row;//��ʼ��������
	uint8 curr_col = jilu_col;//��ʼ��������
	dire_left = 0; //��ʼ���ϸ��߽�������
	center_turn_flag = 1;//��ʼ����δ�궨״̬
	center_lost_flag_l = 0;//�м����߱�־λ
	//��ʼ���ߣ����ȡ150���㣬���������ѣ���7����λ
	for (int i = 1; i < search_amount; i++)    //�������70����
	{
		////Խ���˳� ��Խ�����Խ�磨���������������ң�
		if (curr_row < L_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < L_edge_end_row)  break;
		if (curr_col > max_col || curr_col < min_col && !center_lost_flag_l)   //max=160-5 min=5
		{
			if (++L_search_edge_count == 3)//����3���������߽磬�˳�      //�����������߽������Ҫ����  5��155��Ҫ����
			{
				curr_col = L_edge_lost_start_col;

				for (uint8 rowi = curr_row; rowi > 0; rowi--)
				{
					//printf("row=%d", rowi);
					if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
					{
						printf("�м�����������������ʼ�㣺(x=%d,y=%d)\n", rowi - 2, L_edge_lost_start_col);
						curr_row = rowi - 2;
						curr_col = L_edge_lost_start_col;
						center_lost_flag_l = 1;
						center_lost_row_l = curr_row;     //�м����߿�ʼ������
						center_lost_col_l = curr_col + 1; //�м����߿�ʼ������
						dire_left = 0; //��ʼ���ϸ��߽�������
						L_top_corner_start = i;//���Ϲյ㿪ʼ���
						break;
					}
				}
			}
		}
		else
			L_search_edge_count = 0;
		//���߹���
		if (black_(image[curr_row + dir_front[dire_left][0]][curr_col + dir_front[dire_left][1]]))
		{
			dire_left = (dire_left + 1) % 4;
			i = i - 1;
			//turn++;
		}
		else if (black_(image[curr_row + dir_frontleft[dire_left][0]][curr_col + dir_frontleft[dire_left][1]]))
		{
			curr_row += dir_front[dire_left][0];
			curr_col += dir_front[dire_left][1];
			L_edge[i].row = curr_row;
			L_edge[i].col = curr_col;
			L_edge[i].flag = 1;
			L_edge_count = L_edge_count + 1;
		}
		else
		{
			curr_row += dir_frontleft[dire_left][0];
			curr_col += dir_frontleft[dire_left][1];
			dire_left = (dire_left + 3) % 4;
			L_edge[i].row = curr_row;
			L_edge[i].col = curr_col;
			L_edge[i].flag = 1;
			L_edge_count = L_edge_count + 1;
		}
		//printf("(x1=%d,y1=%d)\n", L_edge[i].row, L_edge[i].col);
		//dd = dd + 1;
	}
	//printf("count%d ", L_edge_count);
}

// �����Թ�Ѳ��  //y=115,x=30
void findline_righthand_adaptive(int jilu_row, int jilu_col, int search_amount)
{
	R_edge[0].row = jilu_row;
	R_edge[0].col = jilu_col;
	R_edge[0].flag = 1;
	uint8 curr_row = jilu_row;//��ʼ��������
	uint8 curr_col = jilu_col;//��ʼ��������
	dire_right = 0; //��ʼ���ϸ��߽�������
	center_turn_flag = 1;//��ʼ����δ�궨״̬
	center_lost_flag_r = 0;//�м����߱�־λ
	//��ʼ���ߣ����ȡ150���㣬���������ѣ���7����λ
	for (int i = 1; i < search_amount; i++)    //�������70����
	{
		////Խ���˳� ��Խ�����Խ�磨���������������ң�
		if (curr_row < R_edge_end_row || curr_row>Image_H - 1 || curr_row + 1 < R_edge_end_row)  break;
		if (curr_col > max_col || curr_col < min_col && !center_lost_flag_r)   //max=160-5 min=5
		{
			if (++R_search_edge_count == 3)//����3���������߽磬�˳�      //�����������߽������Ҫ����  5��155��Ҫ����
			{
				curr_col = R_edge_lost_start_col;

				for (uint8 rowi = curr_row; rowi > 0; rowi--)
				{
					//printf("row=%d", rowi);
					if (black_(image[rowi - 3][curr_col]) && black_(image[rowi - 2][curr_col]) && white_(image[rowi - 1][curr_col]) && white_(image[rowi][curr_col]))
					{
						printf("�м��Ҷ���������ұ�����ʼ�㣺(x=%d,y=%d)\n", rowi - 2, R_edge_lost_start_col);
						curr_row = rowi - 2;
						curr_col = R_edge_lost_start_col;
						center_lost_flag_r = 1;
						center_lost_row_r = curr_row;     //�м����߿�ʼ������
						center_lost_col_r = curr_col + 1; //�м����߿�ʼ������
						dire_right = 0; //��ʼ���ϸ��߽�������
						R_top_corner_start = i;//���Ϲյ㿪ʼ���
						break;
					}
				}
			}
		}
		else
			R_search_edge_count = 0;
		//���߹���
		if (black_(image[curr_row + dir_front[dire_right][0]][curr_col + dir_front[dire_right][1]]))
		{
			dire_right = (dire_right + 3) % 4;
			i = i - 1;
			//turn++;
		}
		else if (black_(image[curr_row + dir_frontright[dire_right][0]][curr_col + dir_frontright[dire_right][1]]))
		{
			curr_row += dir_front[dire_right][0];
			curr_col += dir_front[dire_right][1];
			R_edge[i].row = curr_row;
			R_edge[i].col = curr_col;
			R_edge[i].flag = 1;
			R_edge_count = R_edge_count + 1;
		}
		else
		{
			curr_row += dir_frontright[dire_right][0];
			curr_col += dir_frontright[dire_right][1];
			dire_right = (dire_right + 1) % 4;
			R_edge[i].row = curr_row;
			R_edge[i].col = curr_col;
			R_edge[i].flag = 1;
			R_edge_count = R_edge_count + 1;
		}
		//printf("(x1=%d,y1=%d)\n", L_edge[i].row, L_edge[i].col);
		//dd = dd + 1;
	}
	//printf("count%d ", L_edge_count);
}



/**  @brief    ������  */
#define ROAD_MAIN_ROW      40

/**  @brief    ʹ����ʼ��120  */
#define ROAD_START_ROW     119

/**  @brief    ʹ�ý�����10  */
#define ROAD_END_ROW       1
//
//#define turn_big           40  //����������ֵ���40�ж�Ϊǰ���д���
//
//
///*******************�����ǻ���ʶ�����õ������ȫ�ֱ��� �ȷ�����***************/
///********************�����ǻ���ʶ�����õ�����ر����ȷ�����*********************/
//uint8 Island_flag = 0;
//uint8 Big_island_flag = 0;
//int16 Fir_jump_point_row;
//int16 Fir_jump_point_column;
//int16 Sec_jump_point_row;
//int16 Sec_jump_point_column;
//
//int16 Island_change_time = 400;//����״̬�л���ʱ 400ms �ٶȴ�ʱ��С �ٶ�Сʱ�Ӵ�
//
//int16 Island_time;
//int16 meet_time;
//
///**********************�����ǻᳵ��ʶ�����õ�����ر���************************/
//uint8 meet_flag = 0;
//uint8 Left_car_flag_mid;
//int16 Start_time;
//uint8 Left_car_flag_end;
//int16 Stop_ready2;
//
//
//uint8 tesssss[ROW][COL];
//
///********************���µ�ȫ�ֱ��������ڿ��Ż�Ϊ�ֲ�����*********************/
//
//
//
///*************************����Ϊ����Ѳ�������ȫ�ֱ���*************************/
//coordinate Left_line; //����������� ö�� �������������
//coordinate Right_line;//�ұ��������� ö�� ���ұ���������
//coordinate Midd_line; //����������   ö�� ������������
//struct PID Steer;// PID PDö�� ���������
//uint8 binary_img[ROW][COL];//��ֵ�����ͼ������
//uint8 image[ROW][COL];
//uint8 Left_line_lost[70]; //�洢��Ӧ������ߵ� ���������1 Ϊ����δ�ҵ�                                                //40
//uint8 Right_line_lost[70];//�洢��Ӧ���ұ��ߵ� ���������1 Ϊ����δ�ҵ�                                                //40
//uint8 Found_left_flag, Found_right_flag;//�������ұ��ߺڵ�Ѱ�ұ�־λ 1Ϊ�ҵ�
//uint8 Near_flag;//���������־ С��ǰ�˳�ͷ�����ٽ������߽� ����ȫ����(����һƬ��)
//uint8 Cross_road_flag;//ʮ�� ��־λ
//uint8 out_flag;//�����־λ
//uint8 Highest_speed_flag, Mult_speed_flag, Lowest_speed_flag;//С������ ���� ��־λ
//uint8 Ram_flag = 0;//�µ���־λ
//uint8 Thre = 130;
//int16 L_lost_cnt = 0, R_lost_cnt = 0;//���Ҷ������ߣ��㣩����  
//int16 L_R_lost_cnt, Cross_road_cnt;//����ͬʱ�����ߣ��㣩����  ʮ�ֶ��ߴ������б�˫�߶��ߴ���
//int16 Row_begging = 63, Row_end = 0, Last_row_end, Column_end;//ɨ�� ��ʼ�� ������  ��һ��ɨ�߽����� �����У���ʼ�У�   //63
//int16 L_last_lost_row = 63, R_last_lost_row = 63;//���ұ���󶪵��ߵ�����                                           //63
//int16 L_R_lost_row = 63;//ʮ������                                                                                   //63
//int16 Last_mid_line = 93;//���� �е������
//int16 Vertical_longest_length_1;//�����������ȣ���Զ�ˣ�
//int16 old_error;//�ϴ�����ƫ��
//int16 row = 63;//����                                                                                               //63
//int16 Row_start = 63;//ɨ����ʼ��;                                                                                   //63
//int16 Last_L_column;//��һ��ͼ��ĳһ�е���߽����� ��ʱδ�õ�
//int16 Last_R_column;//��һ��ͼ��ĳһ�е��ұ߽����� ��ʱδ�õ�
//int16 Zebra_first_row;
//
//float Middle_line = 93;//ͼ������  �ɸĶ�
//float Left_slope = 0; //�����  ����б��
//float Right_slope = 0;//�ұ��� ����б��
//float Last_left_slope = 0; //��һ��ͼ��������б��
//float Last_right_slope = 0;//��һ��ͼ����ұ���б��
//
//float Add_weight[60] = {
//							2.3,2.3,2.3,2.3,2.3,2,2,2,2,2,
//							2,2,1.7,1.7,1.7,1.7,1.5,1,1,1,
//							1,1,1,1,1,1,1,1,1,1,
//
//							1,1,1,1,1,1,1,1,1,1,//�����ʮ��  �����õ���������ʮ��
//							1,1,1,1,1,1,1,1,1,1,
//
//							1,1,1,1,1,1,1,1,1,1,
//
//};//��Ȩ�㷨 Ȩ�� ��·��  ���ڿ��÷ֶμ�Ȩ 
//float Single_L_line[35] = {
//								78,76,75,73,71,69,68,66,64,62,
//								60,58,57,55,53,52,50,49,47,46,
//								44,42,41,39,38,37,35,63,61,31,
//								29,28,27,25,24,
//};
///*float Single_R_line[35] = {
//								90,92,94,96,99,100,102,104,105,107,
//								109,110,112,114,115,117,118,120,122,123,
//								125,126,128,129,130,131,133,135,136,137,
//								139,140,141,143,144,
//						   };*/
//float Single_R_line[35] = {
//								91,92,94,95,96,97,99,101,103,105,
//								107,110,111,112,114,116,117,118,120,122,
//								123,125,126,128,129,131,132,163,135,137,
//								138,140,141,142,144,
//};
//
//
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

//int add(int num1, int num2);
//
//void main()
//{
//	int a = 2;
//	int c = 0;
//	c = add(a, c);
//	printf("%d", c);
//}
//
///**
// * @brief ���������
// */
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
//
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
//
//
//void hhh(uint8_t image[MT9V03X_H][MT9V03X_W],uint8_t img[MT9V03X_H][MT9V03X_W], uint8_t th)
//{
//	for (int i = 0; i < MT9V03X_H; i++)
//	{
//		for (int j = 0; j < MT9V03X_W; j++)
//		{
//			img[i][j] = image[i][j];
//		}
//	}
//	for (int i = 0; i < MT9V03X_H; i++)
//	{
//		for (int j = 0; j < MT9V03X_W; j++)
//		{
//			if(img[i][j]>th)
//				img[i][j] = 255;
//			else
//				img[i][j] = 0;
//		}
//	}
//}
//
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



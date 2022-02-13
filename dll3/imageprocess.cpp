#include "stdafx.h"
#include <stdio.h>
#include "test.h"
#include "common.h"
#include "sensor_process.h"

#include "main.h"
#include "cross.h"
#include "yroad.h"
#include "circle.h"
#include "garage.h"
#include "apriltag.h"

#include "imgproc.h"
#include "camera_param.h"
#include "flash_param.h"
#include "utils.h"

#define _USE_MATH_DEFINES
#include <math.h>

/*!
  * @file     imageprocess.cpp
  * @brief    图像处理
  * @author   YYY
  * @version  V1.2
  * @date     2022/2/7
  */
uint8_t img_raw_data[MT9V03X_CSI_H][MT9V03X_CSI_W];
image_t img_raw = DEF_IMAGE((uint8_t*)img_raw_data, MT9V03X_CSI_W, MT9V03X_CSI_H);

uint8_t img_thres_data[MT9V03X_CSI_H][MT9V03X_CSI_W];
image_t img_thres = DEF_IMAGE((uint8_t*)img_thres_data, MT9V03X_CSI_W, MT9V03X_CSI_H);
uint8_t img_line_data[MT9V03X_CSI_H][MT9V03X_CSI_W];
image_t img_line = DEF_IMAGE((uint8_t*)img_line_data, MT9V03X_CSI_W, MT9V03X_CSI_H);
  // 原图左右边线
int ipts0[POINTS_MAX_LEN][2];
int ipts1[POINTS_MAX_LEN][2];
int ipts0_num, ipts1_num;
// 变换后左右边线
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
int rpts0_num, rpts1_num;
// 变换后左右边线+滤波
float rpts0b[POINTS_MAX_LEN][2];
float rpts1b[POINTS_MAX_LEN][2];
int rpts0b_num, rpts1b_num;
// 变换后左右边线+等距采样
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
int rpts0s_num, rpts1s_num;
// 左右边线局部角度变化率
float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
int rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
int rpts0an_num, rpts1an_num;
// 左/右中线
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
int rptsc0_num, rptsc1_num;
// 中线
float(*rpts)[2];
int rpts_num;
// 归一化中线
float rptsn[POINTS_MAX_LEN][2];
int rptsn_num;

// Y角点
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;

// L角点
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;

// 长直道
bool is_straight0, is_straight1;

// 弯道
bool is_turn0, is_turn1;

// 当前巡线模式
enum track_type_e track_type = TRACK_RIGHT;


////串口打印标志位,调试用
//void print_all() {
//	static char buffer[512];
//	int len = 0;
//
//	// Flags
//	len += snprintf(buffer + len, sizeof(buffer) - len, "%s\t", apriltag_type_name[apriltag_type]);
//	len += snprintf(buffer + len, sizeof(buffer) - len, "%s\t", yroad_type_name[yroad_type]);
//	len += snprintf(buffer + len, sizeof(buffer) - len, "%s\t", cross_type_name[cross_type]);
//	len += snprintf(buffer + len, sizeof(buffer) - len, "%s\t", circle_type_name[circle_type]);
//	len += snprintf(buffer + len, sizeof(buffer) - len, "%s\t", garage_type_name[garage_type]);
//
//	// Line
//	len += snprintf(buffer + len, sizeof(buffer) - len, "%d\t%d\t", rpts0s_num, rpts1s_num);
//	len += snprintf(buffer + len, sizeof(buffer) - len, "%d\t%d\t", Lpt0_found * Lpt0_rpts0s_id,
//		Lpt1_found * Lpt1_rpts1s_id);
//
//
//	// Control
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%f\t%f\t", motor_l.target_speed, motor_r.target_speed);
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%f\t%f\t", motor_l.encoder_speed, motor_r.encoder_speed);
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%d\t%d\t", (int32_t)motor_l.total_encoder, (int32_t)motor_r.total_encoder);
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%d\t%d\t", motor_l.duty, motor_r.duty);
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%f\t%f\t", angle, eulerAngle.yaw);
//
//	// FIX CIRCLE
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%d\t%d\t", Lpt0_found*Lpt0_rpts0s_id, Lpt1_found*Lpt1_rpts1s_id);
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%d\t%d\t", rpts0s_num, rpts1s_num);
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%d\t%d\t", is_straight0, is_straight1);
////    len += snprintf(buffer+len, sizeof(buffer)-len, "%f\t%f\t", rpts0s[0][0], rpts1s[0][0]);
//
//	// Time
//	//len += snprintf(buffer + len, sizeof(buffer) - len, "%d\n", rt_tick_get_millisecond());
//	//seekfree_wireless_send_buff((uint8_t*)buffer, len);
//}

//int main(void) {
//	camera_sem = rt_sem_create("camera", 0, RT_IPC_FLAG_FIFO);
//	debugger_init();
//	mt9v03x_csi_init();
//	icm20602_init_spi();
//	//陀螺仪零漂矫正
//	gyroOffset_init();
//	encoder_init();
//	buzzer_init();
//	//    button_init();
//	motor_init();
//	elec_init();
//	display_init();
//	openart_mini();
//	smotor_init();
//	laser_init();
//	timer_pit_init();
//	seekfree_wireless_init();
//
//	// 加载flash参数
//	flash_param_init();
//	if (flash_param_check()) {
//		flash_param_load();
//	}
//
//	// 初始化调试GPIO
//	gpio_init(DEBUGGER_PIN, GPI, 0, GPIO_PIN_CONFIG);
//	gpio_init(C30, GPI, 0, GPIO_PIN_CONFIG);
//	if (gpio_get(C30) == 0) garage_type = GARAGE_OUT_LEFT;
//	else garage_type = GARAGE_OUT_RIGHT;
//
//	// 上位机注册
//	debugger_register_image(&img0);
//	debugger_register_image(&img1);
//	debugger_register_image(&img2);
//	debugger_register_param(&p0);
//	debugger_register_param(&p1);
//	debugger_register_param(&p2);
//	debugger_register_param(&p3);
//	debugger_register_param(&p4);
//	debugger_register_param(&p5);
//	debugger_register_param(&p6);
//	debugger_register_param(&p7);
//	debugger_register_param(&p8);
//	debugger_register_param(&p9);
//	debugger_register_param(&p10);
//	debugger_register_option(&opt0);
//	debugger_register_option(&opt1);
//	debugger_register_option(&opt2);
//	debugger_register_option(&opt3);
//
//	EnableGlobalIRQ(0);
//
//	while (1) {
//		// 等待摄像头采集完毕
//		rt_sem_take(camera_sem, RT_WAITING_FOREVER);
//		img_raw.data = mt9v03x_csi_image[0];
//		img0.buffer = mt9v03x_csi_image[0];
//
//		// main-loop计时
//		uint32_t t1 = rt_tick_get_millisecond();
//		// 开始处理摄像头图像
//		process_image();    // 边线提取&处理
//		find_corners();     // 角点提取&筛选
//
//		// 预瞄距离,动态效果更佳
//		aim_distance = 0.58;
//
//		// 单侧线少，切换巡线方向  切外向圆
//		if (rpts0s_num < rpts1s_num / 2 && rpts0s_num < 60) {
//			track_type = TRACK_RIGHT;
//		}
//		else if (rpts1s_num < rpts0s_num / 2 && rpts1s_num < 60) {
//			track_type = TRACK_LEFT;
//		}
//		else if (rpts0s_num < 20 && rpts1s_num > rpts0s_num) {
//			track_type = TRACK_RIGHT;
//		}
//		else if (rpts1s_num < 20 && rpts0s_num > rpts1s_num) {
//			track_type = TRACK_LEFT;
//		}
//
//		// 车库斑马线检查(斑马线优先级高，最先检查)
//		check_garage();
//
//		// 总钻风检查Apriltag(找赛道上的黑斑)
//		if (!enable_adc && garage_type == GARAGE_NONE && get_total_encoder() - openart.aprilencoder > ENCODER_PER_METER)
//			check_apriltag();
//
//		// 分别检查十字 三叉 和圆环, 十字优先级最高
//		if (garage_type == GARAGE_NONE &&
//			apriltag_type != APRILTAG_FOUND && apriltag_type != APRILTAG_LEAVE)
//			check_cross();
//		if (garage_type == GARAGE_NONE && cross_type == CROSS_NONE && circle_type == CIRCLE_NONE &&
//			apriltag_type != APRILTAG_FOUND && apriltag_type != APRILTAG_LEAVE)
//			check_yroad();
//		if (garage_type == GARAGE_NONE && cross_type == CROSS_NONE && yroad_type == YROAD_NONE &&
//			apriltag_type != APRILTAG_FOUND && apriltag_type != APRILTAG_LEAVE)
//			check_circle();
//		if (cross_type != CROSS_NONE) {
//			circle_type = CIRCLE_NONE;
//			yroad_type = YROAD_NONE;
//		}
//
//		//车库 ,十字清Aprltag标志
//		if (garage_type != GARAGE_NONE || cross_type != CROSS_NONE) apriltag_type = APRILTAG_NONE;
//
//		//根据检查结果执行模式
//		if (yroad_type != YROAD_NONE) run_yroad();
//		if (cross_type != CROSS_NONE) run_cross();
//		if (circle_type != CIRCLE_NONE) run_circle();
//		if (garage_type != GARAGE_NONE) run_garage();
//
//		//检查Openart收发
//		check_openart();
//
//		// 中线跟踪
//		if (cross_type != CROSS_IN) {
//			if (track_type == TRACK_LEFT) {
//				rpts = rptsc0;
//				rpts_num = rptsc0_num;
//			}
//			else {
//				rpts = rptsc1;
//				rpts_num = rptsc1_num;
//			}
//		}
//		else {
//			//十字根据远线控制
//			if (track_type == TRACK_LEFT) {
//				track_leftline(far_rpts0s + far_Lpt0_rpts0s_id, far_rpts0s_num - far_Lpt0_rpts0s_id, rpts,
//					(int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
//				rpts_num = far_rpts0s_num - far_Lpt0_rpts0s_id;
//			}
//			else {
//				track_rightline(far_rpts1s + far_Lpt1_rpts1s_id, far_rpts1s_num - far_Lpt1_rpts1s_id, rpts,
//					(int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
//				rpts_num = far_rpts1s_num - far_Lpt1_rpts1s_id;
//			}
//		}
//
//		// 车轮对应点(纯跟踪起始点)
//		float cx = mapx[MT9V03X_CSI_W / 2][(int)(MT9V03X_CSI_H * 0.78f)];
//		float cy = mapy[MT9V03X_CSI_W / 2][(int)(MT9V03X_CSI_H * 0.78f)];
//
//		// 找最近点(起始点中线归一化)
//		float min_dist = 1e10;
//		int begin_id = -1;
//		for (int i = 0; i < rpts_num; i++) {
//			float dx = rpts[i][0] - cx;
//			float dy = rpts[i][1] - cy;
//			float dist = sqrt(dx * dx + dy * dy);
//			if (dist < min_dist) {
//				min_dist = dist;
//				begin_id = i;
//			}
//		}
//
//		// 特殊模式下，不找最近点(由于边线会绕一圈回来，导致最近点为边线最后一个点，从而中线无法正常生成)
//		if (garage_type == GARAGE_IN_LEFT || garage_type == GARAGE_IN_RIGHT || cross_type == CROSS_IN) begin_id = 0;
//
//		// 中线有点，同时最近点不是最后几个点
//		if (begin_id >= 0 && rpts_num - begin_id >= 3) {
//			// 归一化中线
//			rpts[begin_id][0] = cx;
//			rpts[begin_id][1] = cy;
//			rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
//			resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
//
//			// 远预锚点位置
//			int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
//			// 近预锚点位置
//			int aim_idx_near = clip(round(0.25 / sample_dist), 0, rptsn_num - 1);
//
//			// 计算远锚点偏差值
//			float dx = rptsn[aim_idx][0] - cx;
//			float dy = cy - rptsn[aim_idx][1] + 0.2 * pixel_per_meter;
//			float dn = sqrt(dx * dx + dy * dy);
//			float error = -atan2f(dx, dy) * 180 / M_PI;
//			assert(!isnan(error));
//
//			// 若考虑近点远点,可近似构造Stanley算法,避免撞路肩
//			// 计算近锚点偏差值
//			float dx_near = rptsn[aim_idx_near][0] - cx;
//			float dy_near = cy - rptsn[aim_idx_near][1] + 0.2 * pixel_per_meter;
//			float dn_near = sqrt(dx_near * dx_near + dy_near * dy_near);
//			float error_near = -atan2f(dx_near, dy_near) * 180 / M_PI;
//			assert(!isnan(error_near));
//
//			// 远近锚点综合考虑
//			//angle = pid_solve(&servo_pid, error * far_rate + error_near * (1 - far_rate));
//			// 根据偏差进行PD计算
//			//float angle = pid_solve(&servo_pid, error);
//
//			// 纯跟踪算法(只考虑远点)
//			float pure_angle = -atanf(pixel_per_meter * 2 * 0.2 * dx / dn / dn) / M_PI * 180 / SMOTOR_RATE;
//			angle = pid_solve(&servo_pid, pure_angle);
//			angle = MINMAX(angle, -14.5, 14.5);
//
//			//非上坡电感控制,PD计算之后的值用于寻迹舵机的控制
//			if (enable_adc) {
//				// 当前上坡模式，不控制舵机，同时清除所有标志
//				yroad_type = YROAD_NONE;
//				circle_type = CIRCLE_NONE;
//				cross_type = CROSS_NONE;
//				garage_type = GARAGE_NONE;
//				apriltag_type = APRILTAG_NONE;
//			}
//			else if (adc_cross && cross_type == CROSS_IN) {
//				// 当前是十字模式，同时启用了电感过十字，则不控制舵机
//			}
//			else {
//				smotor1_control(servo_duty(SMOTOR1_CENTER + angle));
//			}
//
//		}
//		else {
//			// 中线点过少(出现问题)，则不控制舵机
//			rptsn_num = 0;
//		}
//
//		// 以下为绘制调试图像
//		if (gpio_get(DEBUGGER_PIN)) {
//
//			// 调试模式下定时写入flash参数
//			static int flash_cnt = 0;
//			if (++flash_cnt % 100 == 0) {
//				flash_param_write();
//			}
//
//			// 原图绘制中线
////            for(int i=0; i<rptsn_num; i++){
////                int pt[2];
////                if(map_inv(rptsn[i], pt)){ 
////                    AT_IMAGE(&img_raw, clip(pt[0], 0, img_raw.width-1), clip(pt[1], 0, img_raw.height-1)) = 0;
////                }
////            }
//
//			// 绘制二值化图像
//			if (p_active_image == &img1) {
//				//threshold(&img_raw, &img_thres, low_thres, 0, 255);
//				adaptive_threshold(&img_raw, &img_thres, block_size, clip_value, 0, 255);
//				for (int i = 0; i < img_thres.width / 2 - begin_x; i++) {
//					AT_IMAGE(&img_thres, (int)i, (int)begin_y) = 0;
//				}
//				for (int i = img_thres.width / 2 + begin_x; i < img_thres.width; i++) {
//					AT_IMAGE(&img_thres, (int)i, (int)begin_y) = 0;
//				}
//			}
//
//			//
//			clear_image(&img_line);
//			// 绘制道路元素
//			draw_yroad();
//			draw_circle();
//			draw_cross();
//
//			// 绘制道路线            
//			for (int i = 0; i < rpts0s_num; i++) {
//				AT_IMAGE(&img_line, clip(rpts0s[i][0], 0, img_line.width - 1),
//					clip(rpts0s[i][1], 0, img_line.height - 1)) = 255;
//			}
//			for (int i = 0; i < rpts1s_num; i++) {
//				AT_IMAGE(&img_line, clip(rpts1s[i][0], 0, img_line.width - 1),
//					clip(rpts1s[i][1], 0, img_line.height - 1)) = 255;
//			}
//			for (int i = 0; i < rptsn_num; i++) {
//				AT_IMAGE(&img_line, clip(rptsn[i][0], 0, img_line.width - 1),
//					clip(rptsn[i][1], 0, img_line.height - 1)) = 255;
//			}
//			// 绘制锚点
//			int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
//			draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
//			// 绘制角点
//			if (Lpt0_found) {
//				draw_x(&img_line, rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1], 3, 255);
//			}
//			if (Lpt1_found) {
//				draw_x(&img_line, rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1], 3, 255);
//			}
//		}
//
//		// 打印main-loop运行时间
//		uint32_t t2 = rt_tick_get_millisecond();
//		static uint8_t buffer[64];
//		int len = snprintf((char*)buffer, sizeof(buffer), "main time: %fms\n", (t2 - t1) / 1000.f);
//		//seekfree_wireless_send_buff(buffer, len);
//
//		// 调试模式下定时发送上位机数据
//		static int cnt = 0;
//		if (gpio_get(DEBUGGER_PIN)) {
//			if (++cnt % 4 == 0) debugger_worker();
//		}
//
//		// 打印其他调试数据
//		//flag_out();
//		//wireless_show();
//		//print_all();
//	}
//}


uint8* process_image(uint8 imagein[MT9V03X_CSI_H][MT9V03X_CSI_W],uint8_t side[POINTS_MAX_LEN*2*2]) {
	// 原图找左右边线
	for (int i = 0; i < MT9V03X_CSI_H; i++)
	{
		for (int j = 0; j < MT9V03X_CSI_W; j++)
		{
			AT_IMAGE(&img_raw, i, j) = imagein[i][j];
			//printf("%d ", AT_IMAGE(&img_raw, i, j));
		}
	}
	img_raw.height = MT9V03X_CSI_H;
	img_raw.width = MT9V03X_CSI_W;
	int x1 = img_raw.width / 2 - begin_x, y1 = begin_y;   //x=94 y=119
	//int x1 = begin_y, y1 = img_raw.width / 2 - begin_x ;
	ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
	//for (; x1 > 0; x1--) if (AT_IMAGE(&img_raw, x1 - 1, y1) < thres) break;
	// 寻找左边线起始点(y=119,x=30)
	//for (; x1 > 0; x1--) if (AT_IMAGE(&img_raw, y1, x1-1) < thres) break;
	for (; x1 > 0; x1--) if (AT_IMAGE(&img_raw, y1, x1 )==255 ) break;
	//printf("(x1=%d,y1=%d,th=%d)",y1,x1, AT_IMAGE(&img_raw, y1, x1));
	// 四邻域搜线
	if (AT_IMAGE(&img_raw, y1, x1) >= thres)
		findline_lefthand_adaptive(&img_raw, block_size, clip_value, x1, y1, ipts0, &ipts0_num);
	else ipts0_num = 0;
	int x2 = img_raw.width / 2 + begin_x, y2 = begin_y;   //x=94  y=115
	//printf("222");
	ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
	//for (; x2 < img_raw.width - 1; x2++) if (AT_IMAGE(&img_raw, x2 + 1, y2) < thres) break;
	// 寻找右边线起始点(y=119,x=164)
	for (; x2 < img_raw.width - 1; x2++) if (AT_IMAGE(&img_raw, y2, x2+1) < thres) break;
	printf("(x2=%d,y2=%d,th=%d)", y2, x2, AT_IMAGE(&img_raw, y2, x2));
	if (AT_IMAGE(&img_raw, y2, x2) >= thres)
		findline_righthand_adaptive(&img_raw, block_size, clip_value, x2, y2, ipts1, &ipts1_num);
	else ipts1_num = 0;
	//printf("444");
	for (int i = 0,m=0; i < POINTS_MAX_LEN*2; i=i+2,m++)
	{
		//printf("\n(ss=%d,ss=%d) ", ipts0[m][0], ipts0[m][1]);
		side[i] = ipts0[m][0];
		side[i + 1] = ipts0[m][1];
	}

	for (int i = POINTS_MAX_LEN * 2, m = 0; i < POINTS_MAX_LEN * 4; i = i + 2, m++)
	{
		//printf("(%d,%d) ", ipts1[m][0], ipts1[m][1]);
		side[i] = ipts1[m][0];
		side[i + 1] = ipts1[m][1];
	}

	return side;    //搜左边线起始点x1，y1   搜右边线起始点x2，y2
	// 去畸变+透视变换
	for (int i = 0; i < ipts0_num; i++) {
		rpts0[i][0] = mapx[ipts0[i][1]][ipts0[i][0]];
		rpts0[i][1] = mapy[ipts0[i][1]][ipts0[i][0]];
	}
	rpts0_num = ipts0_num;
	for (int i = 0; i < ipts1_num; i++) {
		rpts1[i][0] = mapx[ipts1[i][1]][ipts1[i][0]];
		rpts1[i][1] = mapy[ipts1[i][1]][ipts1[i][0]];
	}
	rpts1_num = ipts1_num;

	// 边线滤波
	blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
	rpts0b_num = rpts0_num;
	blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
	rpts1b_num = rpts1_num;

	// 边线等距采样
	rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
	resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
	rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
	resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);

	// 边线局部角度变化率
	local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(angle_dist / sample_dist));
	rpts0a_num = rpts0s_num;
	local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(angle_dist / sample_dist));
	rpts1a_num = rpts1s_num;

	// 角度变化率非极大抑制
	nms_angle(rpts0a, rpts0a_num, rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
	rpts0an_num = rpts0a_num;
	nms_angle(rpts1a, rpts1a_num, rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
	rpts1an_num = rpts1a_num;

	// 左右中线跟踪
	track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
	rptsc0_num = rpts0s_num;
	track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
	rptsc1_num = rpts1s_num;
}

void find_corners() {
	// 识别Y,L拐点
	Ypt0_found = Ypt1_found = Lpt0_found = Lpt1_found = false;
	is_straight0 = rpts0s_num > 1. / sample_dist;
	is_straight1 = rpts1s_num > 1. / sample_dist;
	for (int i = 0; i < rpts0s_num; i++) {
		if (rpts0an[i] == 0) continue;
		int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
		int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
		float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;

		//Y角点阈值
		if (Ypt0_found == false && 30. / 180. * M_PI < conf && conf < 65. / 180. * M_PI && i < 0.8 / sample_dist) {
			Ypt0_rpts0s_id = i;
			Ypt0_found = true;
		}
		//L角点阈值
		if (Lpt0_found == false && 70. / 180. * M_PI < conf && conf < 140. / 180. * M_PI && i < 0.8 / sample_dist) {
			Lpt0_rpts0s_id = i;
			Lpt0_found = true;
		}
		//长直道阈值
		if (conf > 5. / 180. * M_PI && i < 1.0 / sample_dist) is_straight0 = false;
		if (Ypt0_found == true && Lpt0_found == true && is_straight0 == false) break;
	}
	for (int i = 0; i < rpts1s_num; i++) {
		if (rpts1an[i] == 0) continue;
		int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
		int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
		float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
		if (Ypt1_found == false && 30. / 180. * M_PI < conf && conf < 65. / 180. * M_PI && i < 0.8 / sample_dist) {
			Ypt1_rpts1s_id = i;
			Ypt1_found = true;
		}
		if (Lpt1_found == false && 70. / 180. * M_PI < conf && conf < 140. / 180. * M_PI && i < 0.8 / sample_dist) {
			Lpt1_rpts1s_id = i;
			Lpt1_found = true;
		}

		if (conf > 5. / 180. * M_PI && i < 1.0 / sample_dist) is_straight1 = false;

		if (Ypt1_found == true && Lpt1_found == true && is_straight1 == false) break;
	}
	// Y点二次检查,依据两角点距离及角点后张开特性
	if (Ypt0_found && Ypt1_found) {
		float dx = rpts0s[Ypt0_rpts0s_id][0] - rpts1s[Ypt1_rpts1s_id][0];
		float dy = rpts0s[Ypt0_rpts0s_id][1] - rpts1s[Ypt1_rpts1s_id][1];
		float dn = sqrtf(dx * dx + dy * dy);
		if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter) {
			float dwx = rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
				rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
			float dwy = rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
				rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
			float dwn = sqrtf(dwx * dwx + dwy * dwy);
			if (!(dwn > 0.7 * pixel_per_meter &&
				rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] < rpts0s[Ypt0_rpts0s_id][0] &&
				rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0] > rpts1s[Ypt1_rpts1s_id][0])) {
				Ypt0_found = Ypt1_found = false;
			}
		}
		else {
			Ypt0_found = Ypt1_found = false;
		}
	}
	// L点二次检查，车库模式不检查, 依据L角点距离及角点后张开特性
	//if (garage_type == GARAGE_NONE) {
	//	if (Lpt0_found && Lpt1_found) {
	//		float dx = rpts0s[Lpt0_rpts0s_id][0] - rpts1s[Lpt1_rpts1s_id][0];
	//		float dy = rpts0s[Lpt0_rpts0s_id][1] - rpts1s[Lpt1_rpts1s_id][1];
	//		float dn = sqrtf(dx * dx + dy * dy);
	//		if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter) {
	//			float dwx = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
	//				rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
	//			float dwy = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
	//				rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
	//			float dwn = sqrtf(dwx * dwx + dwy * dwy);
	//			if (!(dwn > 0.7 * pixel_per_meter &&
	//				rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] < rpts0s[Lpt0_rpts0s_id][0] &&
	//				rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0] > rpts1s[Lpt1_rpts1s_id][0])) {
	//				Lpt0_found = Lpt1_found = false;
	//			}
	//		}
	//		else {
	//			Lpt0_found = Lpt1_found = false;
	//		}
	//	}
	//}
}


//舵机控制偏差
float angle;






/**  @brief    主跑行  */
#define ROAD_MAIN_ROW      40

/**  @brief    使用起始行120  */
#define ROAD_START_ROW     119

/**  @brief    使用结束行10  */
#define ROAD_END_ROW       1
//

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
// * @brief 开发版参数
// */
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
//
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

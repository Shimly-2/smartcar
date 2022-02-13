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
  * @brief    ͼ����
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
  // ԭͼ���ұ���
int ipts0[POINTS_MAX_LEN][2];
int ipts1[POINTS_MAX_LEN][2];
int ipts0_num, ipts1_num;
// �任�����ұ���
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
int rpts0_num, rpts1_num;
// �任�����ұ���+�˲�
float rpts0b[POINTS_MAX_LEN][2];
float rpts1b[POINTS_MAX_LEN][2];
int rpts0b_num, rpts1b_num;
// �任�����ұ���+�Ⱦ����
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
int rpts0s_num, rpts1s_num;
// ���ұ��߾ֲ��Ƕȱ仯��
float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
int rpts0a_num, rpts1a_num;
// ���ұ��߾ֲ��Ƕȱ仯��+�Ǽ�������
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
int rpts0an_num, rpts1an_num;
// ��/������
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
int rptsc0_num, rptsc1_num;
// ����
float(*rpts)[2];
int rpts_num;
// ��һ������
float rptsn[POINTS_MAX_LEN][2];
int rptsn_num;

// Y�ǵ�
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;

// L�ǵ�
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;

// ��ֱ��
bool is_straight0, is_straight1;

// ���
bool is_turn0, is_turn1;

// ��ǰѲ��ģʽ
enum track_type_e track_type = TRACK_RIGHT;


////���ڴ�ӡ��־λ,������
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
//	//��������Ư����
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
//	// ����flash����
//	flash_param_init();
//	if (flash_param_check()) {
//		flash_param_load();
//	}
//
//	// ��ʼ������GPIO
//	gpio_init(DEBUGGER_PIN, GPI, 0, GPIO_PIN_CONFIG);
//	gpio_init(C30, GPI, 0, GPIO_PIN_CONFIG);
//	if (gpio_get(C30) == 0) garage_type = GARAGE_OUT_LEFT;
//	else garage_type = GARAGE_OUT_RIGHT;
//
//	// ��λ��ע��
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
//		// �ȴ�����ͷ�ɼ����
//		rt_sem_take(camera_sem, RT_WAITING_FOREVER);
//		img_raw.data = mt9v03x_csi_image[0];
//		img0.buffer = mt9v03x_csi_image[0];
//
//		// main-loop��ʱ
//		uint32_t t1 = rt_tick_get_millisecond();
//		// ��ʼ��������ͷͼ��
//		process_image();    // ������ȡ&����
//		find_corners();     // �ǵ���ȡ&ɸѡ
//
//		// Ԥ�����,��̬Ч������
//		aim_distance = 0.58;
//
//		// �������٣��л�Ѳ�߷���  ������Բ
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
//		// ��������߼��(���������ȼ��ߣ����ȼ��)
//		check_garage();
//
//		// �������Apriltag(�������ϵĺڰ�)
//		if (!enable_adc && garage_type == GARAGE_NONE && get_total_encoder() - openart.aprilencoder > ENCODER_PER_METER)
//			check_apriltag();
//
//		// �ֱ���ʮ�� ���� ��Բ��, ʮ�����ȼ����
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
//		//���� ,ʮ����Aprltag��־
//		if (garage_type != GARAGE_NONE || cross_type != CROSS_NONE) apriltag_type = APRILTAG_NONE;
//
//		//���ݼ����ִ��ģʽ
//		if (yroad_type != YROAD_NONE) run_yroad();
//		if (cross_type != CROSS_NONE) run_cross();
//		if (circle_type != CIRCLE_NONE) run_circle();
//		if (garage_type != GARAGE_NONE) run_garage();
//
//		//���Openart�շ�
//		check_openart();
//
//		// ���߸���
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
//			//ʮ�ָ���Զ�߿���
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
//		// ���ֶ�Ӧ��(��������ʼ��)
//		float cx = mapx[MT9V03X_CSI_W / 2][(int)(MT9V03X_CSI_H * 0.78f)];
//		float cy = mapy[MT9V03X_CSI_W / 2][(int)(MT9V03X_CSI_H * 0.78f)];
//
//		// �������(��ʼ�����߹�һ��)
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
//		// ����ģʽ�£����������(���ڱ��߻���һȦ���������������Ϊ�������һ���㣬�Ӷ������޷���������)
//		if (garage_type == GARAGE_IN_LEFT || garage_type == GARAGE_IN_RIGHT || cross_type == CROSS_IN) begin_id = 0;
//
//		// �����е㣬ͬʱ����㲻����󼸸���
//		if (begin_id >= 0 && rpts_num - begin_id >= 3) {
//			// ��һ������
//			rpts[begin_id][0] = cx;
//			rpts[begin_id][1] = cy;
//			rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
//			resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
//
//			// ԶԤê��λ��
//			int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
//			// ��Ԥê��λ��
//			int aim_idx_near = clip(round(0.25 / sample_dist), 0, rptsn_num - 1);
//
//			// ����Զê��ƫ��ֵ
//			float dx = rptsn[aim_idx][0] - cx;
//			float dy = cy - rptsn[aim_idx][1] + 0.2 * pixel_per_meter;
//			float dn = sqrt(dx * dx + dy * dy);
//			float error = -atan2f(dx, dy) * 180 / M_PI;
//			assert(!isnan(error));
//
//			// �����ǽ���Զ��,�ɽ��ƹ���Stanley�㷨,����ײ·��
//			// �����ê��ƫ��ֵ
//			float dx_near = rptsn[aim_idx_near][0] - cx;
//			float dy_near = cy - rptsn[aim_idx_near][1] + 0.2 * pixel_per_meter;
//			float dn_near = sqrt(dx_near * dx_near + dy_near * dy_near);
//			float error_near = -atan2f(dx_near, dy_near) * 180 / M_PI;
//			assert(!isnan(error_near));
//
//			// Զ��ê���ۺϿ���
//			//angle = pid_solve(&servo_pid, error * far_rate + error_near * (1 - far_rate));
//			// ����ƫ�����PD����
//			//float angle = pid_solve(&servo_pid, error);
//
//			// �������㷨(ֻ����Զ��)
//			float pure_angle = -atanf(pixel_per_meter * 2 * 0.2 * dx / dn / dn) / M_PI * 180 / SMOTOR_RATE;
//			angle = pid_solve(&servo_pid, pure_angle);
//			angle = MINMAX(angle, -14.5, 14.5);
//
//			//�����µ�п���,PD����֮���ֵ����Ѱ������Ŀ���
//			if (enable_adc) {
//				// ��ǰ����ģʽ�������ƶ����ͬʱ������б�־
//				yroad_type = YROAD_NONE;
//				circle_type = CIRCLE_NONE;
//				cross_type = CROSS_NONE;
//				garage_type = GARAGE_NONE;
//				apriltag_type = APRILTAG_NONE;
//			}
//			else if (adc_cross && cross_type == CROSS_IN) {
//				// ��ǰ��ʮ��ģʽ��ͬʱ�����˵�й�ʮ�֣��򲻿��ƶ��
//			}
//			else {
//				smotor1_control(servo_duty(SMOTOR1_CENTER + angle));
//			}
//
//		}
//		else {
//			// ���ߵ����(��������)���򲻿��ƶ��
//			rptsn_num = 0;
//		}
//
//		// ����Ϊ���Ƶ���ͼ��
//		if (gpio_get(DEBUGGER_PIN)) {
//
//			// ����ģʽ�¶�ʱд��flash����
//			static int flash_cnt = 0;
//			if (++flash_cnt % 100 == 0) {
//				flash_param_write();
//			}
//
//			// ԭͼ��������
////            for(int i=0; i<rptsn_num; i++){
////                int pt[2];
////                if(map_inv(rptsn[i], pt)){ 
////                    AT_IMAGE(&img_raw, clip(pt[0], 0, img_raw.width-1), clip(pt[1], 0, img_raw.height-1)) = 0;
////                }
////            }
//
//			// ���ƶ�ֵ��ͼ��
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
//			// ���Ƶ�·Ԫ��
//			draw_yroad();
//			draw_circle();
//			draw_cross();
//
//			// ���Ƶ�·��            
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
//			// ����ê��
//			int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
//			draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
//			// ���ƽǵ�
//			if (Lpt0_found) {
//				draw_x(&img_line, rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1], 3, 255);
//			}
//			if (Lpt1_found) {
//				draw_x(&img_line, rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1], 3, 255);
//			}
//		}
//
//		// ��ӡmain-loop����ʱ��
//		uint32_t t2 = rt_tick_get_millisecond();
//		static uint8_t buffer[64];
//		int len = snprintf((char*)buffer, sizeof(buffer), "main time: %fms\n", (t2 - t1) / 1000.f);
//		//seekfree_wireless_send_buff(buffer, len);
//
//		// ����ģʽ�¶�ʱ������λ������
//		static int cnt = 0;
//		if (gpio_get(DEBUGGER_PIN)) {
//			if (++cnt % 4 == 0) debugger_worker();
//		}
//
//		// ��ӡ������������
//		//flag_out();
//		//wireless_show();
//		//print_all();
//	}
//}


uint8* process_image(uint8 imagein[MT9V03X_CSI_H][MT9V03X_CSI_W],uint8_t side[POINTS_MAX_LEN*2*2]) {
	// ԭͼ�����ұ���
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
	// Ѱ���������ʼ��(y=119,x=30)
	//for (; x1 > 0; x1--) if (AT_IMAGE(&img_raw, y1, x1-1) < thres) break;
	for (; x1 > 0; x1--) if (AT_IMAGE(&img_raw, y1, x1 )==255 ) break;
	//printf("(x1=%d,y1=%d,th=%d)",y1,x1, AT_IMAGE(&img_raw, y1, x1));
	// ����������
	if (AT_IMAGE(&img_raw, y1, x1) >= thres)
		findline_lefthand_adaptive(&img_raw, block_size, clip_value, x1, y1, ipts0, &ipts0_num);
	else ipts0_num = 0;
	int x2 = img_raw.width / 2 + begin_x, y2 = begin_y;   //x=94  y=115
	//printf("222");
	ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
	//for (; x2 < img_raw.width - 1; x2++) if (AT_IMAGE(&img_raw, x2 + 1, y2) < thres) break;
	// Ѱ���ұ�����ʼ��(y=119,x=164)
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

	return side;    //���������ʼ��x1��y1   ���ұ�����ʼ��x2��y2
	// ȥ����+͸�ӱ任
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

	// �����˲�
	blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
	rpts0b_num = rpts0_num;
	blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
	rpts1b_num = rpts1_num;

	// ���ߵȾ����
	rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
	resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
	rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
	resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);

	// ���߾ֲ��Ƕȱ仯��
	local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(angle_dist / sample_dist));
	rpts0a_num = rpts0s_num;
	local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(angle_dist / sample_dist));
	rpts1a_num = rpts1s_num;

	// �Ƕȱ仯�ʷǼ�������
	nms_angle(rpts0a, rpts0a_num, rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
	rpts0an_num = rpts0a_num;
	nms_angle(rpts1a, rpts1a_num, rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
	rpts1an_num = rpts1a_num;

	// �������߸���
	track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
	rptsc0_num = rpts0s_num;
	track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
	rptsc1_num = rpts1s_num;
}

void find_corners() {
	// ʶ��Y,L�յ�
	Ypt0_found = Ypt1_found = Lpt0_found = Lpt1_found = false;
	is_straight0 = rpts0s_num > 1. / sample_dist;
	is_straight1 = rpts1s_num > 1. / sample_dist;
	for (int i = 0; i < rpts0s_num; i++) {
		if (rpts0an[i] == 0) continue;
		int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
		int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
		float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;

		//Y�ǵ���ֵ
		if (Ypt0_found == false && 30. / 180. * M_PI < conf && conf < 65. / 180. * M_PI && i < 0.8 / sample_dist) {
			Ypt0_rpts0s_id = i;
			Ypt0_found = true;
		}
		//L�ǵ���ֵ
		if (Lpt0_found == false && 70. / 180. * M_PI < conf && conf < 140. / 180. * M_PI && i < 0.8 / sample_dist) {
			Lpt0_rpts0s_id = i;
			Lpt0_found = true;
		}
		//��ֱ����ֵ
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
	// Y����μ��,�������ǵ���뼰�ǵ���ſ�����
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
	// L����μ�飬����ģʽ�����, ����L�ǵ���뼰�ǵ���ſ�����
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


//�������ƫ��
float angle;






/**  @brief    ������  */
#define ROAD_MAIN_ROW      40

/**  @brief    ʹ����ʼ��120  */
#define ROAD_START_ROW     119

/**  @brief    ʹ�ý�����10  */
#define ROAD_END_ROW       1
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

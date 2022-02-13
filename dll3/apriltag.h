#pragma once
#ifndef APRILTAG_H
#define APRILTAG_H

enum apriltag_type_e {
	APRILTAG_NONE = 0,  // �޺ڰ�apriltag
	APRILTAG_MAYBE,     // ʶ��Զ����ڰߣ�����
	APRILTAG_FOUND,     // ʶ�𵽽�����ڰߣ�ͣ��
	APRILTAG_LEAVE,     // ʻ��apriltag�У��������жϣ���ֹ������־���ж��ܵ�apriltag��Ӱ��
	APRILTAG_NUM,       // 
};

extern enum apriltag_type_e apriltag_type;
extern int apriltag_time;

extern const char* apriltag_type_name[APRILTAG_NUM];

void check_apriltag();

#endif // APRILTAG_H
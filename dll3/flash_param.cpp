#include "flash_param.h"
//#include "zf_flash.h"

//
#define PARAM_FLASH_SECTOR        (FLASH_SECTOR_NUM-1)

//
#define PARAM_FLASH_SECTOR_PAGE   (0)

// flashһ��page��256���ֽڣ������4���ֽ���Ϊ�Ӻ�У�飬�����ֽ����ڷ��ò���

// ��flash������һ���Զ�ȡ�������ٽ���У��λ�ļ��
uint32_t flash_buffer[64];

//// ʵ�ʳ�������Ĵ����ʼ��ַ
//uint32_t* const flash_data = (void*)0x20200000;

// ��ʽ�涨�����ĵ�ַ����֤����������0x20200000~0x20200100��Ƭ�ڴ����򡣱��ڱ��浽flash�Լ���flash�м���
float thres = 125;
float block_size = 7;
float clip_value = 2;
float begin_x = 0;
float begin_y = 115;
float line_blur_kernel = 7;
float pixel_per_meter = 102;
float sample_dist = 0.02;
float angle_dist = 0.2;
float far_rate = 0.5;
float aim_distance = 0.68;
bool adc_cross = false;

// ���4���ֽڷ�У��ֵ
static uint32_t check = 0;


// ���У��ֵ�������ж�flash�Ƿ���������
bool flash_param_check() {
	uint32_t check_value = 0xA5;    // magic number��Ϊ��ֵ������ȫƬ����������ͨ��У�顣
	for (int i = 0; i < 63; i++) {
		check_value += flash_buffer[i];
	}
	return check_value == flash_buffer[63];
}

//// ����flash����������flash�����ݿ�����������Ӧ���ڴ�λ��
//void flash_param_load() {
//	memcpy(flash_data, flash_buffer, 256);
//}
//
//// дflash������
//void flash_param_write() {
//	check = 0;
//	for (int i = 0; i < 63; i++) {
//		check += flash_data[i];
//	}
//	// �Ȳ�����д�룬����д���ʧ��
//	flash_erase_sector(PARAM_FLASH_SECTOR);
//	flash_page_program(PARAM_FLASH_SECTOR, PARAM_FLASH_SECTOR_PAGE, (uint32*)flash_data, 64);
//}

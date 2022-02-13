# -*- coding:utf-8 -*-
"""
@ author   YYY
@ desc     用于17届智能车比赛-平衡单车组上位机开发
@ date     2022/2/12
@ file     test.py
@ version  1.3
"""
import time,sys
from tkinter.font import Font
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtWidgets, QtCore, QtGui
import json
import cv2
import os
import numpy as np
from ctypes import *
import datetime
from PIL import Image
from imageprocess import *

# 加载动态链接库
mylib = cdll.LoadLibrary(r'C:/Users/17628/Desktop/ui/dll/x64/Debug/dll.dll')
mylib2 = cdll.LoadLibrary(r'C:/Users/17628/Desktop/ui/dll2/x64/Debug/dll.dll')
mylib3 = cdll.LoadLibrary(r'C:/Users/17628/Desktop/ui/dll3/x64/Debug/dll.dll')


# 使用起始行120  
ROAD_START_ROW=115

# 使用结束行10  
ROAD_END_ROW=10

# 主跑行
ROAD_MAIN_ROW = 40

# 图像长宽
MT9V03X_H = 120
MT9V03X_W = 188

global inn
inn=[]
for i in range(120):
	inn.append([])
	for j in range(188):
		inn[i].append(0)

# 图像数组 120*188
global aa
aa=[]
for i in range(120):
	aa.append([])
	for j in range(188):
		aa[i].append(0)

# 边线数组 120*3
global bb
bb=[]
for i in range(120):
	bb.append([])
	for j in range(2):
		bb[i].append(0)

# 滤波后数组 120*188
global bc
bc=[]
for i in range(120):
	bc.append([])
	for j in range(188):
		bc[i].append(0)

# 拐点数组，未用到
global guai
guai=[]
for i in range(4):
	guai.append([])
	for j in range(2):
		guai[i].append(0)

# 边线数组 64*3
global side
side=[]
for i in range(64):
	side.append([])
	for j in range(3):
		side[i].append(0)

# 图像数组 64*188
global md
md=[]
for i in range(64):
	md.append([])
	for j in range(188):
		md[i].append(0)

# 边线数组 150*3*2
global side2
side2=[]
for i in range(450):
	side2.append([])
	for j in range(2):
		side2[i].append(0)

# 边线数组 240*2
global side3
side3=[]
for i in range(480):
	side3.append([])
	for j in range(2):
		side3[i].append(0)

# 参数数组 30*1
global value2
value2=[]
for i in range(30):
	value2.append(0)

# 参数传递数组 30*1
global value3
value3=[]
for i in range(30):
	value3.append(0)

# 图像数组 120*160
global md2
md2=[]
for i in range(120):
	md2.append([])
	for j in range(160):
		md2[i].append(0)

# 图像数组 120*160
global img_binary
img_binary=[]
for i in range(120):
	img_binary.append([])
	for j in range(160):
		img_binary[i].append(0)

# 左边线
global left_line
left_line=[]
for i in range(120):
	left_line.append([])
	for j in range(2):
		left_line[i].append(0)


# 右边线
global right_line
right_line=[]
for i in range(120):
	right_line.append([])
	for j in range(2):
		right_line[i].append(0)

# 中线
global mid_line
mid_line=[]
for i in range(120):
	mid_line.append([])
	for j in range(2):
		mid_line[i].append(0)

# 判断是不是黑点
def black_(x):
	if (x == 0):
		return 1
	elif (x == 255):
		return 0

# 判断是不是白点
def white_(x):
	if (x == 255):
		return 1
	elif (x == 0):
		return 0

# 存储边线调试变量类
class control_unit_bianxian(object):
    #类变量
    file_name = 'student.txt'
    def __init__(self):
        self.L_basic_row_start = 118    	# 左边界行搜索起始点
        self.R_basic_row_start = 118    	# 右边界行搜索起始点
        self.L_edge_start_col = 3    		# 左边界列搜索起始点
        self.R_edge_start_col = 157     	# 右边界列搜索起始点
        self.L_search_amount = 150  		# 左边界搜点最多允许数
        self.R_search_amount = 150    		# 右边界搜点最多允许数
        self.min_col = 3           			# 左搜索结束列值
        self.max_col = 157       			# 右搜索结束列值
        self.L_lost_ = 10  					# 左丢线限制次数
        self.R_lost_ = 10    				# 右丢线限制次数
        self.L_edge_lost_start_col = 4   	# 中间左丢线列搜索起始点
        self.R_edge_lost_start_col = 155    # 中间右丢线列搜索起始点
        self.dist = 4         				# 求拐点角度的点数间隔
    #保存数据的函数
    def save_data(self):
        #1.打开文件
        f = open(self.file_name,'a',encoding='utf-8')
        #2.写入数据
        time = datetime.datetime.now()
        time1_str = datetime.datetime.strftime(time,'%Y-%m-%d %H:%M:%S')
        print(time1_str)
        f.write("------------------")
        f.write(time1_str)
        f.write("------------------\n")

        f.write("self.L_basic_row_start = ")
        f.write(str(self.L_basic_row_start))
        f.write("    	# 左边界行搜索起始点")
        f.write('\n')

        f.write("self.R_basic_row_start = ")
        f.write(str(self.R_basic_row_start))
        f.write("    	# 右边界行搜索起始点")
        f.write('\n')

        f.write("self.L_edge_start_col = ")
        f.write(str(self.L_edge_start_col))
        f.write("    		# 左边界列搜索起始点")
        f.write('\n')

        f.write("self.R_edge_start_col = ")
        f.write(str(self.R_edge_start_col))
        f.write("     	# 右边界列搜索起始点")
        f.write('\n')

        f.write("self.L_search_amount = ")
        f.write(str(self.L_search_amount))
        f.write("  		# 左边界搜点最多允许数")
        f.write('\n')

        f.write("self.R_search_amount = ")
        f.write(str(self.R_search_amount))
        f.write("  		# 右边界搜点最多允许数")
        f.write('\n')

        f.write("self.min_col = ")
        f.write(str(self.min_col))
        f.write("           			# 左搜索结束列值")
        f.write('\n')

        f.write("self.max_col = ")
        f.write(str(self.max_col))
        f.write("       			# 右搜索结束列值")
        f.write('\n')

        f.write("self.L_lost_ = ")
        f.write(str(self.L_lost_))
        f.write("  					# 左丢线限制次数")
        f.write('\n')

        f.write("self.R_lost_ = ")
        f.write(str(self.R_lost_))
        f.write("  					# 右丢线限制次数")
        f.write('\n')

        f.write("self.L_edge_lost_start_col = ")
        f.write(str(self.L_edge_lost_start_col))
        f.write("   	# 中间左丢线列搜索起始点")
        f.write('\n')

        f.write("self.R_edge_lost_start_col = ")
        f.write(str(self.R_edge_lost_start_col))
        f.write("    # 中间右丢线列搜索起始点")
        f.write('\n')

        f.write("self.dist = ")
        f.write(str(self.dist))
        f.write("         				# 求拐点角度的点数间隔")
        f.write('\n')
        
        f.write("-----------------------------------------------------------\n")
        #3.关闭文件
        f.close()
# 创建对象
control_unit_bianxian.file_name = 'control_unit_bianxian.txt'
ctr_bianxian=control_unit_bianxian()

# 存储二值化调试变量类
class control_unit_erzhi(object):
    #类变量
    file_name = 'student.txt'
    def __init__(self):
        self.sobel_motor = 50   # sobel算子手动调节阈值
        self.sobel_auto = 10    # sobel算子自动调节像素比
        self.th = 130           # 固定阈值
        self.filter = 4         # 滤波效果
    #保存数据的函数
    def save_data(self):
        #1.打开文件
        f = open(self.file_name,'a',encoding='utf-8')
        #2.写入数据
        time = datetime.datetime.now()
        time1_str = datetime.datetime.strftime(time,'%Y-%m-%d %H:%M:%S')
        print(time1_str)
        f.write("------------------")
        f.write(time1_str)
        f.write("------------------\n")

        f.write("self.sobel_motor = ")
        f.write(str(self.sobel_motor))
        f.write("   # sobel算子手动调节阈值")
        f.write('\n')

        f.write("self.sobel_auto = ")
        f.write(str(self.sobel_auto))
        f.write("    # sobel算子自动调节像素比")
        f.write('\n')

        f.write("self.th = ")
        f.write(str(self.th))
        f.write("           # 固定阈值")
        f.write('\n')

        f.write("self.filter = ")
        f.write(str(self.filter))
        f.write("         # 滤波效果")
        f.write('\n')
        
        f.write("-----------------------------------------------------------\n")
        #3.关闭文件
        f.close()
# 创建对象
control_unit_erzhi.file_name = 'control_unit_erzhi.txt'
ctr=control_unit_erzhi()

# 二值化图像参数类
class config:
	WIDTH=188							#地图列数
	HEIGHT=120							#地图行数
	blockLength=8						#绘制画面时每一个节点方块的边长
	blockLength2=3						#绘制画面时每一个节点方块的边长

# 二值化工具可视化类，pyqt5进行编写。
class board_widget(QMainWindow):
	# 视频播放列表
	bClose = False
	fn_list = ["C:/Users/17628/Desktop/ui/test video/慢速版.mp4","C:/Users/17628/Desktop/ui/test video/正常版.mp4"]
	play_index = 0
	def __init__(self):
        # 1.初始化：初始化地图数组
		print('初始化二值化工具...')
		self.Map=[]
		for i in range(config.HEIGHT):
			col=[]
			for j in range(config.WIDTH):
				col.append(220)
			self.Map.append(col)
        # 2.初始化：起点和目的地
		self.startPoint=None
		self.endPoint=None
        # 3.初始化：其他参数
		self.search=None
		self.centerTimer=None
		self.yi=None
		self.special=None
		self.displayFlush=False
		super().__init__()
		self.initUI()
	def initUI(self):
		#开始初始化UI部分
		self.label_display=QTextEdit("",self)
		self.button_start=QPushButton("开始巡线",self)
		self.button_clearWall=QPushButton("清空地图墙壁",self)
		self.button_saveMap=QPushButton("一键存参",self)
		self.button_saveMap.setShortcut("Ctrl+s")
		self.button_LQ_OTSU=QPushButton("龙邱OTSU",self)
		self.button_OTSU=QPushButton("OTSU",self)
		self.button_th_average=QPushButton("平均阈值",self)
		self.button_sobel_motor=QPushButton("sobel算子 手动阈值",self)
		self.button_sobel_auto=QPushButton("sobel算子 动态阈值",self)
		self.button_th_motor=QPushButton("手动调节阈值",self)
		self.button_filter=QPushButton("图像滤波",self)

		# 字体类设置
		font = QtGui.QFont()
		font.setFamily("Consolas")
		font.setPointSize(12)
		font.setBold(False)
		font.setItalic(False)
		font.setWeight(3)

		font2 = QtGui.QFont()
		font2.setFamily("Consolas")
		font2.setPointSize(10)
		font2.setBold(False)
		font2.setItalic(False)
		font2.setWeight(3)

		self.button_start.setFont(font)
		self.button_clearWall.setFont(font)
		self.button_saveMap.setFont(font)
		self.button_saveMap.setFont(font)
		self.button_LQ_OTSU.setFont(font)
		self.button_OTSU.setFont(font)
		self.button_th_average.setFont(font)
		self.button_sobel_motor.setFont(font)
		self.button_sobel_auto.setFont(font)
		self.button_th_motor.setFont(font)
		self.button_filter.setFont(font)
		self.label_display.setFont(font)

		self.label_1=QLabel("全局阈值",self)
		self.label_2=QLabel("130",self)
		self.label_2.setFont(font)

		self.label_5=QLabel("sobel手动",self)
		self.label_6=QLabel("50",self)
		self.label_6.setFont(font)

		self.label_7=QLabel("sobel自动",self)
		self.label_8=QLabel("10.0",self)
		self.label_8.setFont(font)

		self.label_9=QLabel("滤波效果",self)
		self.label_10=QLabel("4",self)
		self.label_10.setFont(font)

		# self.collec_btn = QPushButton('调试小摩托', self)
		# self.collec_btn.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        #                                    "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        #                                    "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		# self.collec_btn.move(1580, 10)

		# self.collec_btn2 = QPushButton('八邻域搜线算法', self)
		# self.collec_btn2.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        #                                    "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        #                                    "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		# self.collec_btn2.move(1580, 950)

		self.label = QLabel(self)
		self.label.setText("显示图片")
		self.label.setFixedSize(188, 120)
		self.label.move(1630, 50)

		self.label_3 = QLabel(self)
		self.label_3.setText("OTSU阈值：")
		self.label_3.setFixedSize(50, 50)
		self.label_3.move(1830, 80)

		self.label_4 = QLabel(self)
		self.label_4.setText("")
		self.label_4.setFixedSize(50, 50)
		self.label_4.move(1850, 120)

		self.radioButton = QRadioButton(self)
		self.radioButton.setText("OTSU")
		self.radioButton.resize(120,30)
		self.radioButton.move(1580,630)

		self.radioButton2 = QRadioButton(self)
		self.radioButton2.setText("龙邱OTSU")
		self.radioButton2.resize(120,30)
		self.radioButton2.move(1700,630)

		self.radioButton3 = QRadioButton(self)
		self.radioButton3.setText("手动阈值")
		self.radioButton3.resize(120,30)
		self.radioButton3.move(1820,630)

		self.radioButton4 = QRadioButton(self)
		self.radioButton4.setText("sobel手动阈值")
		self.radioButton4.resize(120,30)
		self.radioButton4.move(1580,660)

		self.radioButton5 = QRadioButton(self)
		self.radioButton5.setText("sobel自动阈值")
		self.radioButton5.resize(120,30)
		self.radioButton5.move(1700,660)

		self.radioButton6 = QRadioButton(self)
		self.radioButton6.setText("平均阈值")
		self.radioButton6.resize(120,30)
		self.radioButton6.move(1820,660)

		self.btn = QPushButton(self)
		self.btn.setText("打开图片")
		self.btn.move(1580, 10)
		self.btn.resize(150,30)
		self.btn.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.btn.clicked.connect(self.openimage)
		self.btn.setFont(font)

		self.play_video = QPushButton(self)
		self.play_video.setText("打开视频")
		self.play_video.move(1740, 10)
		self.play_video.resize(150,30)
		self.play_video.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.play_video.clicked.connect(self.manual_choose)
		self.play_video.setFont(font)

		# 添加滑动条控件
		self.horizontalSlider =QSlider(self)
		self.horizontalSlider.setMaximum(255)
		self.horizontalSlider.setSingleStep(1)
		self.horizontalSlider.setPageStep(1)
		self.horizontalSlider.setProperty("value", 130)
		self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
		self.horizontalSlider.setObjectName("horizontalSlider")
		self.horizontalSlider.setTickInterval(5)#设置刻度间隔
		self.horizontalSlider.valueChanged.connect(self.valChange)

		self.horizontalSlider2 =QSlider(self)
		self.horizontalSlider2.setMaximum(255)
		self.horizontalSlider2.setSingleStep(1)
		self.horizontalSlider2.setPageStep(1)
		self.horizontalSlider2.setProperty("value", 50)
		self.horizontalSlider2.setOrientation(QtCore.Qt.Horizontal)
		self.horizontalSlider2.setObjectName("horizontalSlider2")
		self.horizontalSlider2.setTickInterval(5)#设置刻度间隔
		self.horizontalSlider2.valueChanged.connect(self.valChange2)

		self.horizontalSlider3 =QSlider(self)
		self.horizontalSlider3.setMaximum(300)
		self.horizontalSlider3.setSingleStep(0.1)
		self.horizontalSlider3.setPageStep(0.1)
		self.horizontalSlider3.setProperty("value", 100.0)
		self.horizontalSlider3.setOrientation(QtCore.Qt.Horizontal)
		self.horizontalSlider3.setObjectName("horizontalSlider3")
		self.horizontalSlider3.setTickInterval(0.1)#设置刻度间隔
		self.horizontalSlider3.valueChanged.connect(self.valChange3)

		self.horizontalSlider4 =QSlider(self)
		self.horizontalSlider4.setMaximum(10)
		self.horizontalSlider4.setSingleStep(1)
		self.horizontalSlider4.setPageStep(1)
		self.horizontalSlider4.setProperty("value", 4)
		self.horizontalSlider4.setOrientation(QtCore.Qt.Horizontal)
		self.horizontalSlider4.setObjectName("horizontalSlider4")
		self.horizontalSlider4.setTickInterval(5)#设置刻度间隔
		self.horizontalSlider4.valueChanged.connect(self.valChange4)

		#设置控件属性
		self.label_2.setWordWrap(True)
		self.label_1.setWordWrap(True)
		#设置控件样式
		self.label_display.setStyleSheet("border:2px solid black")
		self.label_display.setAlignment(Qt.AlignLeft)
		self.label_display.setAlignment(Qt.AlignTop)

		#设置控件的尺寸和位置
		self.label_2.resize(50,30)
		self.label_1.resize(50,30)
		self.label_5.resize(55,30)
		self.label_6.resize(50,30)
		self.label_7.resize(55,30)
		self.label_8.resize(50,30)
		self.label_9.resize(50,30)
		self.label_10.resize(50,30)
		self.horizontalSlider.resize(200,25)
		self.horizontalSlider2.resize(200,25)
		self.horizontalSlider3.resize(200,25)
		self.horizontalSlider4.resize(200,25)
		self.button_saveMap.resize(145,30)
		self.button_LQ_OTSU.resize(145,30)
		self.button_OTSU.resize(140,30)
		self.button_th_average.resize(300,30)
		self.button_sobel_motor.resize(300,30)
		self.button_sobel_auto.resize(300,30)
		self.button_th_motor.resize(300,30)
		self.button_start.resize(300,30)
		self.button_clearWall.resize(145,30)
		self.button_filter.resize(300,30)
		self.label_display.resize(300,200)

		# 设置控件位置
		self.label_2.move(1860,260)
		self.label_1.move(1580,260)
		self.label_5.move(1580,335)
		self.label_6.move(1860,335)
		self.label_7.move(1580,410)
		self.label_8.move(1860,410)
		self.label_9.move(1580,555)
		self.label_10.move(1860,555)
		self.horizontalSlider.move(1640,260)
		self.horizontalSlider2.move(1640,335)
		self.horizontalSlider3.move(1640,410)
		self.horizontalSlider4.move(1640,555)
		self.label_display.move(1580,785)
		self.button_start.move(1580,700)
		self.button_clearWall.move(1580+155,735)
		self.button_saveMap.move(1580,735)
		self.button_LQ_OTSU.move(1580+155,480)
		self.button_OTSU.move(1580,480)
		self.button_th_average.move(1580,515)
		self.button_sobel_motor.move(1580,370)
		self.button_sobel_auto.move(1580,445)
		self.button_th_motor.move(1580,295)
		self.button_filter.move(1580,590)
		
		#给控件绑定事件
		self.button_start.clicked.connect(self.button_StartEvent)
		self.button_clearWall.clicked.connect(self.button_Clear)
		self.button_saveMap.clicked.connect(self.button_SaveMap)
		self.button_LQ_OTSU.clicked.connect(self.button_OTSU_LQ)
		self.button_OTSU.clicked.connect(self.button_OTSU_)
		self.button_th_average.clicked.connect(self.button_th_average_)
		self.button_sobel_motor.clicked.connect(self.button_sobel_motor_)
		self.button_sobel_auto.clicked.connect(self.button_sobel_auto_)
		self.button_th_motor.clicked.connect(self.button_th_motor_)
		self.button_filter.clicked.connect(self.button_filter_)
		self.button_start.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_clearWall.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_saveMap.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_LQ_OTSU.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_OTSU.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_th_average.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_sobel_motor.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_sobel_auto.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_th_motor.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_filter.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_start.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		

		# 设置滑动条
		self.s1 = QSlider(Qt.Horizontal,self)
		self.s1.setToolTip("滑动条")
		self.s1.setMinimum(0)#设置最大值
		self.s1.setMaximum(50)#设置最小值
		self.s1.setSingleStep(1)#设置间隔
		self.s1.setValue(0)#设置当前值
		self.s1.sliderMoved.connect(self.start_drag)
		self.s1.sliderReleased.connect(self.drag_action)
		self.s1.setFixedSize(250, 25)
		self.moving_flag = 0
		self.stop_flag = 0  # 如果当前为播放值为0,如果当前为暂停值为1
		self.s1.move(1600,180)
		# 设置两个标签分别是当前时间和结束时间
		self.label_start = QLabel("00:00",self)
		self.label_start.move(1580,195)
		self.label_start.setFont(font)
		self.label_end = QLabel("00:00",self)
		self.label_end.setFont(font)
		self.label_end.move(1580+188+60,195)
		# 设置暂停播放和下一个按钮
		self.stop_button = QPushButton(self)
		self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/开始.png'))    # 设置图标
		self.stop_button.setIconSize(QSize(30,30))
		self.stop_button.clicked.connect(self.stop_action)
		self.stop_button.move(1580,220)
		self.next_button = QPushButton(self)
		self.next_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/下一个.png'))   # 设置图标
		self.next_button.setIconSize(QSize(30,30))
		self.next_button.clicked.connect(self.next_action)
		self.next_button.move(1580+188+10,220)
		# UI初始化完成
		self.setGeometry(0, 0, 1920, 1080)
		self.setStyleSheet("background-color:#EDEDF5")   # 设置背景色
		self.setWindowTitle('二值化工具')
		# self.show()

	# 获取滑动条的数值函数	
	def valChange(self):
		self.label_2.setNum(self.horizontalSlider.value())

	def valChange2(self):
		self.label_6.setNum(self.horizontalSlider2.value())

	def valChange3(self):
		num=self.horizontalSlider3.value()/10
		self.label_8.setNum(num)

	def valChange4(self):
		self.label_10.setNum(self.horizontalSlider4.value())

	# 打开资源管理器对话框，输入图片
	def openimage(self):
		imgName, imgType = QFileDialog.getOpenFileName(self, "打开图片", "", "*.bmp;;*.jpg;;*.png;;All Files(*)")
		jpg = QtGui.QPixmap(imgName).scaled(self.label.width(), self.label.height())
		self.label.setPixmap(jpg)  # 将图片加载到label上

	# debug窗口
	def addDisplayText(self,text):
		if self.displayFlush:
			self.label_display.setText(text+'\n')
			self.displayFlush=False
		else:
			self.label_display.setText(self.label_display.toPlainText()+text+'\n')

	# 此类没啥用
	def mousePressEvent(self,event):
		x,y=event.x()-50,event.y()-50
		x=x//config.blockLength
		y=y//config.blockLength
		if x>=0 and x<config.WIDTH and y>=0 and y<config.HEIGHT:
            # 左键按下添加墙壁
			if event.button()==Qt.LeftButton:
				if (x,y)!=self.startPoint and (x,y)!=self.endPoint:
					self.Map[y][x]=(1 if self.Map[y][x]==0 else 0)
            # 邮件按下第一次添加起点，第二次终点
			if event.button()==Qt.RightButton:
				if self.Map[y][x]==0:
					if self.startPoint==None:
						self.startPoint=(x,y)
						self.addDisplayText('添加了一个起点:(%d,%d)'%(x,y))
					elif self.endPoint==None and self.startPoint!=(x,y):
						self.endPoint=(x,y)
						self.addDisplayText('添加了一个终点:(%d,%d)'%(x,y))
			self.repaint()

	# 巡线，静态图片调试边线
	def button_StartEvent(self):
		# 获取选项框情况
		type=self.checkRadioButton()
		global aa,bb,bc,side,md
		for i in range(120):
			for j in range(2):
				bb[i][j]=0	
		if(type==1):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			# 转化为可以处理的数组
			frame = np.array(gray, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_uint8*(120*188+1)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),4)
			# 将输出量转化为py中的list，存储在aa中
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					if(aa[i][m]==1):
						aa[i][m]=0
					m=m+1
			# 在label中显示阈值
			self.label_4.setText(str(t1[120*188]))
			# 初始化用于判断边线的数组，存储在md中 64*188
			m=0
			for i in range(56,120):
				d=0
				for j in range(188):
					md[m][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*64*3
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_line_LMR.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			for i in range(3):
				m=0
				for j in range(64*i,(i+1)*64):
					side[m][i]=t1[j]
					m=m+1		
		if(type==2):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			# 转化为可以处理的数组
			frame = np.array(gray, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_uint8*(120*188+1)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),0)
			# 将输出量转化为py中的list，存储在aa中
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					if(aa[i][m]==1):
						aa[i][m]=0
					m=m+1
			# 在label中显示阈值
			self.label_4.setText(str(t1[120*188]))
			# 初始化用于判断边线的数组，存储在md中 64*188
			m=0
			for i in range(56,120):
				d=0
				for j in range(188):
					md[m][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*64*3
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_line_LMR.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			for i in range(3):
				m=0
				for j in range(64*i,(i+1)*64):
					side[m][i]=t1[j]
					m=m+1	
		if(type==3):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)

			# 固定阈值进行二值化
			binary = threshold_demo(mat_img,self.horizontalSlider.value())
			# 在label中显示阈值
			self.label_4.setText(str(self.horizontalSlider.value()))
			# 将输出量转化为py中的list，存储在aa中
			aa = binary.tolist()
			# 初始化用于判断边线的数组，存储在md中 64*188
			m=0
			for i in range(56,120):
				d=0
				for j in range(188):
					md[m][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*64*3
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_line_LMR.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			for i in range(3):
				m=0
				for j in range(64*i,(i+1)*64):
					side[m][i]=t1[j]
					m=m+1	
		if(type==4):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			#将图片数据转换为可处理的整型数据
			frame=gray.tolist
			frame = np.array(gray, dtype=np.float)

			# 为方便调参，调用py编写的函数
			sobel_motor_py(frame,aa,self.horizontalSlider2.value())
			# 初始化用于判断边线的数组，存储在md中 64*188
			m=0
			for i in range(56,120):
				d=0
				for j in range(188):
					md[m][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*64*3
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_line_LMR.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			for i in range(3):
				m=0
				for j in range(64*i,(i+1)*64):
					side[m][i]=t1[j]
					m=m+1	
		if(type==5):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			# 将图片数据转换为可处理的整型数据
			frame=gray.tolist
			frame = np.array(gray, dtype=np.float)
			# 为方便调参，调用py编写的函数
			sobel_auto_py(frame,aa,self.horizontalSlider3.value()/10)
			# 初始化用于判断边线的数组，存储在md中 64*188
			m=0
			for i in range(56,120):
				d=0
				for j in range(188):
					md[m][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*64*3
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_line_LMR.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			for i in range(3):
				m=0
				for j in range(64*i,(i+1)*64):
					side[m][i]=t1[j]
					m=m+1		
		if(type==6):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_uint8*(120*188+1)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),1)
			# 将输出量转化为py中的list，存储在aa中
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					m=m+1
			# 在label中显示阈值
			self.label_4.setText(str(t1[120*188]))
			# 初始化用于判断边线的数组，存储在md中 64*188
			m=0
			for i in range(56,120):
				d=0
				for j in range(188):
					md[m][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*64*3
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_line_LMR.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			for i in range(3):
				m=0
				for j in range(64*i,(i+1)*64):
					side[m][i]=t1[j]
					m=m+1	
		# 滤波，未开启
		# filter_py(aa,bc,self.horizontalSlider4.value()*255)
		# bc,bb=ImageGetSide(bc,bb)
        
		# 输出信息到绘图区
		dk=56
		# 二值化图像
		for i in range(64):
			for j in range(188):
				self.Map[i][j]=aa[i][j]
		for i in range(64):
			for j in range(188):
				md[i][j]=aa[dk][j]
			dk=dk+1
		# 边线信息
		for i in range(64):
			md[i][side[i][0]]=100 #左边线
			md[i][side[i][2]]=200 #右边线
			md[i][side[i][1]]=150 #中线
		for i in range(188):
			md[ROAD_MAIN_ROW][i]=50 #主跑行
		zz=0
		dd=0
		for i in range(56,120):
			dd=0
			for j in range(188):
				self.Map[i][j]=md[zz][dd]
				dd=dd+1
			zz=zz+1
		config.HEIGHT=len(self.Map)
		config.WIDTH=len(self.Map[0])
		self.repaint()
		self.addDisplayText('开始边线规划')

	# 数据保存，可用快捷键ctrl+s
	def button_SaveMap(self):
		# 获取目前的参数值
		ctr.th=self.horizontalSlider.value()
		ctr.sobel_motor=self.horizontalSlider2.value()
		ctr.sobel_auto=self.horizontalSlider3.value()/10
		ctr.filter=self.horizontalSlider4.value()
		# 保存参数
		ctr.save_data()
		# 获取当前时间戳
		time = datetime.datetime.now()
		time1_str = datetime.datetime.strftime(time,'%Y-%m-%d %H:%M:%S')
		self.addDisplayText('参数保存成功,%s'%time1_str)

	# OTSU二值化
	def button_OTSU_(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),4)
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('OTSU')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')

	# 平均阈值二值化
	def button_th_average_(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),1)
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('平均阈值')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')

	# sobel手动阈值二值化
	def button_sobel_motor_(self):
		try:
			# 为了方便gui调参用python写了一遍
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame=gray.tolist
			frame = np.array(gray, dtype=np.float)
			sobel_motor_py(frame,aa,self.horizontalSlider2.value())
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('sobel算子手动阈值')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')
			# c动态链接库直接导入函数求解
			'''
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),2)
			aa=[]
			m=0
			for i in range(120):
				aa.append([])
				for j in range(188*i,(i+1)*188):
					aa[i].append(t1[j])
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('地图加载成功')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')
				'''

	# sobel自动阈值二值化
	def button_sobel_auto_(self):
		try:
			# 为了方便gui调参用python写了一遍
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame=gray.tolist
			frame = np.array(gray, dtype=np.float)
			sobel_auto_py(frame,aa,self.horizontalSlider3.value()/10)
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('sobel算子自动阈值')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')
			# c动态链接库直接导入函数求解
			'''
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),3)
			aa=[]
			m=0
			for i in range(120):
				aa.append([])
				for j in range(188*i,(i+1)*188):
					aa[i].append(t1[j])
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('地图加载成功')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')
				'''

	# 龙邱OTSU二值化
	def button_OTSU_LQ(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),0)
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('龙邱OTSU')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')

	# 固定阈值二值化
	def button_th_motor_(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			binary = threshold_demo(mat_img,self.horizontalSlider.value())
			self.label_4.setText(str(self.horizontalSlider.value()))
			# numpy中ndarray文件转为list
			img_list = binary.tolist()
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(img_list))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('手动阈值')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')

	# 图像滤波
	def button_filter_(self):
		# 为方便调参，调用py编写的函数
		filter_py(aa,bc,self.horizontalSlider4.value()*255)
		with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
			f.write(json.dumps(bc))
		with open('C:/Users/17628/Desktop/map2.txt','r') as f:
			self.Map=json.loads(f.read())
			config.HEIGHT=len(self.Map)
			config.WIDTH=len(self.Map[0])
			self.addDisplayText('进行滤波')
			self.repaint()
	
	# 绘图区清除
	def button_Clear(self):
		sender=self.sender()
		print(self.button_clearSE,type(self.button_clearSE))
		if sender==self.button_clearSE:
			self.startPoint=None
			self.endPoint=None
			self.repaint()
			self.addDisplayText('清空起始点')
		elif sender==self.button_clearWall:
			for i in range(len(self.Map)):
				for j in range(len(self.Map[i])):
					self.Map[i][j]=0
			self.repaint()
			self.addDisplayText('清空所有图像')
	
	# 重写绘图类
	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self.drawBoard(event,qp)
		qp.end()
    
	# 重写绘图类
	def drawBoard(self, event, qp):
		self.drawMap(qp)

	# 绘图区颜色设置
	def drawMap(self,qp):#画面绘制方法，每次地图有所改动都将重绘
		time1=time.time()
		if self.search!=None:
			if self.special!=None:
				path = None
				if self.special[0]!=None:
					e=self.special[0]
					path=[e]
					while True:
						e=e.father
						if e!=None:
							path.append(e)
						else:
							break
			else:
				path=None
			pen=QPen(QColor(0,0,0),1,Qt.SolidLine)
			qp.setPen(pen)
			for i in range(len(self.Map)):
				for j in range(len(self.Map[i])):
					wordTag=False
					if i==self.search.start.x and j==self.search.start.y:   # 起始点颜色
						qp.setBrush(QColor(255,255,0))
					elif i==self.search.end.x and j==self.search.end.y:     # 终止点颜色
						qp.setBrush(QColor(100,200,50))
					else:
						if self.Map[i][j]==255:                             # 非障碍物颜色
							qp.setBrush(QColor(255, 255, 255))
						elif self.Map[i][j]==1 or self.Map[i][j]==0:                               #障碍物颜色
							qp.setBrush(QColor(0,0,0))
						elif self.Map[i][j]==100:                           # 左边线
							qp.setBrush(QColor(255,0,0))
						elif self.Map[i][j]==200:                           # 右边线
							qp.setBrush(QColor(0,255,0))
						elif self.Map[i][j]==150:                           # 中线
							qp.setBrush(QColor(0,0,255))
						else:
							qp.setBrush(QColor(200,200,230))
					qp.drawRect(30+j*config.blockLength,30+i*config.blockLength,config.blockLength,config.blockLength)
					if wordTag:
						qp.setFont(QFont('楷体',5,QFont.Light))
						qp.drawText(50+10+j*config.blockLength,50+10+i*config.blockLength,word)
						wordTag=False
		#time.sleep(20)
		else:
			for i in range(len(self.Map)):
				for j in range(len(self.Map[i])):
					if (j,i)==self.startPoint:
						qp.setBrush(QColor(255,255,0))
					elif (j,i)==self.endPoint:
						qp.setBrush(QColor(100,200,50))
					else:
						if self.Map[i][j]==255:
							qp.setBrush(QColor(255,255,255))
						elif self.Map[i][j]==1 or self.Map[i][j]==0:
							qp.setBrush(QColor(0,0,0))
						elif self.Map[i][j]==100:                           # 左边线
							qp.setBrush(QColor(255,0,0))
						elif self.Map[i][j]==200:                           # 右边线
							qp.setBrush(QColor(0,255,0))
						elif self.Map[i][j]==150:                           # 中线
							qp.setBrush(QColor(0,0,255))
						elif self.Map[i][j]==50:                            # 主跑行
							qp.setBrush(QColor(162,20,203))
						elif self.Map[i][j]==90:                            # 跳变点
							qp.setBrush(QColor(4,211,252))
						else:
							qp.setBrush(QColor(200,200,230))

					qp.drawRect(30+j*config.blockLength,30+i*config.blockLength,config.blockLength,config.blockLength)
		time2=time.time()

	# 未用到
	def timerEvent(self,e):
		try:
			data=next(self.yi)
		except Exception as e:
			self.addDisplayText('搜索结束:')
			print('搜索结束！')
			if self.search.result==None:
				self.addDisplayText('未找到可行路径')
				print('搜索结束！')
			else:
				self.addDisplayText('总计搜索节点数：%d'%self.search.count)
				self.addDisplayText('最终路径长度：%d'%len(self.search.result))
			self.centerTimer.stop()
			self.search=None
			self.yi=None
			self.special=None
			point.clear()
			self.button_start.setEnabled(True)
			self.button_clearSE.setEnabled(True)
			self.button_clearWall.setEnabled(True)
			self.displayFlush=True
		else:
			self.special=data
			self.repaint()

	# 定义将opencv图像转PyQt图像的函数
	def cvImgtoQtImg(self,cvImg):
		QtImgBuf = cv2.cvtColor(cvImg, cv2.COLOR_BGR2BGRA)
		QtImg = QtGui.QImage(QtImgBuf.data, QtImgBuf.shape[1], QtImgBuf.shape[0],QtGui.QImage.Format_RGB32)
		return QtImg

	# 选项框警告
	def checkRadioButton(self):
		if self.radioButton.isChecked():
			type = 1
		if self.radioButton2.isChecked():
			type = 2
		if self.radioButton3.isChecked():
			type = 3
		if self.radioButton4.isChecked():
			type = 4
		if self.radioButton5.isChecked():
			type = 5
		if self.radioButton6.isChecked():
			type = 6
		if not self.radioButton.isChecked() and not self.radioButton2.isChecked() and not self.radioButton3.isChecked() and not self.radioButton4.isChecked() and not self.radioButton5.isChecked() and not self.radioButton6.isChecked():
			QMessageBox.information(self, "消息框标题", "请选择一种方法", QMessageBox.Yes | QMessageBox.No)
			type=1
		return type

	# 播放视频进行二值化调试，与图片的思路基本一致
	def playVideoFile(self,fn):
		# 获取选项框type
		type=self.checkRadioButton()
		self.cap = cv2.VideoCapture(fn)
		# 设置滚动条对应的帧
		frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
		self.settingSlider(frames)
		# 帧数
		fps = 24
		self.loop_flag = 0
		if not self.cap.isOpened():
			print("Cannot open Video File")
			exit()
		while not self.bClose:
			# 逐帧读取影片
			ret, frame = self.cap.read()
			if not ret:
				if frame is None:
					print("The video has end.")
				else:
					print("Read video error!")
				break
			if self.moving_flag==0:
				# 设置时间
				self.label_start.setText(self.int2time(self.loop_flag))
				self.s1.setValue(int(self.loop_flag/fps))#设置当前值
			self.loop_flag += 1
			# 将cv2读取的图像显示在label中
			QtImg = self.cvImgtoQtImg(frame)
			self.label.setPixmap(QtGui.QPixmap.fromImage(QtImg).scaled(self.label.size()))
			self.label.show()  # 刷新界面
			global aa,bb,bc,side,md
			# 视频和图片的处理手法一致
			if(type==1):
				# 获取label上的图像，并进行一些数据转换
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				frame = np.array(gray, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_uint8*(120*188+1)
				t=a()
				mylib.Get_01_Value.restype=POINTER(c_uint8)
				t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),4)
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						aa[i][m]=t1[j]
						if(aa[i][m]==1):
							aa[i][m]=0
						m=m+1
				self.label_4.setText(str(t1[120*188]))
				m=0
				for i in range(56,120):
					d=0
					for j in range(188):
						md[m][d]=aa[i][j]
						d=d+1
					m=m+1
				frame = np.array(md, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_int8*64*3
				t=a()
				mylib.Get_line_LMR.restype=POINTER(c_uint8)
				t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				for i in range(3):
					m=0
					for j in range(64*i,(i+1)*64):
						side[m][i]=t1[j]
						m=m+1	
			if(type==2):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				frame = np.array(gray, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_uint8*(120*188+1)
				t=a()
				mylib.Get_01_Value.restype=POINTER(c_uint8)
				t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),0)
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						aa[i][m]=t1[j]
						if(aa[i][m]==1):
							aa[i][m]=0
						m=m+1
				self.label_4.setText(str(t1[120*188]))
				m=0
				for i in range(56,120):
					d=0
					for j in range(188):
						md[m][d]=aa[i][j]
						d=d+1
					m=m+1
				frame = np.array(md, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_int8*64*3
				t=a()
				mylib.Get_line_LMR.restype=POINTER(c_uint8)
				t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				for i in range(3):
					m=0
					for j in range(64*i,(i+1)*64):
						side[m][i]=t1[j]
						m=m+1
			if(type==3):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				binary = threshold_demo(mat_img,self.horizontalSlider.value())
				self.label_4.setText(str(self.horizontalSlider.value()))
				aa = binary.tolist()
				m=0
				for i in range(56,120):
					d=0
					for j in range(188):
						md[m][d]=aa[i][j]
						d=d+1
					m=m+1
				frame = np.array(md, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_int8*64*3
				t=a()
				mylib.Get_line_LMR.restype=POINTER(c_uint8)
				t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				
				for i in range(3):
					m=0
					for j in range(64*i,(i+1)*64):
						side[m][i]=t1[j]
						m=m+1	
			if(type==4):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				#将图片数据转换为可处理的整型数据
				frame=gray.tolist
				frame = np.array(gray, dtype=np.float)
				sobel_motor_py(frame,aa,self.horizontalSlider2.value())
				m=0
				for i in range(56,120):
					d=0
					for j in range(188):
						md[m][d]=aa[i][j]
						d=d+1
					m=m+1
				frame = np.array(md, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_int8*64*3
				t=a()
				mylib.Get_line_LMR.restype=POINTER(c_uint8)
				t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				
				for i in range(3):
					m=0
					for j in range(64*i,(i+1)*64):
						side[m][i]=t1[j]
						m=m+1	
			if(type==5):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				frame=gray.tolist
				frame = np.array(gray, dtype=np.float)
				sobel_auto_py(frame,aa,self.horizontalSlider3.value()/10)
				m=0
				for i in range(56,120):
					d=0
					for j in range(188):
						md[m][d]=aa[i][j]
						d=d+1
					m=m+1
				frame = np.array(md, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_int8*64*3
				t=a()
				mylib.Get_line_LMR.restype=POINTER(c_uint8)
				t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				
				for i in range(3):
					m=0
					for j in range(64*i,(i+1)*64):
						side[m][i]=t1[j]
						m=m+1		
			if(type==6):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				frame = np.array(gray, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_uint8*(120*188+1)
				t=a()
				mylib.Get_01_Value.restype=POINTER(c_uint8)
				t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),1)
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						aa[i][m]=t1[j]
						m=m+1
				self.label_4.setText(str(t1[120*188]))
				m=0
				for i in range(56,120):
					d=0
					for j in range(188):
						md[m][d]=aa[i][j]
						d=d+1
					m=m+1
				frame = np.array(md, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_int8*64*3
				t=a()
				mylib.Get_line_LMR.restype=POINTER(c_uint8)
				t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				
				for i in range(3):
					m=0
					for j in range(64*i,(i+1)*64):
						side[m][i]=t1[j]
						m=m+1	
			
			dk=56
			for i in range(64):
				for j in range(188):
					self.Map[i][j]=aa[i][j]
			for i in range(64):
				for j in range(188):
					md[i][j]=aa[dk][j]
				dk=dk+1
			for i in range(64):
				md[i][side[i][0]]=100 #左边线
				md[i][side[i][2]]=200 #右边线
				md[i][side[i][1]]=150 #中线
			for i in range(188):
				md[ROAD_MAIN_ROW][i]=50 #主跑行
			zz=0
			dd=0
			for i in range(56,120):
				dd=0
				for j in range(188):
					self.Map[i][j]=md[zz][dd]
					dd=dd+1
				zz=zz+1
			config.HEIGHT=len(self.Map)
			config.WIDTH=len(self.Map[0])
			self.repaint()

			# for i in range(64):
			# 	for j in range(188):
			# 		md[i][j]=255
			# for i in range(64):
			# 	md[i][side[i][0]]=100 #左边线
			# 	md[i][side[i][2]]=200 #右边线
			# 	md[i][side[i][1]]=150 #中线
			# for i in range(188):
			# 	md[ROAD_MAIN_ROW][i]=50 #主跑行
			# zz=0
			# dd=0
			# for i in range(56,120):
			# 	dd=0
			# 	for j in range(188):
			# 		self.Map[i][j]=md[zz][dd]
			# 		dd=dd+1
			# 	zz=zz+1
			# config.HEIGHT=len(self.Map)
			# config.WIDTH=len(self.Map[0])
			# self.repaint()
			# filter_py(aa,bc,self.horizontalSlider4.value()*255)	
			# # numpy中ndarray文件转为list
			# # img_list = binary.tolist()
			# for i in range(MT9V03X_H):
			# 	for j in range(MT9V03X_W):
			# 		self.Map[i][j]=bc[i][j]
			# config.HEIGHT=len(self.Map)
			# config.WIDTH=len(self.Map[0])
			# self.repaint()


			# for i in range(120):
			# 	for j in range(188):
			# 		bianxian[i][j]=0  #白色
			# for i in range(120):
			# 	# bianxian[i*3][int((aa[i][0]+aa[i][1])/2)*3]=255#中线
			# 	bianxian[i][aa[i][0]]=255#左边线
			# 	# bianxian[i*3][int(aa[i][1])*3]=255#右边线
			# # for i in range(119):
			# # 	bianxian[i*3+1][int((aa[i][0]+aa[i][1])/2)*3]=255 #中线
			# # for i in range(119):
			# # 	bianxian[i*3+2][int((aa[i][0]+aa[i][1])/2)*3]=255 #中线
			# for i in range(188):
			# 	bianxian[ROAD_MAIN_ROW][i]=255 #主跑行
			# bianxian = Image.fromarray(np.uint8(bianxian))
			# bianxian = bianxian.toqpixmap() #QPixmap
			# self.label_14.setPixmap(bianxian)
			# self.label_14.show()#刷新界面

			while self.stop_flag == 1:#暂停的动作
				cv2.waitKey(int(100/fps))#休眠一会，因为每秒播放24张图片，相当于放完一张图片后等待41ms
			cv2.waitKey(int(100/fps))#休眠一会，因为每秒播放24张图片，相当于放完一张图片后等待41ms
        #释放
		self.cap.release()

	# 暂停触发
	def stop_action(self):
		if self.stop_flag == 0:
			self.stop_flag = 1
			self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/暂停.png'))
		else:
			self.stop_flag = 0
			self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/开始.png'))

	# 下一个触发
	def next_action(self):
		self.bClose = True
		self.play_index = (self.play_index+1)%3
		self.bClose = False
		self.playVideoFile(self.fn_list[self.play_index])

	# 拖动触发
	def start_drag(self):
		self.moving_flag = 1

	# 拖动进度条行为
	def drag_action(self):
		self.moving_flag = 0
		print('当前进度为%d，被拉动到的进度为%d'%(self.s1.value(), int(self.loop_flag/24)))
		if self.s1.value()!=int(self.loop_flag/24):
			print('当前进度为:'+str(self.s1.value()))
			self.loop_flag = self.s1.value()*24
			self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.loop_flag)

	# 暂停触发
	def video_stop(self):
		self.bClose = True

	# 手动选择视频播放
	def manual_choose(self):
		fn,_ = QFileDialog.getOpenFileName(self,'Open file','C:/Users/17628/Desktop/ui/test video',"Video files (*.mp4 *.avi)")
		self.playVideoFile(fn)

	# 自动选择视频播放，如何自动视频播放？
	def auto_choose(self):
		self.playVideoFile(self.fn_list[self.play_index])
    
	# 设置进度条
	def settingSlider(self,maxvalue):
		self.s1.setMaximum(int(maxvalue/24))
		self.label_end.setText(self.int2time(maxvalue))

	# 视频播放的时间函数
	def int2time(self,num):
        #每秒刷新50帧
		num = int(num/24)
		minute = int(num/60)
		second = num - 60*minute
		if minute < 10:
			str_minute = '0'+str(minute)
		else:
			tr_minute = str(minute)
		if second < 10:
			str_second = '0'+str(second)
		else:
			str_second = str(second)
		return str_minute+":"+str_second

	# 停止行为
	def exit_stop(self):
		self.stop_flag = 0
		self.bClose = True
		self.close()

# 运行监测工具可视化类
class WinForm(QGraphicsView,QMainWindow):
	# 视频播放列表
	bClose = False
	fn_list = ["C:/Users/17628/Desktop/ui/test video/慢速版.mp4","C:/Users/17628/Desktop/ui/test video/正常版.mp4"]
	play_index = 0
	def __init__(self, parent=None):
		super(WinForm, self).__init__(parent)
		print("初始化运行监测工具...")
		self.setWindowTitle('运行监测工具')  # 设置窗口标题
		# self.setWindowTitle("参数跟踪")
		self.setStyleSheet("background-color:#EDEDF5")
		self.setGeometry(0, 0, 1920, 1080)  # 窗口整体窗口位置大小

		# quit = QPushButton('click', self)  # button 对象
		# quit.setGeometry(10, 30, 60, 35)  # 设置按钮的位置 和 大小
		# quit.setStyleSheet("background-color: red")  # 设置按钮的风格和颜色
		# quit.clicked.connect(self.clickbutton)  # 点击按钮之后关闭窗口

		self.scene = QGraphicsScene(self)
		self.scene.setSceneRect(0, 0, 1920, 1080)  # 动画 位置 这个需计算

		# 理论转角区
		self.rota = Rotation()
		self.scene.addItem(self.rota.car_back_item)

		self.rota_top = Rotation_top()
		self.scene.addItem(self.rota_top.car_top_item)

		self.rota_tyre = Rotation_tyre()
		self.scene.addItem(self.rota_tyre.car_tyre_item)

		# 实际转角区
		self.rota_real = Rotation_real()
		self.scene.addItem(self.rota_real.car_back_real_item)

		self.rota_top_real = Rotation_top_real()
		self.scene.addItem(self.rota_top_real.car_top_real_item)

		self.rota_tyre_real = Rotation_tyre_real()
		self.scene.addItem(self.rota_tyre_real.car_tyre_real_item)

		# 设定动画坐标轴
		self.setScene(self.scene)
		self.stop = False

		self.label_13 = QLabel(self)
		self.label_13.setText("显示图片")
		self.label_13.setFixedSize(188*3, 120*3)
		self.label_13.move(10, 100)  #10  485

		self.label_14 = QLabel(self)
		self.label_14.setText("显示图片")
		self.label_14.setFixedSize(188*3, 120*3)
		self.label_14.move(564+15, 100)  #485

		# 设置字体类
		font = QtGui.QFont()
		font.setFamily("Consolas")
		font.setPointSize(12)
		font.setBold(False)
		font.setItalic(False)
		font.setWeight(3)

		font2 = QtGui.QFont()
		font2.setFamily("Consolas")
		font2.setPointSize(24)
		font2.setBold(False)
		font2.setItalic(False)
		font2.setWeight(3)

		self.label_1 = QLabel(self)
		self.label_1.setText("舵机转角理论值：            度")
		self.label_1.setFixedSize(300, 40)
		self.label_1.move(1550, 30)
		self.label_1.setFont(font)

		self.label_7 = QLabel(self)
		self.label_7.setText("40")
		self.label_7.setFixedSize(300, 40)
		self.label_7.move(1680, 30)
		self.label_7.setFont(font2)

		self.label_2 = QLabel(self)
		self.label_2.setText("舵机转角实际值：            度")
		self.label_2.setFixedSize(300, 40)
		self.label_2.move(1175, 30)
		self.label_2.setFont(font)

		self.label_8 = QLabel(self)
		self.label_8.setText("40")
		self.label_8.setFixedSize(150, 40)
		self.label_8.move(1305, 30)
		self.label_8.setFont(font2)

		self.label_3 = QLabel(self)
		self.label_3.setText("倾角实际值：            度")
		self.label_3.setFixedSize(300, 40)
		self.label_3.move(1175, 295)
		self.label_3.setFont(font)

		self.label_9 = QLabel(self)
		self.label_9.setText("40")
		self.label_9.setFixedSize(150, 40)
		self.label_9.move(1275, 295)
		self.label_9.setFont(font2)

		self.label_4 = QLabel(self)
		self.label_4.setText("倾角理论值：            度")
		self.label_4.setFixedSize(300, 40)
		self.label_4.move(1550, 295)
		self.label_4.setFont(font)

		self.label_10 = QLabel(self)
		self.label_10.setText("40")
		self.label_10.setFixedSize(150, 40)
		self.label_10.move(1650, 295)
		self.label_10.setFont(font2)

		self.label_5 = QLabel(self)
		self.label_5.setText("转速实际值：            m/s")
		self.label_5.setFixedSize(300, 40)
		self.label_5.move(1175, 695)
		self.label_5.setFont(font)

		self.label_11 = QLabel(self)
		self.label_11.setText("40")
		self.label_11.setFixedSize(150, 40)
		self.label_11.move(1275, 695)
		self.label_11.setFont(font2)

		self.label_6 = QLabel(self)
		self.label_6.setText("转速理论值：            m/s")
		self.label_6.setFixedSize(300, 40)
		self.label_6.move(1550, 695)
		self.label_6.setFont(font)

		self.label_12 = QLabel(self)
		self.label_12.setText("40")
		self.label_12.setFixedSize(150, 40)
		self.label_12.move(1650, 695)
		self.label_12.setFont(font2)

		# 建立滚动条布局
		self.topFiller = QWidget()
		self.topFiller.setMinimumSize(188*3, 2000)  # 设置滚动条的尺寸

		# 设置基本参数列表，初始化
		global label_canshu_value
		label_canshu=["环岛标志位","大环岛标志位","环岛状态切换延时","左边线黑点寻找标志位","右边线黑点寻找标志位","近处处理标志","十字标志位","出界标志位","坡道标志位","左丢单边线(点)次数","右丢单边线(点)次数","左右同时丢边线(点)次数","十字丢线次数","图像中线"]
		label_canshu_e=["Island_flag","Big_island_flag","Island_change_time","Found_left_flag","Found_right_flag","Near_flag","Cross_road_flag","out_flag","Ram_flag","L_lost_cnt","R_lost_cnt","L_R_lost_cnt","Cross_road_cnt","Middle_line"]
		label_canshu_value=[]
		for i in range(len(label_canshu)):
			num = i
			var = 'v' + str(num)
			label_canshu_value.append(var)

		# 动态生成基本参数列表的label，同时设定对应的布局
		for i in range(len(label_canshu)):
			self.label1000=QLabel(self.topFiller)
			self.label1000.setText(label_canshu[i])
			self.label1000.move(20,50+i*30)
			self.label1000.setFont(font)

			self.label2000=QLabel(self.topFiller)
			self.label2000.setText(label_canshu_e[i])
			self.label2000.move(240,50+i*30)
			self.label2000.setFont(font)

			self.label3000=QLabel(self.topFiller)
			self.label3000.setObjectName(label_canshu_value[i])
			self.label3000.setText("00000")
			self.label3000.move(470,50+i*30)
			self.label3000.setFont(font)

		# 基本参数列表头
		self.label17=QLabel(self.topFiller)
		self.label17.setText("基本参数                  变量名                   参数值\n--------------------------------------------------------")
		self.label17.setStyleSheet("color:#F46F0C")
		self.label17.setFont(font)
		self.label17.resize(188*3,50)
		self.label17.move(20,0)

		# 将布局放置在滚动条中
		self.scrol = QScrollArea(self)
		self.scrol.setWidget(self.topFiller)
		self.scrol.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.scrol.resize(188*3,430)
		self.scrol.move(188*3+15,485)

		# 设置滚动条布局
		self.topFiller2 = QWidget()
		self.topFiller2.setMinimumSize(188*3, 3670)  # 设置滚动条的尺寸

		# 边线列表头
		self.label18=QLabel(self.topFiller2)
		self.label18.setText("行数            左边线            中线            右边线\n--------------------------------------------------------")
		self.label18.setStyleSheet("color:#F46F0C")
		self.label18.resize(188*3,50)
		self.label18.move(20,0)
		self.label18.setFont(font)

		# 动态生成边线行数label，同时设定对应的布局
		for filename in range(120):
			self.label20 = QLabel(self.topFiller2)
			self.label20.setText(str(filename))
			self.label20.move(20,filename*30+50)
			self.label20.setFont(font)
		
		# 设置左中右边线参数列表，初始化
		global labelLeft,labelRight,labelMid
		labelLeft = []
		for i in range(120):
			num = i
			var = 'L' + str(num)
			labelLeft.append(var)
		labelMid = []
		for i in range(120):
			num = i
			var = 'M' + str(num)
			labelMid.append(var)
		labelRight = []
		for i in range(120):
			num = i
			var = 'R' + str(num)
			labelRight.append(var)

		# 动态生成左中右边线列表的label，同时设定对应的布局
		for i in range(120):
			self.label100=QLabel(self.topFiller2)
			self.label100.setObjectName(labelLeft[i])
			self.label100.setText("000")
			self.label100.move(165,50+i*30)
			self.label100.setFont(font)
			self.label100.setStyleSheet("color:#2A78D6")

			self.label200=QLabel(self.topFiller2)
			self.label200.setObjectName(labelMid[i])
			self.label200.setText("000")
			self.label200.move(165+155,50+i*30)
			self.label200.setFont(font)
			self.label200.setStyleSheet("color:#782AD6")

			self.label300=QLabel(self.topFiller2)
			self.label300.setObjectName(labelRight[i])
			self.label300.setText("000")
			self.label300.move(165+150+150,50+i*30)
			self.label300.setFont(font)
			self.label300.setStyleSheet("color:#45C937")

		# 将布局放置在滚动条中
		self.scrol2 = QScrollArea(self)
		self.scrol2.setWidget(self.topFiller2)
		self.scrol2.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.scrol2.resize(188*3,430)
		self.scrol2.move(10,485)

		self.btn = QPushButton(self)
		self.btn.setText("打开图片")
		self.btn.move(10, 30)
		self.btn.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
											"QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
											"QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.btn.clicked.connect(self.openimage)
		self.btn.resize(120,40)
		self.btn.setFont(font)

		self.btn2 = QPushButton(self)
		self.btn2.setText("打开视频")
		self.btn2.move(135, 30)
		self.btn2.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
											"QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
											"QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.btn2.clicked.connect(self.manual_choose)
		self.btn2.resize(120,40)
		self.btn2.setFont(font)

		self.btn1 = QPushButton(self)
		self.btn1.setText("计算角度")
		self.btn1.move(260, 30)
		self.btn1.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
											"QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
											"QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.btn1.clicked.connect(self.startcal)
		self.btn1.resize(120,40)
		self.btn1.setFont(font)

		self.screen = QDesktopWidget().screenGeometry()
		self.resize(self.screen.width(), self.screen.height())

		# 设置菜单栏
		bar = QMenuBar(self)
		bar.setFixedSize(self.screen.width(),24)
		# 设置选项二
		play_menu = bar.addMenu("视频调试")
		# 设置按钮一
		play_video = QAction("打开视频文件",self)
		play_video.setShortcut("Ctrl+1")
		play_video.triggered.connect(self.manual_choose)
		play_menu.addAction(play_video)
		# 设置按钮二
		play_pictures = QAction("自动播放视频",self)
		play_pictures.setShortcut("Ctrl+2")
		play_pictures.triggered.connect(self.auto_choose)
		play_menu.addAction(play_pictures)
		# 设置停止按钮
		play_stop = QAction("停止",self)
		play_stop.setShortcut("Ctrl+3")
		play_stop.triggered.connect(self.video_stop)
		play_menu.addAction(play_stop)
		# 设置退出按钮
		exit_menu = bar.addMenu("图片调试")
		exit_option = QAction("打开图片文件",self)
		exit_option.setShortcut("Ctrl+Q")
		exit_option.triggered.connect(self.openimage)
		exit_menu.addAction(exit_option)
		# 设置菜单栏的位置
		bar.move(0,0)

		# 设置滑动条
		self.s1 = QSlider(Qt.Horizontal,self)
		self.s1.setToolTip("滑动条")
		self.s1.setMinimum(0)  # 设置最大值
		self.s1.setMaximum(50)  # 设置最小值
		self.s1.setSingleStep(1)  # 设置间隔
		self.s1.setValue(0)  # 设置当前值
		self.s1.sliderMoved.connect(self.start_drag)
		self.s1.sliderReleased.connect(self.drag_action)
		self.s1.setFixedSize(564, 30)
		self.moving_flag = 0
		self.stop_flag = 0  # 如果当前为播放值为0,如果当前为暂停值为1
		self.s1.move(10,485+430+20)
		# 设置两个标签分别是当前时间和结束时间
		self.label_start = QLabel("00:00",self)
		self.label_start.move(10,485+430+55)
		self.label_start.setFont(font)
		self.label_end = QLabel("00:00",self)
		self.label_end.setFont(font)
		self.label_end.move(534,485+430+55)
		# 设置暂停播放和下一个按钮
		self.stop_button = QPushButton(self)
		self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/开始.png'))
		self.stop_button.setIconSize(QSize(30,30))
		self.stop_button.clicked.connect(self.stop_action)
		self.stop_button.move(564+25,485+430+20)
		self.next_button = QPushButton(self)
		self.next_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/下一个.png'))
		self.next_button.setIconSize(QSize(30,30))
		self.next_button.clicked.connect(self.next_action)
		self.next_button.move(564+35+35,485+430+20)

	# 定义将opencv图像转PyQt图像的函数
	def cvImgtoQtImg(self,cvImg):  
		QtImgBuf = cv2.cvtColor(cvImg, cv2.COLOR_BGR2BGRA)
		QtImg = QtGui.QImage(QtImgBuf.data, QtImgBuf.shape[1], QtImgBuf.shape[0],QtGui.QImage.Format_RGB32)
		return QtImg

	# 播放视频进行参数跟踪，目前只有OTSU的二值化图像
	def playVideoFile(self,fn):
		self.cap = cv2.VideoCapture(fn)
		frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
		self.settingSlider(frames)
		fps = 24
		self.loop_flag = 0
		if not self.cap.isOpened():
			print("Cannot open Video File")
			exit()
		while not self.bClose:
			ret, frame = self.cap.read()  # 逐帧读取影片
			if not ret:
				if frame is None:
					print("The video has end.")
				else:
					print("Read video error!")
				break
			if self.moving_flag==0:
				self.label_start.setText(self.int2time(self.loop_flag))
				self.s1.setValue(int(self.loop_flag/24))#设置当前值
			self.loop_flag += 1

			# 处理得到二值化图像
			global side,md
			mat_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),4)
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					if(aa[i][m]==1):
						aa[i][m]=0
					m=m+1
			# 得到边线数组
			m=0
			for i in range(56,120):
				d=0
				for j in range(188):
					md[m][d]=aa[i][j]
					d=d+1
				m=m+1
			frame = np.array(md, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_int8*64*3
			t=a()
			mylib.Get_line_LMR.restype=POINTER(c_uint8)
			t1=mylib.Get_line_LMR(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			for i in range(3):
				m=0
				for j in range(64*i,(i+1)*64):
					side[m][i]=t1[j]
					m=m+1	
			# 创建一个新的cv2格式图像存储二值化图像
			img_binary=create_image_singel(aa)
			# 转化到label支持的图像格式
			QtImg = self.cvImgtoQtImg(img_binary)
			# 显示二值化图像
			self.label_13.setPixmap(QtGui.QPixmap.fromImage(QtImg).scaled(self.label_13.size()))
			self.label_13.show()  # 刷新界面

			# 创建一个新的cv2格式图像存储边线图像
			img=create_image(side)
			# 转化到label支持的图像格式
			QtImg = self.cvImgtoQtImg(img)
			# 显示二值化图像
			self.label_14.setPixmap(QtGui.QPixmap.fromImage(QtImg).scaled(self.label_14.size()))
			self.label_14.show()  # 刷新界面
			
			# 用于参数追踪
			a=c_float*2
			t=a()
			mylib.control.restype=POINTER(c_float)
			omega=c_float(1.0)
			cha=side[ROAD_MAIN_ROW][1]-93
			# 输出舵机转角以及车身倾角信息
			t1=mylib.control(omega,cha,byref(t),0)
			if(cha<0):
				t1[0]=t1[0]-90
				t1[1]=t1[1]-90
			# 进行可视化
			self.label_7.setText(str(round(t1[0],2)))
			self.rota_top.car_top_item.setRotation(t1[0])  # 自身改变旋转度
			self.label_10.setText(str(round(t1[1],2)))
			self.rota.car_back_item.setRotation(t1[1])  # 自身改变旋转度
			
			# 进行边线数组以及基本参数的可视化
			global labelLeft,labelRight,labelMid,label_canshu_value
			for i in range(len(label_canshu_value)):
				aaa=self.findChild(QLabel,label_canshu_value[i])
				aaa.setText(str(t1[0]))
			# 进行边线数组以及基本参数的可视化
			for i in range(64):
				aaa=self.findChild(QLabel,labelLeft[i])
				aaa.setText(str(side[i][0]))
				aaa=self.findChild(QLabel,labelMid[i])
				aaa.setText(str(side[i][1]))
				aaa=self.findChild(QLabel,labelRight[i])
				aaa.setText(str(side[i][2]))
				
			while self.stop_flag == 1:  # 暂停的动作
				cv2.waitKey(int(1000/fps))  # 休眠一会，因为每秒播放24张图片，相当于放完一张图片后等待41ms
			cv2.waitKey(int(1000/fps))  # 休眠一会，因为每秒播放24张图片，相当于放完一张图片后等待41ms
        # 释放
		self.cap.release()

	# 暂停触发
	def stop_action(self):
		if self.stop_flag == 0:
			self.stop_flag = 1
			self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/暂停.png'))
		else:
			self.stop_flag = 0
			self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/开始.png'))

	# 下一个触发
	def next_action(self):
		self.bClose = True
		self.play_index = (self.play_index+1)%3
		self.bClose = False
		self.playVideoFile(self.fn_list[self.play_index])

    # 滑动条触发 
	def start_drag(self):
		self.moving_flag = 1

	# 拖动行为
	def drag_action(self):
		self.moving_flag = 0
		print('当前进度为%d，被拉动到的进度为%d'%(self.s1.value(), int(self.loop_flag/24)))
		if self.s1.value()!=int(self.loop_flag/24):
			print('当前进度为:'+str(self.s1.value()))
			self.loop_flag = self.s1.value()*24
			self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.loop_flag)

	# 暂停行为
	def video_stop(self):
		self.bClose = True

	# 手动选择视频播放
	def manual_choose(self):
		fn,_ = QFileDialog.getOpenFileName(self,'Open file','C:/Users/17628/Desktop/ui/test video',"Video files (*.mp4 *.avi)")
		self.playVideoFile(fn)

	# 自动选择视频播放，如何自动视频播放？
	def auto_choose(self):
		self.playVideoFile(self.fn_list[self.play_index])
    
	# 设置进度条
	def settingSlider(self,maxvalue):
		self.s1.setMaximum(int(maxvalue/24))
		self.label_end.setText(self.int2time(maxvalue))

	# 视频播放中的时间函数
	def int2time(self,num):
        # 每秒刷新24帧
		num = int(num/24)
		minute = int(num/60)
		second = num - 60*minute
		if minute < 10:
			str_minute = '0'+str(minute)
		else:
			tr_minute = str(minute)
		if second < 10:
			str_second = '0'+str(second)
		else:
			str_second = str(second)
		return str_minute+":"+str_second

	# 暂停行为
	def exit_stop(self):
		self.stop_flag = 0
		self.bClose = True
		self.close()

    # 用于旋转角度可视化的测试
	def startcal(self):
		a=c_float*2
		t=a()
		mylib.control.restype=POINTER(c_float)
		omega=c_float(1.0)
		t1=mylib.control(omega,10,byref(t),0)
		self.label_7.setText(str(round(t1[0],2)))
		print(t1)
		self.rota_top.car_top_item.setRotation(t1[0])  # 自身改变旋转度
		self.label_10.setText(str(round(t1[1],2)))
		self.rota.car_back_item.setRotation(t1[1])  # 自身改变旋转度
		print(t1[0],t1[1])

	# 打开图片，有bug，未调，不需要
	def openimage(self):
		imgName, imgType = QFileDialog.getOpenFileName(self, "打开图片", "", "*.bmp;;*.jpg;;*.png;;All Files(*)")
		# 将图片转化为label支持的图像格式
		jpg = QtGui.QPixmap(imgName).scaled(self.label.width(), self.label.height())
		# 显示在label上
		self.label.setPixmap(jpg)

		imgptr = self.label.pixmap().toImage()
		ptr = imgptr.constBits()
		ptr.setsize(imgptr.byteCount())
		mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
		mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
		gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
		ret, binary = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)

		frame = np.array(binary, dtype=np.uint8)
		array = frame.astype(c_uint8)
		a=c_uint8*120*2
		t=a()
		mylib.ImageGetSide.restype=c_char_p  
		t1=mylib.ImageGetSide(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
		aa=[]
		m=0
		for i in range(120):
			aa.append([])
			for j in range(2):
				aa[i].append(t1[m])
				m=m+1
		binary = Image.fromarray(np.uint8(binary))
		binary = binary.toqpixmap() #QPixmap
		self.label_13.setPixmap(binary)     
		
	# 用于旋转角度可视化的测试
	def clickbutton(self):
		print("click")
		if not self.stop:
			self.rota.anim.start()
			self.rota_top.anim.start()
			self.rota_tyre.anim.start()
		else:
			self.rota.anim.stop()
			self.rota_top.anim.stop()
			self.rota_tyre.anim.stop()
		self.stop = ~self.stop

# 旋转动画类 后视角
class Rotation(QObject):
	def __init__(self):
		super().__init__()
		car_back = QPixmap("./icon/car_back.png")
		scaledPixmap = car_back.scaled(261/1.5, 458/1.5)  # 动画大小
		self.animation()
		self.car_back_item = QGraphicsPixmapItem(scaledPixmap)
		self.car_back_item.setPos(1600,370)
		self.car_back_item.setTransformOriginPoint(130/1.5, 458/1.5)  # 设置中心为旋转

	def _set_rotation(self, degree):
		self.car_back_item.setRotation(degree)  # 自身改变旋转度

	def animation(self):
		self.anim = QPropertyAnimation(self, b'rotation')  # 动画类型
		self.anim.setDuration(1000)
		self.anim.setStartValue(45)  # 初始角度
		self.anim.setEndValue(-45)
		self.anim.setLoopCount(-1)  # 设置循环旋转

	rotation = pyqtProperty(int, fset=_set_rotation)  # 属性动画改变自身数值

# 旋转动画类 顶视角
class Rotation_top(QObject):
	def __init__(self):
		super().__init__()
		car_top = QPixmap("./icon/car_top.png")
		scaledPixmap2 = car_top.scaled(220/1.5, 247/1.5)  # 动画大小
		self.animation()
		self.car_top_item = QGraphicsPixmapItem(scaledPixmap2)
		self.car_top_item.setPos(1607,100)
		self.car_top_item.setTransformOriginPoint(110/1.5, 230/1.5)  # 设置中心为旋转

	def _set_rotation(self, degree):
		self.car_top_item.setRotation(degree)  # 自身改变旋转度数

	def animation(self):
		self.anim = QPropertyAnimation(self, b'rotation')  # 动画类型
		self.anim.setDuration(1000)
		self.anim.setStartValue(45)  # 初始角度
		self.anim.setEndValue(-45)
		self.anim.setLoopCount(-1)  # 设置循环旋转

	rotation = pyqtProperty(int, fset=_set_rotation)  # 属性动画改变自身数值

# 旋转动画类 车轮
class Rotation_tyre(QObject):
	def __init__(self):
		super().__init__()
		car_tyre = QPixmap("./icon/car_tyre.png")
		scaledPixmap2 = car_tyre.scaled(236/1.5, 236/1.5)  # 动画大小
		self.animation()
		self.car_tyre_item = QGraphicsPixmapItem(scaledPixmap2)
		self.car_tyre_item.setPos(1607,770)
		self.car_tyre_item.setTransformOriginPoint(118/1.5, 118/1.5)  # 设置中心为旋转

	def _set_rotation(self, degree):
		self.car_tyre_item.setRotation(degree)  # 自身改变旋转度数

	def animation(self):
		self.anim = QPropertyAnimation(self, b'rotation')  # 动画类型
		self.anim.setDuration(1000)
		self.anim.setStartValue(0)  # 初始角度
		self.anim.setEndValue(360)
		self.anim.setLoopCount(-1)  # 设置循环旋转

	rotation = pyqtProperty(int, fset=_set_rotation)  # 属性动画改变自身数值

# 旋转动画类 俯视角
class Rotation_real(QObject):
	def __init__(self):
		super().__init__()
		car_back_real = QPixmap("./icon/car_back.png")
		scaledPixmap = car_back_real.scaled(261/1.5, 458/1.5)  # 动画大小
		self.animation()
		self.car_back_real_item = QGraphicsPixmapItem(scaledPixmap)
		self.car_back_real_item.setPos(1225,370)
		self.car_back_real_item.setTransformOriginPoint(130/1.5, 458/1.5)  # 设置中心为旋转

	def _set_rotation(self, degree):
		self.car_back_real_item.setRotation(degree)  # 自身改变旋转度

	def animation(self):
		self.anim = QPropertyAnimation(self, b'rotation')  # 动画类型
		self.anim.setDuration(1000)
		self.anim.setStartValue(45)  # 初始角度
		self.anim.setEndValue(-45)
		self.anim.setLoopCount(-1)  # 设置循环旋转

	rotation = pyqtProperty(int, fset=_set_rotation)  # 属性动画改变自身数值

# 旋转动画类 顶视角
class Rotation_top_real(QObject):
	def __init__(self):
		super().__init__()
		car_top_real = QPixmap("./icon/car_top.png")
		scaledPixmap2 = car_top_real.scaled(220/1.5, 247/1.5)  # 动画大小
		self.animation()
		self.car_top_real_item = QGraphicsPixmapItem(scaledPixmap2)
		self.car_top_real_item.setPos(1232,100)
		self.car_top_real_item.setTransformOriginPoint(110/1.5, 230/1.5)  # 设置中心为旋转

	def _set_rotation(self, degree):
		self.car_top_real_item.setRotation(degree)  # 自身改变旋转度数

	def animation(self):
		self.anim = QPropertyAnimation(self, b'rotation')  # 动画类型
		self.anim.setDuration(1000)
		self.anim.setStartValue(45)  # 初始角度
		self.anim.setEndValue(-45)
		self.anim.setLoopCount(-1)  # 设置循环旋转

	rotation = pyqtProperty(int, fset=_set_rotation)  # 属性动画改变自身数值

# 旋转动画类 车轮
class Rotation_tyre_real(QObject):
	def __init__(self):
		super().__init__()
		car_tyre_real = QPixmap("./icon/car_tyre.png")
		scaledPixmap2 = car_tyre_real.scaled(236/1.5, 236/1.5)  # 动画大小
		self.animation()
		self.car_tyre_real_item = QGraphicsPixmapItem(scaledPixmap2)
		self.car_tyre_real_item.setPos(1232,770)
		self.car_tyre_real_item.setTransformOriginPoint(118/1.5, 118/1.5)  # 设置中心为旋转

	def _set_rotation(self, degree):
		self.car_tyre_item.setRotation(degree)  # 自身改变旋转度数

	def animation(self):
		self.anim = QPropertyAnimation(self, b'rotation')  # 动画类型
		self.anim.setDuration(1000)
		self.anim.setStartValue(0)  # 初始角度
		self.anim.setEndValue(360)
		self.anim.setLoopCount(-1)  # 设置循环旋转

	rotation = pyqtProperty(int, fset=_set_rotation)  # 属性动画改变自身数值

# 边线检测工具
class seed(QMainWindow):
	# 视频播放列表
	bClose = False
	fn_list = ["C:/Users/17628/Desktop/ui/test video/慢速版.mp4","C:/Users/17628/Desktop/ui/test video/正常版.mp4"]
	play_index = 0
	def __init__(self):
        # 1.初始化：初始化地图数组
		print('初始化边线检测工具...')
		self.Map=[]
		for i in range(config.HEIGHT):
			col=[]
			for j in range(config.WIDTH):
				col.append(220)
			self.Map.append(col)
        # 2.初始化：起点和目的地
		self.startPoint=None
		self.endPoint=None
        # 3.初始化：其他参数
		self.search=None
		self.centerTimer=None
		self.yi=None
		self.special=None
		self.displayFlush=False
		super().__init__()
		self.initUI()
	def initUI(self):
		#开始初始化UI部分
		self.label_display=QTextEdit("",self)
		self.button_start=QPushButton("绘制边线",self)
		self.button_clearWall=QPushButton("清空画布",self)
		self.button_saveMap=QPushButton("一键存参",self)
		self.button_saveMap.setShortcut("Ctrl+s")

		# 字体类设置
		font = QtGui.QFont()
		font.setFamily("Consolas")
		font.setPointSize(12)
		font.setBold(False)
		font.setItalic(False)
		font.setWeight(3)

		font2 = QtGui.QFont()
		font2.setFamily("Consolas")
		font2.setPointSize(11)
		font2.setBold(False)
		font2.setItalic(False)
		font2.setWeight(3)

		self.button_start.setFont(font)
		self.button_clearWall.setFont(font)
		self.button_saveMap.setFont(font)
		self.label_display.setFont(font)

		# 建立滚动条布局
		self.topFiller = QWidget()
		self.topFiller.setMinimumSize(188*3, 2000)  # 设置滚动条的尺寸

		# 设置基本参数列表，初始化
		global label_canshu_value,label_canshu_c
		label_canshu=["左边界行搜索起始点","右边界行搜索起始点","左边界列搜索起始点","右边界列搜索起始点","左边界搜点最多允许数","右边界搜点最多允许数","左搜索结束列值","右搜索结束列值","左丢线限制次数","右丢线限制次数","中间左丢线列搜索起始点","中间右丢线列搜索起始点","求拐点角度的点数间隔"]
		label_canshu_e=["L_basic_row_start","R_basic_row_start","(L_edge_start_col)","(R_edge_start_col)","L_search_amount","R_search_amount","(min_col)","(max_col)","(L_lost_)","(R_lost_)","(L_edge_lost_start_col)","(R_edge_lost_start_col)","(dist)"]
		label_canshu_value=[118,118,3,157,150,150,3,157,10,10,4,155,4]
		label_canshu_c=[]
		for i in range(len(label_canshu)):
			num = i
			var = 'v' + str(num)
			label_canshu_c.append(var)

		# 动态生成基本参数列表的label，同时设定对应的布局
		for i in range(len(label_canshu)):
			self.label1000=QLabel(self.topFiller)
			self.label1000.setText(label_canshu[i])
			self.label1000.move(20,50+i*50)
			self.label1000.setFont(font)

			self.label2000=QLabel(self.topFiller)
			self.label2000.setText(label_canshu_e[i])
			self.label2000.move(20,70+i*50)
			self.label2000.setFont(font)
			self.label2000.setStyleSheet("color:#2A78D6")

			self.label3000=QSpinBox(self.topFiller)
			self.label3000.setObjectName(label_canshu_c[i])
			self.label3000.setMinimum(0)
			self.label3000.setMaximum(187)
			self.label3000.setValue(label_canshu_value[i])
			self.label3000.resize(60,30)
			self.label3000.move(250,50+i*50)
			self.label3000.setFont(font)

		# 基本参数列表头
		self.label17=QLabel(self.topFiller)
		self.label17.setText("控制参数                  参数值\n--------------------------------------------------------")
		self.label17.setStyleSheet("color:#F46F0C")
		self.label17.setFont(font)
		self.label17.resize(188*3,50)
		self.label17.move(20,0)

		# 将布局放置在滚动条中
		self.scrol = QScrollArea(self)
		self.scrol.setWidget(self.topFiller)
		self.scrol.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.scrol.resize(350,355)
		self.scrol.move(1555,245)

		self.label = QLabel(self)
		self.label.setText("显示图片")
		self.label.setFixedSize(188, 120)
		self.label.move(1610, 50)
		self.label.setFont(font)

		self.label_3 = QLabel(self)
		self.label_3.setText("OTSU阈值：")
		self.label_3.setFixedSize(80, 50)
		self.label_3.move(1810, 70)
		self.label_3.setFont(font)

		self.label_4 = QLabel(self)
		self.label_4.setText("")
		self.label_4.setFixedSize(50, 50)
		self.label_4.move(1850, 120)
		self.label_4.setFont(font)

		self.radioButton = QRadioButton(self)
		self.radioButton.setText("OTSU")
		self.radioButton.resize(120,30)
		self.radioButton.move(1555,610)
		self.radioButton.setFont(font2)

		self.radioButton2 = QRadioButton(self)
		self.radioButton2.setText("龙邱OTSU")
		self.radioButton2.resize(120,30)
		self.radioButton2.move(1685,610)
		self.radioButton2.setFont(font2)

		self.radioButton3 = QRadioButton(self)
		self.radioButton3.setText("手动阈值")
		self.radioButton3.resize(120,30)
		self.radioButton3.move(1815,610)
		self.radioButton3.setFont(font2)

		self.radioButton4 = QRadioButton(self)
		self.radioButton4.setText("sobel手动阈值")
		self.radioButton4.resize(120,30)
		self.radioButton4.move(1555,640)
		self.radioButton4.setFont(font2)

		self.radioButton5 = QRadioButton(self)
		self.radioButton5.setText("sobel自动阈值")
		self.radioButton5.resize(120,30)
		self.radioButton5.move(1685,640)
		self.radioButton5.setFont(font2)

		self.radioButton6 = QRadioButton(self)
		self.radioButton6.setText("平均阈值")
		self.radioButton6.resize(120,30)
		self.radioButton6.move(1815,640)
		self.radioButton6.setFont(font2)

		self.btn = QPushButton(self)
		self.btn.setText("打开图片")
		self.btn.move(1555, 10)
		self.btn.resize(170,30)
		self.btn.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.btn.clicked.connect(self.openimage)
		self.btn.setFont(font)

		self.play_video = QPushButton(self)
		self.play_video.setText("打开视频")
		self.play_video.move(1730, 10)
		self.play_video.resize(170,30)
		self.play_video.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.play_video.clicked.connect(self.manual_choose)
		self.play_video.setFont(font)

		#设置控件样式
		self.label_display.setStyleSheet("border:2px solid black")
		self.label_display.setAlignment(Qt.AlignLeft)
		self.label_display.setAlignment(Qt.AlignTop)

		#设置控件的尺寸和位置
		self.button_saveMap.resize(170,30)
		self.button_start.resize(345,30)
		self.button_clearWall.resize(170,30)
		self.label_display.resize(350,220)
		self.button_start.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_clearWall.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.button_saveMap.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")	

		# 设置控件位置
		self.label_display.move(1555,765)
		self.button_start.move(1555,680)
		self.button_clearWall.move(1555+175,715)
		self.button_saveMap.move(1555,715)
		
		#给控件绑定事件
		self.button_start.clicked.connect(self.button_StartEvent)
		self.button_clearWall.clicked.connect(self.button_Clear)
		self.button_saveMap.clicked.connect(self.button_SaveMap)

		# 设置滑动条
		self.s1 = QSlider(Qt.Horizontal,self)
		self.s1.setToolTip("滑动条")
		self.s1.setMinimum(0)#设置最大值
		self.s1.setMaximum(50)#设置最小值
		self.s1.setSingleStep(1)#设置间隔
		self.s1.setValue(0)#设置当前值
		self.s1.sliderMoved.connect(self.start_drag)
		self.s1.sliderReleased.connect(self.drag_action)
		self.s1.setFixedSize(250, 25)
		self.moving_flag = 0
		self.stop_flag = 0  # 如果当前为播放值为0,如果当前为暂停值为1
		self.s1.move(1560,180)
		# 设置两个标签分别是当前时间和结束时间
		self.label_start = QLabel("00:00",self)
		self.label_start.move(1555,205)
		self.label_start.setFont(font)
		self.label_end = QLabel("00:00",self)
		self.label_end.setFont(font)
		self.label_end.move(1555+188+40,205)
		# 设置暂停播放和下一个按钮
		self.stop_button = QPushButton(self)
		self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/开始.png'))    # 设置图标
		self.stop_button.setIconSize(QSize(25,25))
		self.stop_button.clicked.connect(self.stop_action)
		self.stop_button.resize(30,30)
		self.stop_button.move(1560+270,180)
		self.next_button = QPushButton(self)
		self.next_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/下一个.png'))   # 设置图标
		self.next_button.setIconSize(QSize(25,25))
		self.next_button.clicked.connect(self.next_action)
		self.next_button.resize(30,30)
		self.next_button.move(1560+270+40,180)
		# UI初始化完成
		self.setGeometry(0, 0, 1920, 1080)
		self.setStyleSheet("background-color:#EDEDF5")   # 设置背景色
		self.setWindowTitle('边线检测工具')
		# self.show()

	# 打开资源管理器对话框，输入图片
	def openimage(self):
		imgName, imgType = QFileDialog.getOpenFileName(self, "打开图片", "", "*.bmp;;*.jpg;;*.png;;All Files(*)")
		jpg = QtGui.QPixmap(imgName).scaled(self.label.width(), self.label.height())
		self.label.setPixmap(jpg)  # 将图片加载到label上

	# debug窗口
	def addDisplayText(self,text):
		if self.displayFlush:
			self.label_display.setText(text+'\n')
			self.displayFlush=False
		else:
			self.label_display.setText(self.label_display.toPlainText()+text+'\n')

	# 此类没啥用
	def mousePressEvent(self,event):
		x,y=event.x()-50,event.y()-50
		x=x//config.blockLength
		y=y//config.blockLength
		if x>=0 and x<config.WIDTH and y>=0 and y<config.HEIGHT:
            # 左键按下添加墙壁
			if event.button()==Qt.LeftButton:
				if (x,y)!=self.startPoint and (x,y)!=self.endPoint:
					self.Map[y][x]=(1 if self.Map[y][x]==0 else 0)
            # 邮件按下第一次添加起点，第二次终点
			if event.button()==Qt.RightButton:
				if self.Map[y][x]==0:
					if self.startPoint==None:
						self.startPoint=(x,y)
						self.addDisplayText('添加了一个起点:(%d,%d)'%(x,y))
					elif self.endPoint==None and self.startPoint!=(x,y):
						self.endPoint=(x,y)
						self.addDisplayText('添加了一个终点:(%d,%d)'%(x,y))
			self.repaint()

	# 巡线，静态图片调试边线
	def button_StartEvent(self):
		# 获取选项框情况
		type=self.checkRadioButton()
		global aa,bb,bc,side,md,label_canshu_c,label_canshu_value
		for i in range(120):
			for j in range(2):
				bb[i][j]=0	
		for i in range(120):
			for j in range(188):
				self.Map[i][j]=0
		for i in range(len(label_canshu_c)):
			aaa=self.findChild(QSpinBox,label_canshu_c[i])
			value3[i]=aaa.value()
		# 转化为可以处理的数组
		frame = np.array(value3, dtype=np.uint8)
		# 将格式转换为c动态链接库的格式
		array = frame.astype(c_uint8)
		# 开辟存储空间
		a=c_uint8*(30)
		# t指针指向存储空间
		t=a()
		# 函数的目标输出格式
		# mylib.git_gui_value.restype=POINTER(c_uint8)
		# 调用动态链接库函数
		mylib2.git_gui_value(array.ctypes.data_as(POINTER(c_uint8)))
		# 将输出量转化为py中的list，存储在value3中
		# for i in range(30):
		# 	value3[i]=t1[i]
		print(value3)
		if(type==1):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			# 转化为可以处理的数组
			frame = np.array(gray, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_uint8*(120*188+1)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),4)
			# 将输出量转化为py中的list，存储在aa中
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					if(aa[i][m]==1):
						aa[i][m]=0
					if(aa[i][m]==255):
						aa[i][m]=1
					m=m+1
			# 在label中显示阈值
			self.label_4.setText(str(t1[120*188]))
			# 初始化用于判断边线的数组，存储在md中 120*160
			m=0
			for i in range(0,120):
				d=0
				for j in range(14,174):
					md2[i][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md2, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*(150*3*2+100+480)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib2.Image_process.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			m=0
			for i in range(0,300,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(300,600,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(600,900,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			# print(side2)
		if(type==2):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			# 转化为可以处理的数组
			frame = np.array(gray, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_uint8*(120*188+1)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),0)
			# 将输出量转化为py中的list，存储在aa中
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					if(aa[i][m]==1):
						aa[i][m]=0
					if(aa[i][m]==255):
						aa[i][m]=1
					m=m+1
			# 在label中显示阈值
			self.label_4.setText(str(t1[120*188]))
			# 初始化用于判断边线的数组，存储在md中 120*160
			m=0
			for i in range(0,120):
				d=0
				for j in range(14,174):
					md2[i][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md2, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*(150*3*2+100+480)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib2.Image_process.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			m=0
			for i in range(0,300,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(300,600,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(600,900,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
		if(type==3):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)

			# 固定阈值进行二值化
			binary = threshold_demo(mat_img,self.horizontalSlider.value())
			# 在label中显示阈值
			self.label_4.setText(str(self.horizontalSlider.value()))
			# 将输出量转化为py中的list，存储在aa中
			aa = binary.tolist()
			# 初始化用于判断边线的数组，存储在md中 64*188
			for i in range(120):
				m=0
				for j in range(188):
					if(aa[i][m]==1):
						aa[i][m]=0
					if(aa[i][m]==255):
						aa[i][m]=1
					m=m+1
			# 初始化用于判断边线的数组，存储在md中 120*160
			m=0
			for i in range(0,120):
				d=0
				for j in range(14,174):
					md2[i][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md2, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*(150*3*2+100+480)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib2.Image_process.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			m=0
			for i in range(0,300,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(300,600,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(600,900,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
		if(type==4):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			#将图片数据转换为可处理的整型数据
			frame=gray.tolist
			frame = np.array(gray, dtype=np.float)

			# 为方便调参，调用py编写的函数
			sobel_motor_py(frame,aa,self.horizontalSlider2.value())
			# 初始化用于判断边线的数组，存储在md中 64*188
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					if(aa[i][m]==1):
						aa[i][m]=0
					if(aa[i][m]==255):
						aa[i][m]=1
					m=m+1
			# 在label中显示阈值
			# 初始化用于判断边线的数组，存储在md中 120*160
			m=0
			for i in range(0,120):
				d=0
				for j in range(14,174):
					md2[i][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md2, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*(150*3*2+100+480)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib2.Image_process.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			m=0
			for i in range(0,300,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(300,600,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(600,900,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
		if(type==5):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			# 将图片数据转换为可处理的整型数据
			frame=gray.tolist
			frame = np.array(gray, dtype=np.float)
			# 为方便调参，调用py编写的函数
			sobel_auto_py(frame,aa,self.horizontalSlider3.value()/10)
			# 初始化用于判断边线的数组，存储在md中 64*188
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					if(aa[i][m]==1):
						aa[i][m]=0
					if(aa[i][m]==255):
						aa[i][m]=1
					m=m+1
			# 在label中显示阈值
			# 初始化用于判断边线的数组，存储在md中 120*160
			m=0
			for i in range(0,120):
				d=0
				for j in range(14,174):
					md2[i][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md2, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*(150*3*2+100+480)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib2.Image_process.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			m=0
			for i in range(0,300,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(300,600,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(600,900,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
		if(type==6):
			# 获取label上的图像，并进行一些数据转换
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)

			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_uint8*(120*188+1)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),1)
			# 将输出量转化为py中的list，存储在aa中
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					if(aa[i][m]==1):
						aa[i][m]=0
					if(aa[i][m]==255):
						aa[i][m]=1
					m=m+1
			# 在label中显示阈值
			self.label_4.setText(str(t1[120*188]))
			# 初始化用于判断边线的数组，存储在md中 120*160
			m=0
			for i in range(0,120):
				d=0
				for j in range(14,174):
					md2[i][d]=aa[i][j]
					d=d+1
				m=m+1
			# 转化为可以处理的数组
			frame = np.array(md2, dtype=np.uint8)
			# 将格式转换为c动态链接库的格式
			array = frame.astype(c_uint8)
			# 开辟存储空间
			a=c_int8*(150*3*2+100+480)
			# t指针指向存储空间
			t=a()
			# 函数的目标输出格式
			mylib2.Image_process.restype=POINTER(c_uint8)
			# 调用动态链接库函数
			t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			# 将输出量转化为py中的list，存储在side中
			m=0
			for i in range(0,300,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(300,600,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
			for i in range(600,900,2):
				side2[m][0]=t1[i]
				side2[m][1]=t1[i+1]
				m=m+1
		# 滤波，未开启
		# filter_py(aa,bc,self.horizontalSlider4.value()*255)
		# bc,bb=ImageGetSide(bc,bb)
        
		# 输出信息到绘图区
		for i in range(24):
			value2[i]=t1[900+i]
		m=0
		for i in range(1000,1240,2): #左边线补线
			side3[m][0]=t1[i]
			side3[m][1]=t1[i+1]
			m=m+1
		m=120
		for i in range(1240,1480,2): #右边线补线
			side3[m][0]=t1[i]
			side3[m][1]=t1[i+1]
			m=m+1
		# 二值化图像
		for i in range(120):
			for j in range(0,14):
				self.Map[i][j]=aa[i][j]
			for j in range(174,188):
				self.Map[i][j]=aa[i][j]
		for i in range(120):
			dk=14
			for j in range(160):
				md2[i][j]=aa[i][dk]
				if(md2[i][j]==1):
					md2[i][j]=255
				dk=dk+1
		# 边线信息
		for i in range(150):
			md2[side2[i][0]][side2[i][1]]=100 #左边线
			# print(side2[i][0],side2[i][1])
		for i in range(150,300):
			md2[side2[i][0]][side2[i][1]]=200 #右边线
			# print(side2[i][0],side2[i][1])
		for i in range(300,450):
			md2[side2[i][0]][side2[i][1]]=150 #中线
		for i in range(0,120):
			# print(side3[i][0],side3[i][1])
			md2[side3[i][0]][side3[i][1]]=75 #中线
		for i in range(120,240):
			md2[side3[i][0]][side3[i][1]]=70 #中线
		# for i in range(160):
		# 	md2[ROAD_MAIN_ROW][i]=50 #主跑行
		# k=-0.624260
		# b=74.928101
		# for i in range(120):
		# 	y=int((k*i+b))
		# 	if(y>159):
		# 		y=159
		# 	md2[i][y]=75
		# k=0.630268
		# b=85.492569
		# for i in range(120):
		# 	y=int((k*i+b))
		# 	if(y>159):
		# 		y=159
		# 	md2[i][y]=70
		# md2[111][3]=50
		# md2[111][157]=50
		md2[value2[0]][value2[1]]=70 #左边线起始点
		md2[value2[2]][value2[3]]=70 #右边线起始点
		md2[value2[4]][value2[5]]=71 #丢线情况，左边线起始点
		md2[value2[6]][value2[7]]=71 #丢线情况，右边线起始点
		md2[value2[8]][value2[9]]=70 #中间左丢线情况，左边线起始点
		md2[value2[10]][value2[11]]=70 #中间右丢线情况，右边线起始点
		md2[value2[12]][value2[13]]=75 #左下拐点
		md2[value2[15]][value2[16]]=75 #右下拐点
		md2[value2[18]][value2[19]]=75 #左上拐点
		md2[value2[21]][value2[22]]=75 #右上拐点

		zz=0
		dd=0
		for i in range(0,120):
			dd=0
			for j in range(14,174):
				self.Map[i][j]=md2[i][dd]
				dd=dd+1
		zz=zz+1
		
		config.HEIGHT=len(self.Map)
		config.WIDTH=len(self.Map[0])
		self.repaint()
		self.addDisplayText('开始边线规划')
		self.addDisplayText('左边线起始点:(%d,%d)'%(value2[0],value2[1]))
		self.addDisplayText('右边线起始点:(%d,%d)'%(value2[2],value2[3]))
		self.addDisplayText('丢线情况:左边线起始点:(%d,%d)'%(value2[4],value2[5]))
		self.addDisplayText('丢线情况:右边线起始点:(%d,%d)'%(value2[6],value2[7]))
		self.addDisplayText('中间左丢线情况,左边线起始点:(%d,%d)'%(value2[8],value2[9]))
		self.addDisplayText('中间右丢线情况,右边线起始点:(%d,%d)'%(value2[10],value2[11]))
		self.addDisplayText('左下拐点:(%d,%d),角度:%d'%(value2[12],value2[13],value2[14]))
		self.addDisplayText('右下拐点:(%d,%d),角度:%d'%(value2[15],value2[16],value2[17]))
		self.addDisplayText('左上拐点:(%d,%d),角度:%d'%(value2[18],value2[19],value2[20]))
		self.addDisplayText('右上拐点:(%d,%d),角度:%d\n'%(value2[21],value2[22],value2[23]))

	# 数据保存，可用快捷键ctrl+s
	def button_SaveMap(self):
		# 获取目前的参数值
		aaa=self.findChild(QSpinBox,label_canshu_c[0])
		ctr_bianxian.L_basic_row_start = aaa.value()    	# 左边界行搜索起始点
		aaa=self.findChild(QSpinBox,label_canshu_c[1])
		ctr_bianxian.R_basic_row_start = aaa.value()    	# 右边界行搜索起始点
		aaa=self.findChild(QSpinBox,label_canshu_c[2])
		ctr_bianxian.L_edge_start_col = aaa.value()    		# 左边界列搜索起始点
		aaa=self.findChild(QSpinBox,label_canshu_c[3])
		ctr_bianxian.R_edge_start_col = aaa.value()     	# 右边界列搜索起始点
		aaa=self.findChild(QSpinBox,label_canshu_c[4])
		ctr_bianxian.L_search_amount = aaa.value()  		# 左边界搜点最多允许数
		aaa=self.findChild(QSpinBox,label_canshu_c[5])
		ctr_bianxian.R_search_amount = aaa.value()    		# 右边界搜点最多允许数
		aaa=self.findChild(QSpinBox,label_canshu_c[6])
		ctr_bianxian.min_col = aaa.value()           		# 左搜索结束列值
		aaa=self.findChild(QSpinBox,label_canshu_c[7])
		ctr_bianxian.max_col = aaa.value()       			# 右搜索结束列值
		aaa=self.findChild(QSpinBox,label_canshu_c[8])
		ctr_bianxian.L_lost_ = aaa.value()  				# 左丢线限制次数
		aaa=self.findChild(QSpinBox,label_canshu_c[9])
		ctr_bianxian.R_lost_ = aaa.value()    				# 右丢线限制次数
		aaa=self.findChild(QSpinBox,label_canshu_c[10])
		ctr_bianxian.L_edge_lost_start_col = aaa.value()   	# 中间左丢线列搜索起始点
		aaa=self.findChild(QSpinBox,label_canshu_c[11])
		ctr_bianxian.R_edge_lost_start_col = aaa.value()    # 中间右丢线列搜索起始点
		aaa=self.findChild(QSpinBox,label_canshu_c[12])
		ctr_bianxian.dist = aaa.value()         			# 求拐点角度的点数间隔
		# 保存参数
		ctr_bianxian.save_data()
		# 获取当前时间戳
		time = datetime.datetime.now()
		time1_str = datetime.datetime.strftime(time,'%Y-%m-%d %H:%M:%S')
		self.addDisplayText('参数保存成功,%s'%time1_str)

	# OTSU二值化
	def button_OTSU_(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*2*2)
			t=a()
			mylib3.process_image.restype=POINTER(c_uint8)
			t1=mylib3.process_image(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
			m=0
			for i in range(120):
				for j in range(2):
					left_line[i][j]=t1[m]
					m=m+1
			m=240
			for i in range(120):
				for j in range(2):
					right_line[i][j]=t1[m]
					m=m+1
			print(left_line)
			# adaptiveThreshold(frame,aa,7,188,120,2)
			sobel_auto_py(frame,aa,10.0)
			# array = frame.astype(c_uint8)
			# a=c_uint8*(120*188+1)
			# t=a()
			# mylib.Get_01_Value.restype=POINTER(c_uint8)
			# t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),4)
			# for i in range(120):
			# 	m=0
			# 	for j in range(188*i,(i+1)*188):
			# 		aa[i][m]=t1[j]
			# 		m=m+1
			# self.label_4.setText(str(t1[120*188]))
			# with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
			# 	f.write(json.dumps(aa))
			# with open('C:/Users/17628/Desktop/map2.txt','r') as f:
			# 	self.Map=json.loads(f.read())
			# 	config.HEIGHT=len(self.Map)
			# 	config.WIDTH=len(self.Map[0])
			# 	self.addDisplayText('OTSU')
			# 	self.repaint()
			for i in range(120):
				for j in range(188):
					self.Map[i][j]=aa[i][j]
			zz=0
			dd=0
			for i in range(120):
				dd=0
				for j in range(2):
					self.Map[left_line[i][0]][left_line[i][1]]=100
					self.Map[right_line[i][0]][right_line[i][1]]=200
					dd=dd+1
			zz=zz+1
			
			config.HEIGHT=len(self.Map)
			config.WIDTH=len(self.Map[0])
			self.repaint()
			self.addDisplayText('开始边线规划')
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')

	# 平均阈值二值化
	def button_th_average_(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),1)
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('平均阈值')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')

	# sobel手动阈值二值化
	def button_sobel_motor_(self):
		try:
			# 为了方便gui调参用python写了一遍
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame=gray.tolist
			frame = np.array(gray, dtype=np.float)
			sobel_motor_py(frame,aa,self.horizontalSlider2.value())
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('sobel算子手动阈值')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')
			# c动态链接库直接导入函数求解
			'''
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),2)
			aa=[]
			m=0
			for i in range(120):
				aa.append([])
				for j in range(188*i,(i+1)*188):
					aa[i].append(t1[j])
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('地图加载成功')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')
				'''

	# sobel自动阈值二值化
	def button_sobel_auto_(self):
		try:
			# 为了方便gui调参用python写了一遍
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame=gray.tolist
			frame = np.array(gray, dtype=np.float)
			sobel_auto_py(frame,aa,self.horizontalSlider3.value()/10)
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('sobel算子自动阈值')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')
			# c动态链接库直接导入函数求解
			'''
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),3)
			aa=[]
			m=0
			for i in range(120):
				aa.append([])
				for j in range(188*i,(i+1)*188):
					aa[i].append(t1[j])
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('地图加载成功')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')
				'''

	# 龙邱OTSU二值化
	def button_OTSU_LQ(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			#将图片数据转换为可处理的整型数据
			frame = np.array(gray, dtype=np.uint8)
			array = frame.astype(c_uint8)
			a=c_uint8*(120*188+1)
			t=a()
			mylib.Get_01_Value.restype=POINTER(c_uint8)
			t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),0)
			for i in range(120):
				m=0
				for j in range(188*i,(i+1)*188):
					aa[i][m]=t1[j]
					m=m+1
			self.label_4.setText(str(t1[120*188]))
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(aa))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('龙邱OTSU')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')

	# 固定阈值二值化
	def button_th_motor_(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
			binary = threshold_demo(mat_img,self.horizontalSlider.value())
			self.label_4.setText(str(self.horizontalSlider.value()))
			# numpy中ndarray文件转为list
			img_list = binary.tolist()
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(img_list))
			with open('C:/Users/17628/Desktop/map2.txt','r') as f:
				self.Map=json.loads(f.read())
				config.HEIGHT=len(self.Map)
				config.WIDTH=len(self.Map[0])
				self.addDisplayText('手动阈值')
				self.repaint()
		except Exception as e:
			print('失败',e,type(e))
			if type(e)==FileNotFoundError:
				self.addDisplayText('地图加载失败:地图文件不存在')
			elif type(e)==json.decoder.JSONDecodeError:
				self.addDisplayText('地图加载失败:错误的地图文件')

	# 图像滤波
	def button_filter_(self):
		# 为方便调参，调用py编写的函数
		filter_py(aa,bc,self.horizontalSlider4.value()*255)
		with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
			f.write(json.dumps(bc))
		with open('C:/Users/17628/Desktop/map2.txt','r') as f:
			self.Map=json.loads(f.read())
			config.HEIGHT=len(self.Map)
			config.WIDTH=len(self.Map[0])
			self.addDisplayText('进行滤波')
			self.repaint()
	
	# 绘图区清除
	def button_Clear(self):
		sender=self.sender()
		print(self.button_clearSE,type(self.button_clearSE))
		if sender==self.button_clearSE:
			self.startPoint=None
			self.endPoint=None
			self.repaint()
			self.addDisplayText('清空起始点')
		elif sender==self.button_clearWall:
			for i in range(len(self.Map)):
				for j in range(len(self.Map[i])):
					self.Map[i][j]=0
			self.repaint()
			self.addDisplayText('清空所有图像')
	
	# 重写绘图类
	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self.drawBoard(event,qp)
		qp.end()
    
	# 重写绘图类
	def drawBoard(self, event, qp):
		self.drawMap(qp)

	# 绘图区颜色设置
	def drawMap(self,qp):#画面绘制方法，每次地图有所改动都将重绘
		time1=time.time()
		if self.search!=None:
			if self.special!=None:
				path = None
				if self.special[0]!=None:
					e=self.special[0]
					path=[e]
					while True:
						e=e.father
						if e!=None:
							path.append(e)
						else:
							break
			else:
				path=None
			pen=QPen(QColor(0,0,0),1,Qt.SolidLine)
			qp.setPen(pen)
			for i in range(len(self.Map)):
				for j in range(len(self.Map[i])):
					wordTag=False
					if i==self.search.start.x and j==self.search.start.y:   # 起始点颜色
						qp.setBrush(QColor(255,255,0))
					elif i==self.search.end.x and j==self.search.end.y:     # 终止点颜色
						qp.setBrush(QColor(100,200,50))
					else:
						if self.Map[i][j]==255:                             # 非障碍物颜色
							qp.setBrush(QColor(255, 255, 255))
						elif self.Map[i][j]==1 or self.Map[i][j]==0:                               #障碍物颜色
							qp.setBrush(QColor(0,0,0))
						elif self.Map[i][j]==100:                           # 左边线
							qp.setBrush(QColor(255,0,0))
						elif self.Map[i][j]==200:                           # 右边线
							qp.setBrush(QColor(0,255,0))
						elif self.Map[i][j]==150:                           # 中线
							qp.setBrush(QColor(0,0,255))
						else:
							qp.setBrush(QColor(200,200,230))
					qp.drawRect(30+j*config.blockLength,30+i*config.blockLength,config.blockLength,config.blockLength)
					if wordTag:
						qp.setFont(QFont('楷体',5,QFont.Light))
						qp.drawText(50+10+j*config.blockLength,50+10+i*config.blockLength,word)
						wordTag=False
		#time.sleep(20)
		else:
			for i in range(len(self.Map)):
				for j in range(len(self.Map[i])):
					if (j,i)==self.startPoint:
						qp.setBrush(QColor(255,255,0))
					elif (j,i)==self.endPoint:
						qp.setBrush(QColor(100,200,50))
					else:
						if self.Map[i][j]==255:
							qp.setBrush(QColor(255,255,255))
						elif self.Map[i][j]==1 or self.Map[i][j]==0:
							qp.setBrush(QColor(0,0,0))
						elif self.Map[i][j]==100:                           # 左边线
							qp.setBrush(QColor(255,0,0))
						elif self.Map[i][j]==200:                           # 右边线
							qp.setBrush(QColor(0,255,0))
						elif self.Map[i][j]==150:                           # 中线
							qp.setBrush(QColor(0,0,255))
						elif self.Map[i][j]==50:                            # 主跑行
							qp.setBrush(QColor(162,20,203))
						elif self.Map[i][j]==90:                            # 跳变点
							qp.setBrush(QColor(4,211,252))
						elif self.Map[i][j]==70:                           # 中线
							qp.setBrush(QColor(255,153,0))
						elif self.Map[i][j]==71:                           # 中线
							qp.setBrush(QColor(255,120,0))
						elif self.Map[i][j]==75:                           # 中线
							qp.setBrush(QColor(0,102,255))
						else:
							qp.setBrush(QColor(200,200,230))

					qp.drawRect(30+j*config.blockLength,30+i*config.blockLength,config.blockLength,config.blockLength)
		time2=time.time()

	# 未用到
	def timerEvent(self,e):
		try:
			data=next(self.yi)
		except Exception as e:
			self.addDisplayText('搜索结束:')
			print('搜索结束！')
			if self.search.result==None:
				self.addDisplayText('未找到可行路径')
				print('搜索结束！')
			else:
				self.addDisplayText('总计搜索节点数：%d'%self.search.count)
				self.addDisplayText('最终路径长度：%d'%len(self.search.result))
			self.centerTimer.stop()
			self.search=None
			self.yi=None
			self.special=None
			point.clear()
			self.button_start.setEnabled(True)
			self.button_clearSE.setEnabled(True)
			self.button_clearWall.setEnabled(True)
			self.displayFlush=True
		else:
			self.special=data
			self.repaint()

	# 定义将opencv图像转PyQt图像的函数
	def cvImgtoQtImg(self,cvImg):
		QtImgBuf = cv2.cvtColor(cvImg, cv2.COLOR_BGR2BGRA)
		QtImg = QtGui.QImage(QtImgBuf.data, QtImgBuf.shape[1], QtImgBuf.shape[0],QtGui.QImage.Format_RGB32)
		return QtImg

	# 选项框警告
	def checkRadioButton(self):
		if self.radioButton.isChecked():
			type = 1
		if self.radioButton2.isChecked():
			type = 2
		if self.radioButton3.isChecked():
			type = 3
		if self.radioButton4.isChecked():
			type = 4
		if self.radioButton5.isChecked():
			type = 5
		if self.radioButton6.isChecked():
			type = 6
		if not self.radioButton.isChecked() and not self.radioButton2.isChecked() and not self.radioButton3.isChecked() and not self.radioButton4.isChecked() and not self.radioButton5.isChecked() and not self.radioButton6.isChecked():
			QMessageBox.information(self, "消息框标题", "请选择一种方法", QMessageBox.Yes | QMessageBox.No)
			type=1
		return type

	# 播放视频进行二值化调试，与图片的思路基本一致
	def playVideoFile(self,fn):
		# 获取选项框type
		type=self.checkRadioButton()
		self.cap = cv2.VideoCapture(fn)
		# 设置滚动条对应的帧
		frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
		self.settingSlider(frames)
		# 帧数
		fps = 24
		self.loop_flag = 0
		if not self.cap.isOpened():
			print("Cannot open Video File")
			exit()
		while not self.bClose:
			# 逐帧读取影片
			ret, frame = self.cap.read()
			if not ret:
				if frame is None:
					print("The video has end.")
				else:
					print("Read video error!")
				break
			if self.moving_flag==0:
				# 设置时间
				self.label_start.setText(self.int2time(self.loop_flag))
				self.s1.setValue(int(self.loop_flag/fps))#设置当前值
			self.loop_flag += 1
			# 将cv2读取的图像显示在label中
			QtImg = self.cvImgtoQtImg(frame)
			self.label.setPixmap(QtGui.QPixmap.fromImage(QtImg).scaled(self.label.size()))
			self.label.show()  # 刷新界面
			global aa,bb,bc,side,md
			# 视频和图片的处理手法一致
			if(type==1):
				# 获取label上的图像，并进行一些数据转换
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				frame = np.array(gray, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_uint8*(120*188+1)
				t=a()
				mylib.Get_01_Value.restype=POINTER(c_uint8)
				t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),4)
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						aa[i][m]=t1[j]
						if(aa[i][m]==1):
							aa[i][m]=0
						if(aa[i][m]==255):
							aa[i][m]=1
						m=m+1
				# 在label中显示阈值
				self.label_4.setText(str(t1[120*188]))
				# 初始化用于判断边线的数组，存储在md中 120*160
				m=0
				for i in range(0,120):
					d=0
					for j in range(14,174):
						md2[i][d]=aa[i][j]
						d=d+1
					m=m+1
				# 转化为可以处理的数组
				frame = np.array(md2, dtype=np.uint8)
				# 将格式转换为c动态链接库的格式
				array = frame.astype(c_uint8)
				# 开辟存储空间
				a=c_int8*(150*3*2)
				# t指针指向存储空间
				t=a()
				# 函数的目标输出格式
				mylib2.Image_process.restype=POINTER(c_uint8)
				# 调用动态链接库函数
				t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				# 将输出量转化为py中的list，存储在side中
				m=0
				for i in range(0,300,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(300,600,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(600,900,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
			if(type==2):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				frame = np.array(gray, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_uint8*(120*188+1)
				t=a()
				mylib.Get_01_Value.restype=POINTER(c_uint8)
				t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),0)
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						aa[i][m]=t1[j]
						if(aa[i][m]==1):
							aa[i][m]=0
						if(aa[i][m]==255):
							aa[i][m]=1
						m=m+1
				# 在label中显示阈值
				self.label_4.setText(str(t1[120*188]))
				# 初始化用于判断边线的数组，存储在md中 120*160
				m=0
				for i in range(0,120):
					d=0
					for j in range(14,174):
						md2[i][d]=aa[i][j]
						d=d+1
					m=m+1
				# 转化为可以处理的数组
				frame = np.array(md2, dtype=np.uint8)
				# 将格式转换为c动态链接库的格式
				array = frame.astype(c_uint8)
				# 开辟存储空间
				a=c_int8*(150*3*2)
				# t指针指向存储空间
				t=a()
				# 函数的目标输出格式
				mylib2.Image_process.restype=POINTER(c_uint8)
				# 调用动态链接库函数
				t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				# 将输出量转化为py中的list，存储在side中
				m=0
				for i in range(0,300,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(300,600,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(600,900,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
			if(type==3):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				binary = threshold_demo(mat_img,self.horizontalSlider.value())
				self.label_4.setText(str(self.horizontalSlider.value()))
				aa = binary.tolist()
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						if(aa[i][m]==1):
							aa[i][m]=0
						if(aa[i][m]==255):
							aa[i][m]=1
						m=m+1
				# 在label中显示阈值
				# 初始化用于判断边线的数组，存储在md中 120*160
				m=0
				for i in range(0,120):
					d=0
					for j in range(14,174):
						md2[i][d]=aa[i][j]
						d=d+1
					m=m+1
				# 转化为可以处理的数组
				frame = np.array(md2, dtype=np.uint8)
				# 将格式转换为c动态链接库的格式
				array = frame.astype(c_uint8)
				# 开辟存储空间
				a=c_int8*(150*3*2)
				# t指针指向存储空间
				t=a()
				# 函数的目标输出格式
				mylib2.Image_process.restype=POINTER(c_uint8)
				# 调用动态链接库函数
				t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				# 将输出量转化为py中的list，存储在side中
				m=0
				for i in range(0,300,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(300,600,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(600,900,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
			if(type==4):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				#将图片数据转换为可处理的整型数据
				frame=gray.tolist
				frame = np.array(gray, dtype=np.float)
				sobel_motor_py(frame,aa,self.horizontalSlider2.value())
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						if(aa[i][m]==1):
							aa[i][m]=0
						if(aa[i][m]==255):
							aa[i][m]=1
						m=m+1
				# 在label中显示阈值
				# 初始化用于判断边线的数组，存储在md中 120*160
				m=0
				for i in range(0,120):
					d=0
					for j in range(14,174):
						md2[i][d]=aa[i][j]
						d=d+1
					m=m+1
				# 转化为可以处理的数组
				frame = np.array(md2, dtype=np.uint8)
				# 将格式转换为c动态链接库的格式
				array = frame.astype(c_uint8)
				# 开辟存储空间
				a=c_int8*(150*3*2)
				# t指针指向存储空间
				t=a()
				# 函数的目标输出格式
				mylib2.Image_process.restype=POINTER(c_uint8)
				# 调用动态链接库函数
				t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				# 将输出量转化为py中的list，存储在side中
				m=0
				for i in range(0,300,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(300,600,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(600,900,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
			if(type==5):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				frame=gray.tolist
				frame = np.array(gray, dtype=np.float)
				sobel_auto_py(frame,aa,self.horizontalSlider3.value()/10)
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						if(aa[i][m]==1):
							aa[i][m]=0
						if(aa[i][m]==255):
							aa[i][m]=1
						m=m+1
				# 在label中显示阈值
				# 初始化用于判断边线的数组，存储在md中 120*160
				m=0
				for i in range(0,120):
					d=0
					for j in range(14,174):
						md2[i][d]=aa[i][j]
						d=d+1
					m=m+1
				# 转化为可以处理的数组
				frame = np.array(md2, dtype=np.uint8)
				# 将格式转换为c动态链接库的格式
				array = frame.astype(c_uint8)
				# 开辟存储空间
				a=c_int8*(150*3*2)
				# t指针指向存储空间
				t=a()
				# 函数的目标输出格式
				mylib2.Image_process.restype=POINTER(c_uint8)
				# 调用动态链接库函数
				t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				# 将输出量转化为py中的list，存储在side中
				m=0
				for i in range(0,300,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(300,600,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(600,900,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1	
			if(type==6):
				imgptr = self.label.pixmap().toImage()
				ptr = imgptr.constBits()
				ptr.setsize(imgptr.byteCount())
				mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
				mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
				gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
				frame = np.array(gray, dtype=np.uint8)
				array = frame.astype(c_uint8)
				a=c_uint8*(120*188+1)
				t=a()
				mylib.Get_01_Value.restype=POINTER(c_uint8)
				t1=mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_uint8)),byref(t),1)
				for i in range(120):
					m=0
					for j in range(188*i,(i+1)*188):
						aa[i][m]=t1[j]
						if(aa[i][m]==1):
							aa[i][m]=0
						if(aa[i][m]==255):
							aa[i][m]=1
						m=m+1
				# 在label中显示阈值
				self.label_4.setText(str(t1[120*188]))
				# 初始化用于判断边线的数组，存储在md中 120*160
				m=0
				for i in range(0,120):
					d=0
					for j in range(14,174):
						md2[i][d]=aa[i][j]
						d=d+1
					m=m+1
				# 转化为可以处理的数组
				frame = np.array(md2, dtype=np.uint8)
				# 将格式转换为c动态链接库的格式
				array = frame.astype(c_uint8)
				# 开辟存储空间
				a=c_int8*(150*3*2)
				# t指针指向存储空间
				t=a()
				# 函数的目标输出格式
				mylib2.Image_process.restype=POINTER(c_uint8)
				# 调用动态链接库函数
				t1=mylib2.Image_process(array.ctypes.data_as(POINTER(c_uint8)),byref(t))
				# 将输出量转化为py中的list，存储在side中
				m=0
				for i in range(0,300,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(300,600,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
				for i in range(600,900,2):
					side2[m][0]=t1[i]
					side2[m][1]=t1[i+1]
					m=m+1
			
			for i in range(120):
				for j in range(0,14):
					self.Map[i][j]=aa[i][j]
				for j in range(174,188):
					self.Map[i][j]=aa[i][j]
			for i in range(120):
				dk=14
				for j in range(160):
					md2[i][j]=aa[i][dk]
					if(md2[i][j]==1):
						md2[i][j]=255
					dk=dk+1
			# 边线信息
			for i in range(150):
				# print(side2[i][0],side2[i][1])
				md2[side2[i][0]][side2[i][1]]=100 #左边线
			for i in range(150,300):
				md2[side2[i][0]][side2[i][1]]=200 #右边线
			for i in range(300,450):
				md2[side2[i][0]][side2[i][1]]=150 #中线
			for i in range(160):
				md2[ROAD_MAIN_ROW][i]=50 #主跑行
			zz=0
			dd=0
			for i in range(0,120):
				dd=0
				for j in range(14,174):
					self.Map[i][j]=md2[i][dd]
					dd=dd+1
				zz=zz+1
			config.HEIGHT=len(self.Map)
			config.WIDTH=len(self.Map[0])
			self.repaint()
			while self.stop_flag == 1:#暂停的动作
				cv2.waitKey(int(100/fps))#休眠一会，因为每秒播放24张图片，相当于放完一张图片后等待41ms
			cv2.waitKey(int(100/fps))#休眠一会，因为每秒播放24张图片，相当于放完一张图片后等待41ms
        #释放
		self.cap.release()

	# 暂停触发
	def stop_action(self):
		if self.stop_flag == 0:
			self.stop_flag = 1
			self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/暂停.png'))
		else:
			self.stop_flag = 0
			self.stop_button.setIcon(QIcon('C:/Users/17628/Desktop/ui/icon/开始.png'))

	# 下一个触发
	def next_action(self):
		self.bClose = True
		self.play_index = (self.play_index+1)%3
		self.bClose = False
		self.playVideoFile(self.fn_list[self.play_index])

	# 拖动触发
	def start_drag(self):
		self.moving_flag = 1

	# 拖动进度条行为
	def drag_action(self):
		self.moving_flag = 0
		print('当前进度为%d，被拉动到的进度为%d'%(self.s1.value(), int(self.loop_flag/24)))
		if self.s1.value()!=int(self.loop_flag/24):
			print('当前进度为:'+str(self.s1.value()))
			self.loop_flag = self.s1.value()*24
			self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.loop_flag)

	# 暂停触发
	def video_stop(self):
		self.bClose = True

	# 手动选择视频播放
	def manual_choose(self):
		fn,_ = QFileDialog.getOpenFileName(self,'Open file','C:/Users/17628/Desktop/ui/test video',"Video files (*.mp4 *.avi)")
		self.playVideoFile(fn)

	# 自动选择视频播放，如何自动视频播放？
	def auto_choose(self):
		self.playVideoFile(self.fn_list[self.play_index])
    
	# 设置进度条
	def settingSlider(self,maxvalue):
		self.s1.setMaximum(int(maxvalue/24))
		self.label_end.setText(self.int2time(maxvalue))

	# 视频播放的时间函数
	def int2time(self,num):
        #每秒刷新50帧
		num = int(num/24)
		minute = int(num/60)
		second = num - 60*minute
		if minute < 10:
			str_minute = '0'+str(minute)
		else:
			tr_minute = str(minute)
		if second < 10:
			str_second = '0'+str(second)
		else:
			str_second = str(second)
		return str_minute+":"+str_second

	# 停止行为
	def exit_stop(self):
		self.stop_flag = 0
		self.bClose = True
		self.close()

class main_window(QMainWindow):
	def __init__(self):
        # 1.初始化：初始化地图数组
		print('初始化主界面...')
		self.Map=[]
		for i in range(config.HEIGHT):
			col=[]
			for j in range(config.WIDTH):
				col.append(220)
			self.Map.append(col)
        # 2.初始化：起点和目的地
		self.startPoint=None
		self.endPoint=None
        # 3.初始化：其他参数
		self.search=None
		self.centerTimer=None
		self.yi=None
		self.special=None
		self.displayFlush=False
		super().__init__()
        # 4. 初始化：主界面
		print('初始化UI...')
		self.initUI()
	def initUI(self):
		#开始初始化UI部分
		font = QtGui.QFont()
		font.setFamily("Consolas")
		font.setPointSize(12)
		font.setBold(False)
		font.setItalic(False)
		font.setWeight(3)

		self.label=QLabel(self)
		self.label.setText("版本说明：添加八领域搜线算法框架\n          添加与动态链接库的交互\n设计团队：YYY")
		self.label.resize(300,100)
		self.label.move(50,130)

		self.mbtn = QPushButton(self)
		self.mbtn.setText("二值化工具")
		self.mbtn.move(150-80, 10)
		self.mbtn.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.mbtn.resize(160,40)
		self.mbtn.setFont(font)

		self.mbtn2 = QPushButton(self)
		self.mbtn2.setText("边线检测工具")
		self.mbtn2.move(150-80, 60)
		self.mbtn2.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.mbtn2.resize(160,40)
		self.mbtn2.setFont(font)

		self.mbtn3 = QPushButton(self)
		self.mbtn3.setText("运行监测工具")
		self.mbtn3.move(150-80, 110)
		self.mbtn3.setStyleSheet("QPushButton{background-color: rgb(200,200,230);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.mbtn3.resize(160,40)
		self.mbtn3.setFont(font)

		# UI初始化完成
		self.setGeometry(0, 0, 300, 210)
		self.setStyleSheet("background-color:#EDEDF5")   # 设置背景色
		self.setWindowTitle('YYY调试神器 V1.3')
		self.show()

# 全局阈值
def threshold_demo(image,th):
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	ret, binary = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY)
	print("阈值：", ret)
	return binary

# 局部阈值
def local_threshold(image):
	gray = cv2.cvtColor(image,cv2.COLOR_BGRA2GRAY)
	# binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,25,10)
	binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 25, 10)
	cv2.imshow("binary ", binary)

# sobel自动阈值
def sobel_auto_py(frame,xx,th):
	# 为了方便gui调参用python写了一遍
	KERNEL_SIZE = 3
	xStart = KERNEL_SIZE / 2
	xEnd = MT9V03X_W - KERNEL_SIZE / 2
	yStart = KERNEL_SIZE / 2
	yEnd = MT9V03X_H - KERNEL_SIZE / 2
	temp=[0,0,0,0]
	for i in range(int(yStart),int(yEnd)):
		for j in range(int(xStart),int(xEnd)):
			temp[0] = -int(frame[i - 1][j - 1]) + int(frame[i - 1][j + 1]) - int(frame[i][j - 1] )+ int(frame[i][j + 1]) - int(frame[i + 1][j - 1]) + int(frame[i + 1][j + 1])    
			temp[1] = -int(frame[i - 1][j - 1]) + int(frame[i + 1][j - 1]) - int(frame[i - 1][j]) + int(frame[i + 1][j]) - int(frame[i - 1][j + 1]) + int(frame[i + 1][j + 1])     
			temp[2] = -int(frame[i - 1][j]) + int(frame[i][j - 1])- int(frame[i][j + 1]) + int(frame[i + 1][j])	- int(frame[i - 1][j + 1]) + int(frame[i + 1][j - 1])				   
			temp[3] = -int(frame[i - 1][j]) + int(frame[i][j + 1])- int(frame[i][j - 1]) + int(frame[i + 1][j]) - int(frame[i - 1][j - 1]) + int(frame[i + 1][j + 1])          
			temp[0] = abs(temp[0])
			temp[1] = abs(temp[1])
			temp[2] = abs(temp[2])
			temp[3] = abs(temp[3])
			# /* 找出梯度幅值最大值  */
			for k in range(1,4):
				if (temp[0] < temp[k]):
					temp[0] = temp[k]
			# /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
			temp[3] = int(frame[i - 1][j - 1]) + int(frame[i - 1][j]) + int(frame[i - 1][j + 1])+ int(frame[i][j - 1]) + int(frame[i][j]) + int(frame[i][j + 1])+ int(frame[i + 1][j - 1]) + int(frame[i + 1][j]) + int(frame[i + 1][j + 1])
			if(temp[0]>temp[3]/(th)):
				xx[i][j]=1
			else:
				xx[i][j]=255

# sobel手动阈值
def sobel_motor_py(frame,xx,th):
	# 为了方便gui调参用python写了一遍
	KERNEL_SIZE = 3
	xStart = KERNEL_SIZE / 2
	xEnd = MT9V03X_W - KERNEL_SIZE / 2
	yStart = KERNEL_SIZE / 2
	yEnd = MT9V03X_H - KERNEL_SIZE / 2
	temp=[0,0,0,0]
	for i in range(int(yStart),int(yEnd)):
		for j in range(int(xStart),int(xEnd)):
			temp[0] = -int(frame[i - 1][j - 1]) + int(frame[i - 1][j + 1]) - int(frame[i][j - 1] )+ int(frame[i][j + 1]) - int(frame[i + 1][j - 1]) + int(frame[i + 1][j + 1])    
			temp[1] = -int(frame[i - 1][j - 1]) + int(frame[i + 1][j - 1]) - int(frame[i - 1][j]) + int(frame[i + 1][j]) - int(frame[i - 1][j + 1]) + int(frame[i + 1][j + 1])     
			temp[2] = -int(frame[i - 1][j]) + int(frame[i][j - 1])- int(frame[i][j + 1]) + int(frame[i + 1][j])	- int(frame[i - 1][j + 1]) + int(frame[i + 1][j - 1])				   
			temp[3] = -int(frame[i - 1][j]) + int(frame[i][j + 1])- int(frame[i][j - 1]) + int(frame[i + 1][j]) - int(frame[i - 1][j - 1]) + int(frame[i + 1][j + 1])          
			temp[0] = abs(temp[0])
			temp[1] = abs(temp[1])
			temp[2] = abs(temp[2])
			temp[3] = abs(temp[3])
			# /* 找出梯度幅值最大值  */
			for k in range(1,4):
				if (temp[0] < temp[k]):
					temp[0] = temp[k]
			if(temp[0]>th):
				xx[i][j]=1
			else:
				xx[i][j]=255

# 图像滤波
def filter_py(xx,yy,fil):	
	temp=0
	for i in range(MT9V03X_H-1):
		for j in range(MT9V03X_W-1):
			temp = int(xx[i - 1][j - 1]) + int(xx[i - 1][j]) + int(xx[i - 1][j + 1]) +int(xx[i][j - 1]) + int(xx[i][j]) + int(xx[i][j + 1]) +int(xx[i + 1][j - 1]) + int(xx[i + 1][j]) + int(xx[i + 1][j + 1])
			# /* 邻域内5个点是边沿 则保留该点 可以调节这里优化滤波效果 */
			if (temp>fil):
				yy[i][j]=255
			else:
				yy[i][j]=1

# 由边线创建cv2格式图像，三通道
def create_image(cc):
    # 多通道8位字节的图像创建
    img = np.zeros([120, 188, 3], np.uint8)   # 创建一个图片，规定长、宽、通道数 数值，类型
    img[:, :, 0] = np.ones([120, 188]) * 255   # 设定第1个通道数值
    img[:, :, 1] = np.ones([120, 188]) * 255   # 设定第2个通道数值
    img[:, :, 2] = np.ones([120, 188]) * 255   # 设定第3个通道数值
    m=0
    for i in range(56,120):
        # for j in range(188):
        img[i][cc[m][0]][0]=214
        img[i][cc[m][0]][1]=120
        img[i][cc[m][0]][2]=42

        img[i][cc[m][1]][0]=188
        img[i][cc[m][1]][1]=31
        img[i][cc[m][1]][2]=225

        img[i][cc[m][2]][0]=69
        img[i][cc[m][2]][1]=201
        img[i][cc[m][2]][2]=55
        m=m+1
    return img

# 由边线创建cv2格式图像，单通道
def create_image_singel(cc):
    # 多通道8位字节的图像创建
    img = np.zeros([120, 188, 1], np.uint8)   # 创建一个图片，规定长、宽、通道数 数值，类型
    img[:, :, 0] = np.ones([120, 188]) * 255   # 设定第1个通道数值
    for i in range(120):
        m=0
        for j in range(188):
            img[i][j][0]=cc[i][j]
        m=m+1
    return img

def adaptiveThreshold(img,out,block,width,height,clip):
	half_block = int (block / 2)
	for x in range(half_block,height-half_block):
		for y in range (half_block,width-half_block):
			thres = 0
			for dx in range(-half_block,half_block+1):
				for dy in range(-half_block,half_block+1):
					thres = thres + img[x+dx][y+dy]
			thres = thres / (block * block) - clip
			if(img[x][y]>thres):
				out[x][y]=255
			else:
				out[x][y]=0


if __name__ == '__main__':
	app = QApplication(sys.argv)
	main_win=main_window()
	ex = board_widget()
	newWin = WinForm()
	newWin2 = seed()
	main_win.mbtn.clicked.connect(ex.show)
	main_win.mbtn2.clicked.connect(newWin2.show)
	main_win.mbtn3.clicked.connect(newWin.show)
	# ex.collec_btn2.clicked.connect(newWin2.show)
	# ex.collec_btn.clicked.connect(newWin.show)
	sys.exit(app.exec_())
 


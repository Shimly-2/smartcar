# -*- coding:utf-8 -*-
import time,sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtWidgets, QtCore, QtGui
import json
import cv2
import os
import numpy as np
from ctypes import *

from PIL import Image

mylib = cdll.LoadLibrary(r'C:/Users/17628/Desktop/ui/dll/x64/Debug/dll.dll')

ROAD_MAIN_ROW = 120-40

class StructPointer(Structure):  
    _fields_ = [("arr_left", c_int * 120), 
                ("arr_right", c_int * 120)] 

class config:
	WIDTH=188							#地图列数
	HEIGHT=120							#地图行数
	blockLength=8						#绘制画面时每一个节点方块的边长
	blockLength2=3						#绘制画面时每一个节点方块的边长

class point:							#点类（每一个唯一坐标只有对应的一个实例）
	_list=[]							#储存所有的point类实例
	_tag=True							#标记最新创建的实例是否为_list中的已有的实例，True表示不是已有实例
	def __new__(cls,x,y):				#重写new方法实现对于同样的坐标只有唯一的一个实例
		for i in point._list:
			if i.x==x and i.y==y:
				point._tag=False
				return i
		nt=super(point,cls).__new__(cls)
		point._list.append(nt)
		return nt
	def __init__(self,x,y):
		if point._tag:
			self.x=x
			self.y=y
			self.father=None
			self.F=0					#当前点的评分  F=G+H
			self.G=0					#起点到当前节点所花费的消耗
			self.cost=0					#父节点到此节点的消耗
		else:
			point._tag=True
	@classmethod
	def clear(cls):						#clear方法，每次搜索结束后，将所有点数据清除，以便进行下一次搜索的时候点数据不会冲突。
		point._list=[]
	def __eq__(self,T):					#重写==运算以便实现point类的in运算
		if type(self)==type(T):
			return (self.x,self.y)==(T.x,T.y)
		else:
			return False
	def __str__(self):
		return'(%d,%d)[F=%d,G=%d,cost=%d][father:(%s)]'%(self.x,self.y,self.F,self.G,self.cost,str((self.father.x,self.father.y)) if self.father!=None else 'null')

class A_Search:							#核心部分，寻路类
	def __init__(self,arg_start,arg_end,arg_map):
		self.start=arg_start			#储存此次搜索的开始点
		self.end=arg_end				#储存此次搜索的目的点
		self.Map=arg_map				#一个二维数组，为此次搜索的地图引用
		self.open=[]					#开放列表：储存即将被搜索的节点
		self.close=[]					#关闭列表：储存已经搜索过的节点
		self.result=[]					#当计算完成后，将最终得到的路径写入到此属性中
		self.count=0					#记录此次搜索所搜索过的节点数
		self.useTime=0					#记录此次搜索花费的时间--在此演示中无意义，因为process方法变成了一个逐步处理的生成器，统计时间无意义。
										#开始进行初始数据处理
		self.open.append(arg_start)
	def cal_F(self,loc):
		print('计算值：',loc)
		G=loc.father.G+loc.cost
		H=self.getEstimate(loc)
		F=G+H
		print("F=%d G=%d H=%d"%(F,G,H))
		return {'G':G,'H':H,'F':F}
	def F_Min(self):					#搜索open列表中F值最小的点并将其返回，同时判断open列表是否为空，为空则代表搜索失败
		if len(self.open)<=0:
			return None
		t=self.open[0]
		for i in self.open:
			if i.F<t.F:
				t=i
		return t
	def getAroundPoint(self,loc):#获取指定点周围所有可通行的点，并将其对应的移动消耗进行赋值。
		l=[(loc.x,loc.y+1,10),(loc.x+1,loc.y+1,14),(loc.x+1,loc.y,10),(loc.x+1,loc.y-1,14),(loc.x,loc.y-1,10),(loc.x-1,loc.y-1,14),(loc.x-1,loc.y,10),(loc.x-1,loc.y+1,14)]
		for i in l[::-1]:
			if i[0]<0 or i[0]>=config.HEIGHT or i[1]<0 or i[1]>=config.WIDTH:
				l.remove(i)
		nl=[]
		for i in l:
			if self.Map[i[0]][i[1]]==0:
				nt=point(i[0],i[1])
				nt.cost=i[2]
				nl.append(nt)
		return nl
 
	def addToOpen(self,l,father):#此次判断的点周围的可通行点加入到open列表中，如此点已经在open列表中则对其进行判断，如果此次路径得到的F值较之之前的F值更小，则将其父节点更新为此次判断的点，同时更新F、G值。
		for i in l:
			if i not in self.open:
				if i not in self.close:
					i.father=father
					self.open.append(i)
					r=self.cal_F(i)
					i.G=r['G']
					i.F=r['F']
			else:
				tf=i.father
				i.father=father
				r=self.cal_F(i)
				if i.F>r['F']:
					i.G=r['G']
					i.F=r['F']
					# i.father=father
				else:
					i.father=tf
	def getEstimate(self,loc):#H :从点loc移动到终点的预估花费
		return (abs(loc.x-self.end.x)+abs(loc.y-self.end.y))*10
	def DisplayPath(self):#在此演示中无意义
		print('搜索花费的时间:%.2fs.迭代次数%d,路径长度:%d'%(self.useTime,self.count,len(self.result)))
		if self.result!=None:
			for i in self.result:
				self.Map[i.x][i.y]=8
			for i in self.Map:
				for j in i:
					if j==0:
						print('%s'%'□',end='')
					elif j==1:
						print('%s'%'▽',end='')
					elif j==8:
						print('%s'%'★',end='')
				print('')
		else:
			print('搜索失败，无可通行路径')
	def process(self):#使用yield将process方法变成一个生成器，可以逐步的对搜索过程进行处理并返回关键数据
		while True:
			self.count+=1
			tar=self.F_Min()#先获取open列表中F值最低的点tar
			print('[debug]',tar)
			if tar==None:
				self.result=None
				self.count=-1
				break
			else:
				aroundP=self.getAroundPoint(tar)#获取tar周围的可用点列表aroundP
				self.addToOpen(aroundP,tar)#把aroundP加入到open列表中并更新F值以及设定父节点
				self.open.remove(tar)#将tar从open列表中移除
				self.close.append(tar)#已经迭代过的节点tar放入close列表中
				if self.end in self.open:#判断终点是否已经处于open列表中
					e=self.end
					self.result.append(e)
					while True:
						e=e.father
						if e==None:
							break
						self.result.append(e)
					yield (tar,self.open,self.close)
					break
 
			# self.repaint()
			# print('返回')
			yield (tar,self.open,self.close)
			#time.sleep(1)#暂停

class board_widget(QMainWindow):#可视化类，pyqt5进行编写。
	def __init__(self):
        # 1.初始化：初始化地图数组
		print('初始化地图...')
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
			#创建UI控件
		self.label_tips=QLabel("<p style='color:green'>使用说明：</p>右键单击格子选定起始点,左键格子选定格子为墙壁或删除墙壁。\n<p style='color:green'>颜色说明：</p>\n黄色代表起点，绿色代表终点，黑色代表墙壁，红色代表待搜索的open列表，灰色代表已搜索过的close列表，蓝色代表当前搜索到的路径",self)
		self.label_display=QTextEdit("",self)
		self.button_start=QPushButton("开始巡线",self)
		self.button_clearSE=QPushButton("重选起始点",self)
		self.button_clearWall=QPushButton("清空地图墙壁",self)
		self.button_saveMap=QPushButton("保存地图",self)
		self.button_loadMap=QPushButton("加载地图",self)

		self.label_1=QLabel("全局阈值",self)
		self.label_2=QLabel("130",self)
		font = QtGui.QFont()
		font.setFamily("Consolas")
		font.setPointSize(12)
		font.setBold(False)
		font.setItalic(False)
		font.setWeight(3)
		self.label_2.setFont(font)

		self.collec_btn = QPushButton('调试小摩托', self)
		self.collec_btn.setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.collec_btn.move(1600, 10)

		self.label = QLabel(self)
		self.label.setText("显示图片")
		self.label.setFixedSize(188, 120)
		self.label.move(1600, 100)

		self.label_3 = QLabel(self)
		self.label_3.setText("OTSU阈值：")
		self.label_3.setFixedSize(50, 50)
		self.label_3.move(1830, 120)

		self.label_4 = QLabel(self)
		self.label_4.setText("")
		self.label_4.setFixedSize(50, 50)
		self.label_4.move(1850, 160)

		self.btn = QPushButton(self)
		self.btn.setText("打开图片")
		self.btn.move(1600, 50)
		self.btn.setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
		self.btn.clicked.connect(self.openimage)

		self.horizontalSlider =QSlider(self)
		self.horizontalSlider.setMaximum(255)
		self.horizontalSlider.setSingleStep(1)
		self.horizontalSlider.setPageStep(1)
		self.horizontalSlider.setProperty("value", 130)
		self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
		self.horizontalSlider.setObjectName("horizontalSlider")
		self.horizontalSlider.setTickInterval(5)#设置刻度间隔
		self.horizontalSlider.valueChanged.connect(self.valChange)

		#设置控件属性
		self.label_tips.setWordWrap(True)
		self.label_2.setWordWrap(True)
		self.label_1.setWordWrap(True)
		#设置控件样式
		self.label_display.setStyleSheet("border:2px solid black")
		self.label_display.setAlignment(Qt.AlignLeft)
		self.label_display.setAlignment(Qt.AlignTop)
		#设置控件的尺寸和位置
        
		self.label_tips.resize(330,400)
		self.label_2.resize(50,50)
		self.label_1.resize(50,50)
		self.horizontalSlider.resize(200,30)
		self.button_saveMap.resize(120,40)
		self.button_loadMap.resize(150,40)
		self.button_start.resize(120,40)
		self.button_clearSE.resize(150,40)
		self.button_clearWall.resize(150,40)
		self.label_display.resize(300,120)
 
		self.label_2.move(1870,240)
		self.label_1.move(1590,240)
		self.horizontalSlider.move(1650,250)
		self.label_tips.move(90+(config.WIDTH-1)*config.blockLength,200)
		self.label_display.move(90+(config.WIDTH-1)*config.blockLength,740)
		self.button_start.move(90+(config.WIDTH-1)*config.blockLength,580)
		self.button_clearSE.move(230+(config.WIDTH-1)*config.blockLength,580)
		self.button_clearWall.move(90+(config.WIDTH-1)*config.blockLength,680)
		self.button_saveMap.move(90+(config.WIDTH-1)*config.blockLength,630)
		self.button_loadMap.move(230+(config.WIDTH-1)*config.blockLength,630)
			#给控件绑定事件
		self.button_start.clicked.connect(self.button_StartEvent)
		self.button_clearSE.clicked.connect(self.button_Clear)
		self.button_clearWall.clicked.connect(self.button_Clear)
		self.button_saveMap.clicked.connect(self.button_SaveMap)
		self.button_loadMap.clicked.connect(self.button_LoadMap)
		#UI初始化完成
		self.setGeometry(0, 0, 1920, 1080)
		# self.setGeometry(0, 0, 230+(config.WIDTH*config.blockLength-config.blockLength)+200, 130+(config.HEIGHT*config.blockLength-config.blockLength))
		# self.setMinimumSize(230+(config.WIDTH*config.blockLength-config.blockLength)+200, 130+(config.HEIGHT*config.blockLength-config.blockLength))
		# self.setMaximumSize(230+(config.WIDTH*config.blockLength-config.blockLength)+200, 130+(config.HEIGHT*config.blockLength-config.blockLength))
		self.setWindowTitle('小摩托')
		self.show()

	def valChange(self):
		self.label_2.setNum(self.horizontalSlider.value())

	def openimage(self):
		imgName, imgType = QFileDialog.getOpenFileName(self, "打开图片", "", "*.bmp;;*.jpg;;*.png;;All Files(*)")
		jpg = QtGui.QPixmap(imgName).scaled(self.label.width(), self.label.height())
		self.label.setPixmap(jpg)

	def addDisplayText(self,text):
		if self.displayFlush:
			self.label_display.setText(text+'\n')
			self.displayFlush=False
		else:
			self.label_display.setText(self.label_display.toPlainText()+text+'\n')

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

	def button_StartEvent(self):
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
		img_list = binary.tolist()
		for i in range(120):
			img_list[i][aa[i][0]]=100 #左边线
			img_list[i][aa[i][1]]=200 #右边线
			img_list[i][int((aa[i][0]+aa[i][1])/2)]=150 #中线
		for i in range(188):
			img_list[ROAD_MAIN_ROW][i]=50 #主跑行
		with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
			f.write(json.dumps(img_list))
		with open('C:/Users/17628/Desktop/map2.txt','r') as f:
			self.Map=json.loads(f.read())
			config.HEIGHT=len(self.Map)
			config.WIDTH=len(self.Map[0])
			self.addDisplayText('地图加载成功')
			self.repaint()
		self.addDisplayText('开始进行巡线')

	def button_SaveMap(self):
		path = 'C:/Users/17628/Desktop/'
		# path = 'E:/智能车竞赛/智能车/考核要求_10_30/软件硬件=金屹阳 1950049/Demo/Demo/'
		img = cv2.imread(os.path.join(path, '3.BMP'), 0)
		#cv2.imshow(img)
		binary = threshold_demo(img)
		# numpy中ndarray文件转为list
		img_list = binary.tolist()
		#print(binary)
		with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
			f.write(json.dumps(img_list))
			self.addDisplayText('地图保存成功-->map.txt')

	def button_LoadMap(self):
		try:
			imgptr = self.label.pixmap().toImage()
			ptr = imgptr.constBits()
			ptr.setsize(imgptr.byteCount())
			mat = np.array(ptr).reshape(imgptr.height(), imgptr.width(), 4)
			mat_img = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)

			# TODO
			# gray = cv2.cvtColor(mat_img, cv2.COLOR_RGB2GRAY)
			# print(gray)
			# #将图片数据转换为可处理的整型数据
			# frame = np.array(gray, dtype=np.uint8)
			# array = frame.astype(c_uint8)
			# array = array.tolist()
			# matrix1 = mat(array)     # mat()函数把数组转化为矩阵
			# matrix1=np.array(matrix1,dtype=np.uint8)
			# a=c_uint8*120*188
			# t=a()
			# mylib.Get_01_Value.restype=c_char_p  
			# t1=mylib.Get_01_Value(matrix1.ctypes.data_as(POINTER(c_uint8)),byref(t),0)
			# aa=[]
			# m=0
			# # print(t)
			# for i in range(120):
			# 	aa.append([])
			# 	for j in range(188):
			# 		aa[i].append(t1[j*i])
			# 		m=m+1

			# print(aa)
			# frame = np.array(mat_img, dtype=np.float32)
			# array = frame.astype(int)
			# git = frame.astype(int)
			# th = mylib.Get_01_Value(array.ctypes.data_as(POINTER(c_int)),git.ctypes.data_as(POINTER(c_int)),0)
			
			binary = threshold_demo(mat_img,self.horizontalSlider.value())
			th = mylib.Get_01_Value(mat_img.ctypes.data_as(POINTER(c_int)) , 0) #0:龙邱OTSU，1：平均阈值，4：OTSU
			self.label_4.setText(str(th))
			# numpy中ndarray文件转为list
			img_list = binary.tolist()
			with open('C:/Users/17628/Desktop/map2.txt', 'w') as f:
				f.write(json.dumps(img_list))
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
			self.addDisplayText('清空所有墙壁')
	
	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)
		self.drawBoard(event,qp)
		qp.end()
    
	def drawBoard(self, event, qp):
		self.drawMap(qp)
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
					if i==self.search.start.x and j==self.search.start.y:   #起始点颜色
						qp.setBrush(QColor(255,255,0))
					elif i==self.search.end.x and j==self.search.end.y:     #终止点颜色
						qp.setBrush(QColor(100,200,50))
					else:
						if self.Map[i][j]==255:                               #非障碍物颜色
							qp.setBrush(QColor(255, 255, 255))
						elif self.Map[i][j]==1 or self.Map[i][j]==0:                               #障碍物颜色
							qp.setBrush(QColor(0,0,0))
						elif self.Map[i][j]==100:                           #左边线
							qp.setBrush(QColor(255,0,0))
						elif self.Map[i][j]==200:                           #右边线
							qp.setBrush(QColor(0,255,0))
						elif self.Map[i][j]==150:                           #中线
							qp.setBrush(QColor(0,0,255))
						else:
							qp.setBrush(QColor(220,220,220))
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
						elif self.Map[i][j]==100:                           #左边线
							qp.setBrush(QColor(255,0,0))
						elif self.Map[i][j]==200:                           #右边线
							qp.setBrush(QColor(0,255,0))
						elif self.Map[i][j]==150:                           #中线
							qp.setBrush(QColor(0,0,255))
						elif self.Map[i][j]==50:                           #主跑行
							qp.setBrush(QColor(162,20,203))
						else:
							qp.setBrush(QColor(220,220,220))

					qp.drawRect(30+j*config.blockLength,30+i*config.blockLength,config.blockLength,config.blockLength)
		time2=time.time()
	#time.sleep(20)
		# print('绘制时间：',time2-time1)
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

class WinForm(QGraphicsView,QMainWindow):
    def __init__(self, parent=None):
        super(WinForm, self).__init__(parent)
        self.setWindowTitle('rotation animation')  # 设置窗口标题
        self.setWindowTitle("rotation animation")
        self.setGeometry(0, 0, 1920, 1080)  # 窗口整体窗口位置大小

        quit = QPushButton('click', self)  # button 对象
        quit.setGeometry(10, 10, 60, 35)  # 设置按钮的位置 和 大小
        quit.setStyleSheet("background-color: red")  # 设置按钮的风格和颜色
        quit.clicked.connect(self.clickbutton)  # 点击按钮之后关闭窗口

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

        self.setScene(self.scene)
        self.stop = False

        self.label = QLabel(self)
        self.label.setText("显示图片")
        self.label.setFixedSize(188*3, 120*3)
        self.label.move(10, 100)

        self.label_13 = QLabel(self)
        self.label_13.setText("显示图片")
        self.label_13.setFixedSize(188*3, 120*3)
        self.label_13.move(10, 500)
        
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
        self.label_1.move(1550, 10)
        self.label_1.setFont(font)

        self.label_7 = QLabel(self)
        self.label_7.setText("40")
        self.label_7.setFixedSize(300, 40)
        self.label_7.move(1680, 10)
        self.label_7.setFont(font2)

        self.label_2 = QLabel(self)
        self.label_2.setText("舵机转角实际值：            度")
        self.label_2.setFixedSize(300, 40)
        self.label_2.move(1150, 10)
        self.label_2.setFont(font)

        self.label_8 = QLabel(self)
        self.label_8.setText("40")
        self.label_8.setFixedSize(150, 40)
        self.label_8.move(1280, 10)
        self.label_8.setFont(font2)

        self.label_3 = QLabel(self)
        self.label_3.setText("倾角实际值：            度")
        self.label_3.setFixedSize(300, 40)
        self.label_3.move(1150, 275)
        self.label_3.setFont(font)

        self.label_9 = QLabel(self)
        self.label_9.setText("40")
        self.label_9.setFixedSize(150, 40)
        self.label_9.move(1250, 275)
        self.label_9.setFont(font2)

        self.label_4 = QLabel(self)
        self.label_4.setText("倾角理论值：            度")
        self.label_4.setFixedSize(300, 40)
        self.label_4.move(1550, 275)
        self.label_4.setFont(font)

        self.label_10 = QLabel(self)
        self.label_10.setText("40")
        self.label_10.setFixedSize(150, 40)
        self.label_10.move(1650, 275)
        self.label_10.setFont(font2)

        self.label_5 = QLabel(self)
        self.label_5.setText("转速实际值：            m/s")
        self.label_5.setFixedSize(300, 40)
        self.label_5.move(1150, 675)
        self.label_5.setFont(font)

        self.label_11 = QLabel(self)
        self.label_11.setText("40")
        self.label_11.setFixedSize(150, 40)
        self.label_11.move(1250, 675)
        self.label_11.setFont(font2)

        self.label_6 = QLabel(self)
        self.label_6.setText("转速理论值：            m/s")
        self.label_6.setFixedSize(300, 40)
        self.label_6.move(1550, 675)
        self.label_6.setFont(font)

        self.label_12 = QLabel(self)
        self.label_12.setText("40")
        self.label_12.setFixedSize(150, 40)
        self.label_12.move(1650, 675)
        self.label_12.setFont(font2)

        self.btn = QPushButton(self)
        self.btn.setText("打开图片")
        self.btn.move(100, 10)
        self.btn.setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
        self.btn.clicked.connect(self.openimage)
        self.btn.resize(120,40)

        self.btn1 = QPushButton(self)
        self.btn1.setText("计算角度")
        self.btn1.move(250, 10)
        self.btn1.setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
                                           "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
                                           "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}")
        self.btn1.clicked.connect(self.startcal)
        self.btn1.resize(120,40)
    
    def startcal(self):
        a=c_float*2
        t=a()
        mylib.control.restype=POINTER(c_float)
        omega=c_float(1.0)
        t1=mylib.control(omega,20,byref(t),0)
        self.label_7.setText(str(round(t1[0],2)))
        self.rota_top.car_top_item.setRotation(t1[0])  # 自身改变旋转度
        self.label_10.setText(str(round(t1[1],2)))
        self.rota.car_back_item.setRotation(t1[1])  # 自身改变旋转度
        print(t1[0],t1[1])

    def openimage(self):
        imgName, imgType = QFileDialog.getOpenFileName(self, "打开图片", "", "*.bmp;;*.jpg;;*.png;;All Files(*)")
        jpg = QtGui.QPixmap(imgName).scaled(self.label.width(), self.label.height())
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

        for i in range(120):
            binary[i*3][int((aa[i][0]+aa[i][1])/2)*3]=20 #中线
        for i in range(119):
            binary[i*3+1][int((aa[i][0]+aa[i][1])/2)*3]=20 #中线
        for i in range(119):
            binary[i*3+2][int((aa[i][0]+aa[i][1])/2)*3]=20 #中线
        for i in range(188):
            binary[ROAD_MAIN_ROW*3][i*3]=20 #主跑行
        binary = Image.fromarray(np.uint8(binary))
        binary = binary.toqpixmap() #QPixmap
        self.label_13.setPixmap(binary)

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

        # self.rota.anim.stop() #动画停止
        # self.scene.deleteLater() #删除加载的动画


class Rotation(QObject):
	def __init__(self):
		super().__init__()
		car_back = QPixmap("car_back.png")
		scaledPixmap = car_back.scaled(261/1.5, 458/1.5)  # 动画大小
		self.animation()
		self.car_back_item = QGraphicsPixmapItem(scaledPixmap)
		self.car_back_item.setPos(1600,350)
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


class Rotation_top(QObject):
	def __init__(self):
		super().__init__()
		car_top = QPixmap("car_top.png")
		scaledPixmap2 = car_top.scaled(220/1.5, 247/1.5)  # 动画大小
		self.animation()
		self.car_top_item = QGraphicsPixmapItem(scaledPixmap2)
		self.car_top_item.setPos(1607,80)
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

class Rotation_tyre(QObject):
	def __init__(self):
		super().__init__()
		car_tyre = QPixmap("car_tyre.png")
		scaledPixmap2 = car_tyre.scaled(236/1.5, 236/1.5)  # 动画大小
		self.animation()
		self.car_tyre_item = QGraphicsPixmapItem(scaledPixmap2)
		self.car_tyre_item.setPos(1607,750)
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

class Rotation_real(QObject):
	def __init__(self):
		super().__init__()
		car_back_real = QPixmap("car_back.png")
		scaledPixmap = car_back_real.scaled(261/1.5, 458/1.5)  # 动画大小
		self.animation()
		self.car_back_real_item = QGraphicsPixmapItem(scaledPixmap)
		self.car_back_real_item.setPos(1200,350)
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


class Rotation_top_real(QObject):
	def __init__(self):
		super().__init__()
		car_top_real = QPixmap("car_top.png")
		scaledPixmap2 = car_top_real.scaled(220/1.5, 247/1.5)  # 动画大小
		self.animation()
		self.car_top_real_item = QGraphicsPixmapItem(scaledPixmap2)
		self.car_top_real_item.setPos(1207,80)
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

class Rotation_tyre_real(QObject):
	def __init__(self):
		super().__init__()
		car_tyre_real = QPixmap("car_tyre.png")
		scaledPixmap2 = car_tyre_real.scaled(236/1.5, 236/1.5)  # 动画大小
		self.animation()
		self.car_tyre_real_item = QGraphicsPixmapItem(scaledPixmap2)
		self.car_tyre_real_item.setPos(1207,750)
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


# 全局阈值
def threshold_demo(image,th):
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	ret, binary = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY)
	print("阈值：", ret)
	#cv2.imshow("binary", binary)
	return binary

# 局部阈值
def local_threshold(image):
	gray = cv2.cvtColor(image,cv2.COLOR_BGRA2GRAY)
	# binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,25,10)
	binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 25, 10)
	cv2.imshow("binary ", binary)

if __name__ == '__main__':
	app = QApplication(sys.argv)
	ex = board_widget()
	newWin = WinForm()
	ex.collec_btn.clicked.connect(newWin.show)
	sys.exit(app.exec_())
 


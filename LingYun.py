# -- coding: utf-8 --
# 程序更新说明：添加上位机程序，（usb交互）
#更新说明:调整参数九米内准确定位

import sys
import copy
import os
import termios
import time
import serial
import json
import string
from ctypes import *
import cv2
import numpy as np
sys.path.append("/home/lingyun/Documents/work/MvImport")
from MvCameraControl_class import *

global cam

winfun_ctype = CFUNCTYPE

stFrameInfo = POINTER(MV_FRAME_OUT_INFO_EX)
pData = POINTER(c_ubyte)
FrameInfoCallBack = winfun_ctype(None, pData, stFrameInfo, c_void_p)
def usb_start():
	coordinate = (2,0,0,0,0,0,0)
	serial.write(coordinate)
	data = serial.read(3)
	c =str(data,encoding='utf-8')
	c1 = c.split(",")[0]
	c2 = c.split(",")[-1]
	return c1,c2
def usb_1(x,y):
	x = str(x)
	y = str(y)
	
	x1 = x.split(".")[0]
	x1 = int(x1)
	x2 = x.split(".")[-1]
	x2 = int(x2)
	y1 = y.split(".")[0]
	y1 = int(y1)
	y2 = y.split(".")[-1]
	y2 = int(y2)


	cc_1000 = x1//100
	x1 %=100
	cc_100 = x1


	mm_1000 = y1//100
	y1 %=100
	mm_100 = y1
	
	coordinate = (1,cc_1000,cc_100,x2,mm_1000,mm_100,y2)
	print(coordinate)
	serial.write(coordinate)
	# data = serial.read(3)
	# c =str(data,encoding='utf-8')
	# c1 = c.split(",")[0]
	# c2 = c.split(",")[-1]
	# if c1 == '3':
	# 	pass
	# else:
	# 	print("usb_error")

	
def usb_0():
	coordinate = (3,0,0,0,0,0,0)
	serial.write(coordinate)
	# data = serial.read(3)
	# c =str(data,encoding='utf-8')
	# c1 = c.split(",")[0]
	# c2 = c.split(",")[-1]
	# if c1 == '3':
	# 	pass
	# else:
	# 	print("usb_error")


def image_callback_red(pData, pFrameInfo, pUser):
	# 己方红色对方蓝色
	start = time.time()
	stFrameInfo = cast(pFrameInfo, POINTER(MV_FRAME_OUT_INFO_EX)).contents
	if stFrameInfo:
		print ("第 %s 帧" % (stFrameInfo.nFrameNum))
		m_pBufForSaveImage = None
		file_path = "/home/lingyun/Documents/work/main.jpg"
		nRGBSize = stFrameInfo.nWidth * stFrameInfo.nHeight * 3+ 2048
		if m_pBufForSaveImage is None:
			m_pBufForSaveImage = (c_ubyte * nRGBSize)()
		stParam = MV_SAVE_IMAGE_PARAM_EX()
		stParam.enImageType = MV_Image_Jpeg;stParam.nWidth = 200  # ch:相机对应的宽 | en:Width
		stParam.enPixelType = stFrameInfo.enPixelType  # ch:相机对应的像素格式 | en:Camera pixel type
		stParam.nWidth = stFrameInfo.nWidth  # ch:相机对应的宽 | en:Width
		stParam.nHeight = stFrameInfo.nHeight  # ch:相机对应的高 | en:Height
		stParam.nDataLen = stFrameInfo.nFrameLen
		stParam.pData = cast(pData, POINTER(c_ubyte))
		stParam.pImageBuffer = cast(byref(m_pBufForSaveImage), POINTER(c_ubyte))
		stParam.nBufferSize = nRGBSize  # ch:存储节点的大小 | en:Buffer node size
		stParam.nJpgQuality = 80;  # ch:jpg编码，仅在保存Jpg图像时有效。保存BMP时SDK内忽略该参数
		return_code = cam.MV_CC_SaveImageEx2(stParam)
		if return_code != 0:
			print("save jpg fail ret[0x%x]" % return_code)
		file_open = open(file_path.encode('ascii'), 'wb+')
		img_buff = (c_ubyte * stParam.nImageLen)()
		memmove(byref(img_buff), stParam.pImageBuffer, stParam.nImageLen)
		file_open.write(img_buff)


		# python视觉程序部分
		image = cv2.imread("/home/lingyun/Documents/work/main.jpg")
		# 高斯模糊
		image = cv2.blur(image, (1, 15))
		# hsv
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_hsv = np.array([60, 100, 100])
		upper_hsv = np.array([120, 255, 255])
		mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
		dst = cv2.bitwise_and(image, image, mask=mask)
		# 灰度化
		img = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
		# 找轮廓
		dilation, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		select = []
		for i in range(len(contours)):
			cnt = contours[i]
			x, y, w, h = cv2.boundingRect(cnt)
			area = cv2.contourArea(cnt)
			# 通过面积剔除小干扰项
			if area > 0:
				aspect_ratio = float(w/h) # 宽长比
				# print('宽长比=',aspect_ratio)
				if 0.2 < aspect_ratio <= 0.5:
					if h-w>=9:
						select.append(i)
		print("长度=",len(select))
		if len(select) == 2:
			x1, y1, w1, h1 = cv2.boundingRect(contours[select[0]])
			x2, y2, w2, h2 = cv2.boundingRect(contours[select[1]])
			if x1 > x2:
				x1, y1, w1, h1 = cv2.boundingRect(contours[select[0]])
				x2, y2, w2, h2 = cv2.boundingRect(contours[select[1]])
				x = (x1 + w1 - x2) / 2 + x2
				y = (h1 / 2) + y2
				y4 = y1+h1
				x4 = x1+w1
				y3 = y2
				x3 = x2
				cropped = image[y3:y4, x3:x4]
				cv2.imwrite("/home/lingyun/Documents/work/cv_cut_thor.jpg", cropped)
				# im = cv2.imread("/home/lingyun/Documents/work/cv_cut_thor.jpg")
				try:
					#if im != None:
					print("x=", x)
					print("y=", y)
					usb_1(x,y)
					# else:
						# print("0-1")
						# usb_0
				except:
					print("error")
			else:
				x1, y1, w1, h1 = cv2.boundingRect(contours[select[1]])
				x2, y2, w2, h2 = cv2.boundingRect(contours[select[0]])
				x = (x1 + w1 - x2) / 2 + x2
				y = (h1 / 2) + y2
				y4 = y2 + h2
				x4 = x2 + w2
				y3 = y1
				x3 = x1
				cropped = image[y3:y4, x3:x4]
				cv2.imwrite("/home/lingyun/Documents/work/cv_cut_thor.jpg", cropped)
				# im = cv2.imread("/home/lingyun/Documents/work/cv_cut_thor.jpg")
				# try:
					
				print("x=", x)
				print("y=", y)
				usb_1(x,y)
				# except:
					# print("system error")
		if len(select)==0:
			for i in range(len(contours)):
				cnt = contours[i]
				x, y, w, h = cv2.boundingRect(cnt)
				area = cv2.contourArea(cnt)
				aspect_ratio = float(w/h) 
				if 0.2<aspect_ratio<0.5:
					select.append(i)
			if len(select)==2:
				x1,y1,w1,h1 = cv2.boundingRect(contours[select[0]])
				x2,y2,w2,h2 = cv2.boundingRect(contours[select[1]])
				if x1>x2:
					x1,y1,w1,h1 = cv2.boundingRect(contours[select[0]])
					x2,y2,w2,h2 = cv2.boundingRect(contours[select[1]])
					x = (x1+w1-x2)/2+x2
					print("x=",x)
					y = (h1/2)+y2
					print("y=",y)
					usb_1(x,y)
				else:
					x1,y1,w1,h1 = cv2.boundingRect(contours[select[1]])
					x2,y2,w2,h2 = cv2.boundingRect(contours[select[0]])
					x = (x1+w1-x2)/2+x2
					print("x=",x)
					y = (h1/2)+y2
					print("y=",y)
					usb_1(x,y)

			else:
				print("没有装甲板")
				usb_0()	
	end= time.time()
	print(end-start)
def image_callback_blue(pData, pFrameInfo, pUser):
	# 己方蓝色对方红色
	start = time.time()
	stFrameInfo = cast(pFrameInfo, POINTER(MV_FRAME_OUT_INFO_EX)).contents
	if stFrameInfo:
		print ("第 %s 帧" % (stFrameInfo.nFrameNum))
		m_pBufForSaveImage = None
		file_path = "/home/lingyun/Documents/work/main.jpg"
		nRGBSize = stFrameInfo.nWidth * stFrameInfo.nHeight * 3+ 2048
		if m_pBufForSaveImage is None:
			m_pBufForSaveImage = (c_ubyte * nRGBSize)()
		stParam = MV_SAVE_IMAGE_PARAM_EX()
		stParam.enImageType = MV_Image_Jpeg;stParam.nWidth = 200  # ch:相机对应的宽 | en:Width
		stParam.enPixelType = stFrameInfo.enPixelType  # ch:相机对应的像素格式 | en:Camera pixel type
		stParam.nWidth = stFrameInfo.nWidth  # ch:相机对应的宽 | en:Width
		stParam.nHeight = stFrameInfo.nHeight  # ch:相机对应的高 | en:Height
		stParam.nDataLen = stFrameInfo.nFrameLen
		stParam.pData = cast(pData, POINTER(c_ubyte))
		stParam.pImageBuffer = cast(byref(m_pBufForSaveImage), POINTER(c_ubyte))
		stParam.nBufferSize = nRGBSize  # ch:存储节点的大小 | en:Buffer node size
		stParam.nJpgQuality = 80;  # ch:jpg编码，仅在保存Jpg图像时有效。保存BMP时SDK内忽略该参数
		return_code = cam.MV_CC_SaveImageEx2(stParam)
		if return_code != 0:
			print("save jpg fail ret[0x%x]" % return_code)
		file_open = open(file_path.encode('ascii'), 'wb+')
		img_buff = (c_ubyte * stParam.nImageLen)()
		memmove(byref(img_buff), stParam.pImageBuffer, stParam.nImageLen)
		file_open.write(img_buff)


		# python视觉程序部分
		image = cv2.imread("/home/lingyun/Documents/work/main.jpg")
		# 高斯模糊
		image = cv2.blur(image, (1, 15))
		# hsv
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_hsv = np.array([120, 100, 100])
		upper_hsv = np.array([180, 255, 255])
		mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
		dst = cv2.bitwise_and(image, image, mask=mask)
		# 灰度化
		img = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
		# 找轮廓
		dilation, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		select = []
		for i in range(len(contours)):
			cnt = contours[i]
			x, y, w, h = cv2.boundingRect(cnt)
			area = cv2.contourArea(cnt)
			# 通过面积剔除小干扰项
			if area > 0:
				aspect_ratio = float(w/h) # 宽长比
				# print('宽长比=',aspect_ratio)
				if 0.2 < aspect_ratio <= 0.5:
					if h-w>=9:
						select.append(i)
		print("长度=",len(select))
		if len(select) == 2:
			x1, y1, w1, h1 = cv2.boundingRect(contours[select[0]])
			x2, y2, w2, h2 = cv2.boundingRect(contours[select[1]])
			if x1 > x2:
				x1, y1, w1, h1 = cv2.boundingRect(contours[select[0]])
				x2, y2, w2, h2 = cv2.boundingRect(contours[select[1]])
				x = (x1 + w1 - x2) / 2 + x2
				y = (h1 / 2) + y2
				y4 = y1+h1
				x4 = x1+w1
				y3 = y2
				x3 = x2
				cropped = image[y3:y4, x3:x4]
				cv2.imwrite("/home/lingyun/Documents/work/cv_cut_thor.jpg", cropped)
				# im = cv2.imread("/home/lingyun/Documents/work/cv_cut_thor.jpg")
				try:
					print("x=", x)
					print("y=", y)
					usb_1(x,y)
				except:
					print("system error")
			else:
				x1, y1, w1, h1 = cv2.boundingRect(contours[select[1]])
				x2, y2, w2, h2 = cv2.boundingRect(contours[select[0]])
				x = (x1 + w1 - x2) / 2 + x2
				y = (h1 / 2) + y2
				y4 = y2 + h2
				x4 = x2 + w2
				y3 = y1
				x3 = x1
				cropped = image[y3:y4, x3:x4]
				cv2.imwrite("/home/lingyun/Documents/work/cv_cut_thor.jpg", cropped)
				# im = cv2.imread("/home/lingyun/Documents/work/cv_cut_thor.jpg")
				try:
					print("x=", x)
					print("y=", y)
					usb_1(x,y)
				except:
					print("error")
		if len(select)==0:
			# print("超出五米")
			for i in range(len(contours)):
				cnt = contours[i]
				x, y, w, h = cv2.boundingRect(cnt)
				area = cv2.contourArea(cnt)

				aspect_ratio = float(w/h)  # ³¤¿í±È
				if 0.2<aspect_ratio<0.5:
					select.append(i)
			if len(select)==2:
				x1,y1,w1,h1 = cv2.boundingRect(contours[select[0]])

				x2,y2,w2,h2 = cv2.boundingRect(contours[select[1]])
				if x1>x2:
					x1,y1,w1,h1 = cv2.boundingRect(contours[select[0]])
					x2,y2,w2,h2 = cv2.boundingRect(contours[select[1]])
					x = (x1+w1-x2)/2+x2
					print("x=",x)
					y = (h1/2)+y2
					print("y=",y)
					usb_1(x,y)
				else:
					x1,y1,w1,h1 = cv2.boundingRect(contours[select[1]])
					x2,y2,w2,h2 = cv2.boundingRect(contours[select[0]])
					x = (x1+w1-x2)/2+x2
					print("x=",x)
					y = (h1/2)+y2
					print("y=",y)
					usb_1(x,y)
			else:
				print("没有装甲板")
				usb_0()	
	end= time.time()
	print(end-start)
# 己方红色对方蓝色
CALL_BACK_FUN_RED = FrameInfoCallBack(image_callback_red)
# 己方蓝色对方红色
CALL_BACK_FUN_BLUE = FrameInfoCallBack(image_callback_blue)


def press_any_key_exit():
	fd = sys.stdin.fileno()
	old_ttyinfo = termios.tcgetattr(fd)
	new_ttyinfo = old_ttyinfo[:]
	new_ttyinfo[3] &= ~termios.ICANON
	new_ttyinfo[3] &= ~termios.ECHO
	#sys.stdout.write(msg)
	#sys.stdout.flush()
	termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
	try:
		os.read(fd, 7)
	except:
		pass
	finally:
		termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)

if __name__ == "__main__":
	# try:
	serial = serial.Serial('/dev/ttyUSB0',115200,timeout=0.0001)
	while True:
		res1,res2 = usb_start()
		if res2 == '1':	
			deviceList = MV_CC_DEVICE_INFO_LIST()
			tlayerType =  MV_USB_DEVICE
			# ch:枚举设备 | en:Enum device
			ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
			if deviceList.nDeviceNum == 0:
				print ("find no device!")
				sys.exit()
			nConnectionNum = 0
			if int(nConnectionNum) >= deviceList.nDeviceNum:
				print ("intput error!")
				sys.exit()
			# ch:创建相机实例 | en:Creat Camera Object
			cam = MvCamera()
			# ch:选择设备并创建句柄 | en:Select device and create handle
			stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
			ret = cam.MV_CC_CreateHandle(stDeviceList)
			if ret != 0:
				print ("create handle fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:打开设备 | en:Open device
			ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
			if ret != 0:
				print ("open device fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:设置触发模式为off | en:Set trigger mode as off
			ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
			if ret != 0:
				print ("set trigger mode fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:注册抓图回调 | en:Register image callback
			ret = cam.MV_CC_RegisterImageCallBackEx(CALL_BACK_FUN_RED,None)
			if ret != 0:
				print ("register image callback fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:开始取流 | en:Start grab image
			ret = cam.MV_CC_StartGrabbing()
			if ret != 0:
				print ("start grabbing fail! ret[0x%x]" % ret)
				sys.exit()
			print ("press a key to stop grabbing.")
			press_any_key_exit()
			# ch:停止取流 | en:Stop grab image
			ret = cam.MV_CC_StopGrabbing()
			if ret != 0:
				print ("stop grabbing fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:关闭设备 | Close device
			ret = cam.MV_CC_CloseDevice()
			if ret != 0:
				print ("close deivce fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:销毁句柄 | Destroy handle
			ret = cam.MV_CC_DestroyHandle()
			if ret != 0:
				print ("destroy handle fail! ret[0x%x]" % ret)
				sys.exit()
			break
		elif res2 == '2':
			deviceList = MV_CC_DEVICE_INFO_LIST()
			tlayerType =  MV_USB_DEVICE
			# ch:枚举设备 | en:Enum device
			ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
			if deviceList.nDeviceNum == 0:
				print ("find no device!")
				sys.exit()
			nConnectionNum = 0
			if int(nConnectionNum) >= deviceList.nDeviceNum:
				print ("intput error!")
				sys.exit()
			# ch:创建相机实例 | en:Creat Camera Object
			cam = MvCamera()
			# ch:选择设备并创建句柄 | en:Select device and create handle
			stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
			ret = cam.MV_CC_CreateHandle(stDeviceList)
			if ret != 0:
				print ("create handle fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:打开设备 | en:Open device
			ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
			if ret != 0:
				print ("open device fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:设置触发模式为off | en:Set trigger mode as off
			ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
			if ret != 0:
				print ("set trigger mode fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:注册抓图回调 | en:Register image callback
			ret = cam.MV_CC_RegisterImageCallBackEx(CALL_BACK_FUN_BLUE,None)
			if ret != 0:
				print ("register image callback fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:开始取流 | en:Start grab image
			ret = cam.MV_CC_StartGrabbing()
			if ret != 0:
				print ("start grabbing fail! ret[0x%x]" % ret)
				sys.exit()
			print ("press a key to stop grabbing.")
			press_any_key_exit()
			# ch:停止取流 | en:Stop grab image
			ret = cam.MV_CC_StopGrabbing()
			if ret != 0:
				print ("stop grabbing fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:关闭设备 | Close device
			ret = cam.MV_CC_CloseDevice()
			if ret != 0:
				print ("close deivce fail! ret[0x%x]" % ret)
				sys.exit()
			# ch:销毁句柄 | Destroy handle
			ret = cam.MV_CC_DestroyHandle()
			if ret != 0:
				print ("destroy handle fail! ret[0x%x]" % ret)
				sys.exit()
			break
		else:
			print("没有接受到颜色的数据！")
		time.sleep(0.01)
	# except:
		# print("程序出问题了")
		
			
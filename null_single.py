import cv2
import glob
import os
import numpy as np
#from os import listdir
#from os.path import isfile,join
from os import listdir
from os.path import isfile,join
import math
import serial

#ser = serial.Serial('COM5',baudrate = 115200,timeout =0.1) #tao cong noi tiep
font = cv2.FONT_HERSHEY_DUPLEX
#đọc ảnh
upper_yellow = np.array([17,255,255])
frame = cv2.imread('Capture.PNG');
#hiển thị ảnh lên
cv2.imshow('frame', frame);
#bấm 1 phím bất kỳ để tắt của sổ
#loc anh Gauss
frame_gauss = cv2.GaussianBlur(frame,(5,5),0.3)
rgb_frame =cv2.cvtColor(frame_gauss,cv2.COLOR_BGR2RGB) 
cv2.imshow('rgb_frame',rgb_frame)
cv2.imwrite('frame_gauss.jpg', frame_gauss);
#cv2.imshow('frame_gauss', frame_gauss);
obj_hsv = np.zeros(frame.shape,dtype="uint8")
mask_green = np.zeros(frame.shape,dtype="uint8")
mask_yellow = np.zeros(frame.shape,dtype="uint8")
frame_copy = []
frame_copy = frame.copy()
hsv_frame =cv2.cvtColor(frame_gauss,cv2.COLOR_BGR2HSV) 
cv2.imwrite('hsv_frame.jpg', hsv_frame);

#loc bo nen trang
lower_background = np.array([5,50,0])
upper_background = np.array([80,255,255])
mask_background = cv2.inRange(hsv_frame,lower_background,upper_background)
kernel_background = np.ones((5,5),np.uint8)
#cv2.imshow('mask_background_inRange', mask_background)
mask_background = cv2.erode(mask_background,kernel_background)
cv2.imwrite('mask_background_erode.jpg', mask_background)
#tim duong bao
contours,_ = cv2.findContours(mask_background,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
contours = sorted(contours,key = cv2.contourArea)
#print(len(contours))
#cv2.waitKey(0)
cnt = contours[len(contours)-1]
dien_tich = cv2.contourArea(cnt)
approx = cv2.approxPolyDP(cnt,0.0001*cv2.arcLength(cnt,True),True)
print(approx)
#cv2.imshow('approx',approx)
cv2.drawContours(frame_copy,[approx],0,(255,0,0),5)
cv2.imwrite('contour_no_box.jpg', frame_copy)

#ve hinh chu nhat quanh contour d
rect = cv2.minAreaRect(cnt)
box = cv2.boxPoints(rect)
box = np.int0(box)
cv2.drawContours(frame_copy, [box], 0, (0,0,255), 2)
cv2.imwrite('conour_with_box.jpg', frame_copy)
#mask_background = cv2.bitwise_not(mask_background)     
            #tim tam và ve tam Contour
M = cv2.moments(cnt)
cX = int(M["m10"] / M["m00"])
cY = int(M["m01"] / M["m00"])                
cv2.circle(frame_copy, (cX, cY), 7, (255, 255, 255), -1)
#cv2.putText(frame_copy, "center", (cX - 20, cY - 20),cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 0), 2)
    #print (cX,cY)
    
    #chieu dai chieu rong qua chuoi
kich_thuoc_1 = rect[1][0] 
kich_thuoc_2 = rect[1][1]
if kich_thuoc_1 > kich_thuoc_2:
        chieu_dai_pixel = int(kich_thuoc_1)
        chieu_rong_pixel = int(kich_thuoc_2)
else:
        chieu_dai_pixel = int(kich_thuoc_2)
        chieu_rong_pixel = int(kich_thuoc_1)
chieu_dai_thuc = int(chieu_dai_pixel*10 /28)
chieu_rong_thuc = int(chieu_rong_pixel*10/28)
#print (chieu_dai_pixel)
    

    
    # trich anh cua object ra gray va mau 
mask = np.zeros(frame.shape,dtype="uint8")
cv2.imshow('mask_trich',mask)
cv2.drawContours(mask, [cnt], -1, (255,255,255), -1) 
cv2.imshow('mask_trich',mask)
cv2.imwrite('only_mask.jpg',mask)         
mask_gray = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)            
_, mask_threshhold = cv2.threshold(mask_gray, 10, 255, cv2.THRESH_BINARY)  
cv2.imwrite('mask_with_contour.jpg',mask_threshhold)         
obj_rbg = cv2.bitwise_and(frame,frame,mask = mask_threshhold)  #trich anh vat the _anh mau  
cv2.imwrite('obj_rbg.jpg',obj_rbg)         
obj_hsv = cv2.cvtColor(obj_rbg,cv2.COLOR_BGR2HSV)
#cv2.imshow('obj_rbg',obj_rbg)
#cv2.imshow('obj_hsv',obj_hsv)

    #loc_mau_xanh
lower_green = np.array([26,80,0])
upper_green = np.array([80,250,255])
mask_green = cv2.inRange(obj_hsv,lower_green,upper_green)
cv2.imwrite('mask_green.jpg',mask_green)
#kernel_green = np.ones((5,5),np.uint8)
#mask_green = cv2.erode(mask_green,kernel_green)
so_pixel_xanh = cv2.countNonZero(mask_green)
print(so_pixel_xanh)
ti_le_xanh = int((so_pixel_xanh / dien_tich) *100)

    #loc mau vang
lower_yellow = np.array([10,100,0])
upper_yellow = np.array([20,255,200])
mask_yellow = cv2.inRange(obj_hsv,lower_yellow,upper_yellow)
cv2.imshow('mask_yellow_1',mask_yellow)
kernel_yellow = np.ones((5,5),np.uint8)
mask_yellow = cv2.erode(mask_yellow,kernel_yellow)
so_pixel_vang = cv2.countNonZero(mask_yellow)
print(so_pixel_vang)
ti_le_vang = int((so_pixel_vang / dien_tich) *100)
cv2.imwrite('mask_yellow.jpg',mask_yellow)
print(dien_tich - so_pixel_xanh - so_pixel_vang)


    #tinh so pixel den
so_pixel_den = dien_tich - so_pixel_xanh - so_pixel_vang
ti_le_den = 100 - ti_le_xanh - ti_le_vang
print('ti_le_den = '+str(ti_le_den))
    #phan loai mau xanh vang
if so_pixel_xanh > so_pixel_vang:
        mau_sac = 'xanh'
else:
        mau_sac = 'vang'


    #phan loai kich thuoc
if chieu_dai_thuc > 120:
        kich_thuoc = 'dai'
else:
        kich_thuoc = 'ngan'

    #phan loai chung 
if (mau_sac == 'xanh') and (kich_thuoc == 'dai'):    phan_loai = 1;
if (mau_sac == 'xanh') and (kich_thuoc == 'ngan'):    phan_loai = 2;
if (mau_sac == 'vang') and (kich_thuoc == 'dai'):    phan_loai = 3;b
if (mau_sac == 'vang') and (kich_thuoc == 'ngan'):    phan_loai = 4;
        
    

    #hien thi thong so 
    #print ('dien tich =' + str(dien_tich) )
    #print ('so_pixel_xanh =' + str(so_pixel_xanh) )
    #print ('so_pixel_vang =' + str(so_pixel_vang) )
    #print ('so_pixel_den=' + str(so_pixel_den) )
    #print ('ti_le_den=' + str(ti_le_den) + '%')
cv2.putText(frame_copy,'ti_le_xanh ='+ str(ti_le_xanh),(0,30),font,0.75,(0,0,0))
cv2.putText(frame_copy,'ti_le_vang ='+ str(ti_le_vang),(0,60),font,0.75,(0,0,0))
cv2.putText(frame_copy,'ti_le_den ='+ str(ti_le_den)+'%',(0,120),font,0.75,(0,0,0))
cv2.putText(frame_copy,'chieu dai ='+ str(chieu_dai_pixel) + 'pixel'+ '=' + str(chieu_dai_thuc),(0,150),font,0.75,(0,0,0))
cv2.putText(frame_copy,'chieu rong='+ str(chieu_rong_pixel) + 'pixel'+ '=' + str(chieu_rong_thuc),(0,180),font,0.75,(0,0,0))
cv2.putText(frame_copy,'Phan loai: ' + mau_sac +'  ' + kich_thuoc ,(0,210),font,0.75,(0,0,0))                
cv2.putText(frame_copy,'Toa do tam ' + str(cX) +'  ' + str(cY) ,(0,240),font,0.75,(0,0,0))




#chuyen thong tin cho vi xu li 
thong_tin = str(chieu_rong_thuc)+'.'+str(phan_loai)+'.'+'1'
#ser.write(bytes(thong_tin,encoding='utf8'))
print(thong_tin)
#chuyen thong tin len giao dien
thong_tin_giao_dien = str(chieu_dai_thuc) +'.' +  str(chieu_rong_thuc)+ '.' +str(ti_le_xanh)+ '.' + str(ti_le_vang)+ '.' + str(ti_le_den)+ '.'+str(phan_loai);
#ser_1.write(bytes(thong_tin_giao_dien,encoding='utf8'))

cv2.imshow('frame_hien_thi',frame_copy)

cv2.waitKey(0)
#giải phòng bộ nhớ
cv2.destroyAllWindows();
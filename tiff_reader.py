import cv2
import numpy as np
import tiffcapture as tc
import matplotlib.pyplot as plt


def normalize_thermals(img):
    cv2.normalize(img, img, 0, 2**16-1, cv2.NORM_MINMAX)
    return img


tiff = tc.opentiff("lufthavn24feb/20200224_132038/20200224_132425_IR_A.tiff")
vidObj = cv2.VideoCapture("lufthavn24feb/20200224_132038/20200224_132425_VIS_A.mov")

n_RGB_frames = int(vidObj.get(cv2.CAP_PROP_FRAME_COUNT))
n_thermal_frames = tiff.length

ratio = n_RGB_frames / n_thermal_frames

_, first_img = tiff.retrieve() 

scale_percent = 400 # percent of original size
width = int(first_img.shape[1] * scale_percent / 100)
height = int(first_img.shape[0] * scale_percent / 100)
dim = (width, height)

cv2.namedWindow('thermal_video')
#cv2.namedWindow('RGB_video')
count = 0

y1 = 30
y2 = 90
x1 = 40
x2 = 120

playing = True

ret, img = tiff.read()  
while(ret):
    #if count > ratio: # There are more RGB frames than thermal. To play both videos at the same time, this must be done. 
    if playing:
        ret, img = tiff.read()  
            #count = 0
        
        #ret, RGB_img = vidObj.read()
        #plt.imshow(tiff.read())
        #plt.show()

        img_Celcius = img * 0.01 - 273.15

        roi = img[y1:y2, x1:x2]
        img = roi

        img = normalize_thermals(img)

        resized_thermal = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        #resized_RGB = cv2.resize(RGB_img, dim, interpolation = cv2.INTER_AREA)

        #cv2.imshow('RGB_video', resized_RGB)
        cv2.imshow('thermal_video', resized_thermal)
        cv2.imshow('thermal_video', img_Celcius)
        cv2.waitKey(10)
        

    if cv2.waitKey(4) & 0xFF == ord('p'):#Pause
        playing = False
    if cv2.waitKey(4) & 0xFF == ord('c'):#Continue
        playing = True

    count += 1


cv2.destroyWindow('video')
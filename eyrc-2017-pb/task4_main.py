'''
# Team Id :PB#5624
# Author List :ALOK SENAPATI, ASHIS KUMAR MAHARANA
# Filename:task4_main.py
# Theme:Planter Bot
# Functions:sharp_left,sharp_right,move_forward,stop,detect_line_color,detect_cm_color,detect_cm_shape,
            blinking,end_blinking,blink_append,centroid,flower_select,zone_select,blend_transparent
# Global Variables:None
# variables:error,last_error,turn_speed,kp,kd,initial_speed,black_contour_count,stop_count,zi_count,cm_count,frame_mid,end_line
'''


#### Importing necessary library modules ####
import cv2
import numpy as np
import RPi.GPIO as gpio
import picamera
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
import csv
#############################################
#### GPIO Setup and Pin Declaration #########
gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)
left_motor_forward=35
left_motor_backward=37
right_motor_forward=31
right_motor_backward=33
left_motor_enable=38
right_motor_enable=32
blue_led=16
green_led=18
red_led=22
gpio.setup(red_led,gpio.OUT)
gpio.setup(blue_led,gpio.OUT)
gpio.setup(green_led,gpio.OUT)
gpio.setup(right_motor_enable,gpio.OUT)
gpio.setup(left_motor_enable,gpio.OUT)
gpio.setup(left_motor_forward,gpio.OUT)
gpio.setup(left_motor_backward,gpio.OUT)
gpio.setup(right_motor_forward,gpio.OUT)
gpio.setup(right_motor_backward,gpio.OUT)
right_enable=gpio.PWM(right_motor_enable,90)
left_enable=gpio.PWM(left_motor_enable,90)
right_enable.start(0)
left_enable.start(0)
###########################################

#### Change log ###########################
####### PD Control ##########
kp=0.0915
kd=0.46
initial_speed=14
#############################


#### Variable initialization and assignment ####
error=0
last_error=50
turn_speed=13
blink_sequence=[]
frame_count=0
frame_mid=220
stop_count=0
black_contour_count=0
zi_count=0
################################################
'''
* Function Name: sharp_left
* Input: None
* Output: None
* Logic: turning left upto 90 degree using PWM with duty cycle turn_speed
* Example Call: sharp_left()
'''
def sharp_left():
    right_enable.ChangeDutyCycle(turn_speed)
    left_enable.ChangeDutyCycle(turn_speed)
    gpio.output(left_motor_forward,False)
    gpio.output(left_motor_backward,True)
    gpio.output(right_motor_forward,True)
    gpio.output(right_motor_backward,False)

'''
* Function Name: sharp_right
* Input: None
* Output: None
* Logic: turning right upto 90 degree using PWM with duty cycle turn_speed
* Example Call: sharp_right()
'''
def sharp_right():
    right_enable.ChangeDutyCycle(x)
    left_enable.ChangeDutyCycle(x)
    gpio.output(left_motor_forward,True)
    gpio.output(left_motor_backward,False)
    gpio.output(right_motor_forward,False)
    gpio.output(right_motor_backward,True)

'''
* Function Name: move_forward
* Input: None
* Output: None
* Logic: move forward using PWM with duty cycle turn_speed
* Example Call: move_forward()
'''
def move_forward():
    right_enable.ChangeDutyCycle(13)
    left_enable.ChangeDutyCycle(13)
    gpio.output(left_motor_forward,True)
    gpio.output(left_motor_backward,False)
    gpio.output(right_motor_forward,True)
    gpio.output(right_motor_backward,False)

'''
* Function Name: stop_moving
* Input: None
* Output: None
* Logic: makes duty cycle 0 and stops the motor
* Example Call: stop_moving()
'''
def stop():
    right_enable.ChangeDutyCycle(0)
    left_enable.ChangeDutyCycle(0)
    gpio.output(left_motor_forward,False)
    gpio.output(left_motor_backwward,False)
    gpio.output(right_motor_forward,False)
    gpio.output(right_motor_backward,False)

'''
* Function Name: detect_line_color
* Input: cx,cy->Centroid co-ordinates of the line
* Output: clr->Color of the line
* Logic: returns color of the line using the b,g,r value of the centroid
* Example Call: detect_line_color(a,b)
'''
def detect_line_color(cx,cy):
    b,g,r=img[cy,cx]
    if b<60 and g<60 and r<60:
        clr='black'
    else:
        clr='None'
    return clr

'''
* Function Name: detect_cm_color
* Input: cx,cy->Centroid co-ordinates of the color marker
* Output: clr->Color of the color_marker
* Logic: returns color of the color marker using the b,g,r value of their centroid
* Example Call: detect_cm_color(a,b)
'''
def detect_cm_color(cx,cy):
    b,g,r=cropped[cy,cx]
    if r+5>b and r+5>g:
        clr= 'Red'
    elif b+15>g and b+10>r:
        clr= 'Blue'    
    elif g-5>b and g-5>r:
        clr= 'Green'
    return clr

'''
* Function Name: blinking
* Input: color->color of the color marker,
         t->led ON time period in seconds
* Output: None
* Logic: Blinks the RGB led based on the color and the GPIO pin allocated to that color
* Example Call: color('Blue',0.5)
'''
def blinking(color,t):
    if color=='Blue':
        gpio.output(blue_led,True)
        gpio.output(green_led,False)
        gpio.output(red_led,False)
        time.sleep(t)
        gpio.output(blue_led,False)
    elif color=='Green':
        gpio.output(blue_led,False)
        gpio.output(green_led,True)
        gpio.output(red_led,False)
        time.sleep(t)
        gpio.output(green_led,False)
    elif color=='Red':
        gpio.output(blue_led,False)
        gpio.output(green_led,False)
        gpio.output(red_led,True)
        time.sleep(t)
        gpio.output(red_led,False)
    time.sleep(t)    

'''
* Function Name: blink_append
* Input: color->color of the color marker,
* Output: None
* Logic: An blank array is declared above as blink_sequence,
         When a color marker is detected succesfully the array is
         appended with the pin number assigned to the respective led
* Example Call: blink_append('Red')
'''
def blink_append(color):
    if color=='Red':
        blink_sequence.append(red_led)
    elif color=='Blue':
        blink_sequence.append(blue_led)
    elif color=='Green':
        blink_sequence.append(green_led)

'''
* Function Name: end_blinking
* Input: blink_sequence -> The array declared to store
         the pin numbers of the colors detected at each zi sequentially
* Output: None
* Logic: Depending on the occurance of sequence of colors at the zone indicator,
         it blinks the leds at the end of run
* Example Call: end_blinking(blink_sequence)
'''
def end_blinking(blink_sequence):
    for blk in blink_sequence:
        gpio.output(blk,True)
        time.sleep(0.2)
        gpio.output(blk,False)
        time.sleep(1)
        
'''
* Function Name: detect_cm_shape
* Input: cnt->Contour detected at the zone indicators
* Output: shape->Shape of the contour
* Logic: cv2.approxPolyDP returns the minimum number of vertices of vertices the contour can be reconstructed,
         Based on this number the geometrical shapes are detected
* Example Call: detect_cm_shape(x)
'''    
def detect_cm_shape(x):
    approx = cv2.approxPolyDP(x,0.035*cv2.arcLength(x,True),True)
    if len(approx)==4:
        shape='Square'
    elif 12>len(approx)>6:
        shape='Circle'
    elif len(approx)==3:
        shape='Triangle'
    else:
        shape='None'
    return shape

'''
* Function Name: centroid
* Input: cnt->Contour detected at the zone indicators
* Output: cx,cy-> x and y co-ordinates of the centoid of the contour
* Logic: moment about the centroid is zero
* Example Call: centroid(x)
'''
def centroid(contour):
    M=cv2.moments(contour)
    cx=int(M['m10']/M['m00'])
    cy=int(M['m01']/M['m00'])
    return cx,cy

'''
* Function Name: flower_select
* Input: cm_shape,cm_color->Color marker shape, Color marker Color
* Output: flower->Gives the name of the flower to be overlayed, as string
* Logic: Compares the cm_color and cm_shape with the 1st and 2nd element of the rows given in the Input Table.csv file
         and return the third element of the row when matched
* Example Call: flower_select('Triangle','Green')
'''
def flower_select(cm_shape,cm_color):
    with open('Input Table.csv','rb') as csvfile:
        reader=csv.reader(csvfile,delimiter=',',quotechar='|')
        if cm_shape!='None':
            for row in reader:
                if row[0]==cm_color and row[1]==cm_shape:
                    flower=row[2]
    return flower

'''
* Function Name: zone_select
* Input: zi_count,cm_count ->Zone Indicator number, Number of color markers detected
* Output: resize_x,resize_y ->resizing parameters the flower choosen,
          yi,xi ->starting point for overlay
          distance_factor ->distance between the two consecutive flowers
* Logic: The ROI of plantation zones are detected manually and are fixed. zi_count selects the plantaion zone.
         Depending on the plantation zone and cm_count the resizing parameters for the flower selected are made.
         Distance_factor is based on the size of plantation zone.         
* Example Call: zone_select(3,4)
'''
def zone_select(zi_count,cm_count):
    if zi_count==1:
        if cm_count<=2:
            resize_x=int(abs((475-593))/2)
            resize_y=int(abs((230-298)))
            yi=abs(210)
            xi=abs(400)
        else:
            resize_x=int(abs((370-593))/cm_count)
            resize_y=int(abs((230-298)))
            yi=abs(210)
            xi=abs(320)
        distance_factor=7
    elif zi_count==2:
        if cm_count<=2:
            resize_x=int(int(abs((120-189))/2))
            resize_y=int(int(abs((221-257))))
            yi=abs(221)
            xi=abs(95)
        else:
            resize_x=int(int(abs((70-189))/cm_count))
            resize_y=int(int(abs((221-257))))
            yi=abs(221)
            xi=abs(65)
        distance_factor=3
    elif zi_count==3:
        if cm_count<=2:
            resize_x=int(int(abs((300-380))/2))
            resize_y=int(int(abs((165-200))))
            yi=abs(160)
            xi=abs(275)
        else:
            resze_x=int(int(abs((252-380))/cm_count))
            resize_y=int(int(abs((165-200))))
            yi=abs(160)
            xi=abs(252)
        distance_factor=3
    elif zi_count==4:
        if cm_count<=2:
            resize_x=int(int(abs((540-625))/2))
            resize_y=int(int(abs((180-220))))
            yi=abs(170)
            xi=abs(520)
        else:
            resize_x=int(int(abs((510-625))/cm_count))
            resize_y=int(int(abs((180-215))))
            yi=abs(175)
            xi=abs(505)
        distance_factor=3
    return resize_x,resize_y,xi,yi,distance_factor

'''
* Function Name: blend_transparent
* Input: face_img, overlay_t_img -> the ROI of the two images to be overlayed
* Output: overlay ->result image after overlay process same as the size of the inputs
* Logic: The input images must be of same size for this function. It separates the alpha channel and bgr of the image to be overlayed.
         Then it calculates the mask,weight for the two images and overlays them using addWeighted.
* Example Call: blend_transparent(face[:,:],glasses[:,:])
'''
def blend_transparent(face_img, overlay_t_img):
    # Split out the transparency mask from the colour info
    overlay_img = overlay_t_img[:,:,:3] # Grab the BRG planes
    overlay_mask = overlay_t_img[:,:,3:]  # And the alpha plane

    # Again calculate the inverse mask
    background_mask = 255 - overlay_mask

    # Turn the masks into three channel, so we can use them as weights
    overlay_mask = cv2.cvtColor(overlay_mask, cv2.COLOR_GRAY2BGR)
    background_mask = cv2.cvtColor(background_mask, cv2.COLOR_GRAY2BGR)

    # Create a masked out face image, and masked out overlay
    # We convert the images to floating point in range 0.0 - 1.0
    face_part = (face_img * (1 / 255.0)) * (background_mask * (1 / 255.0))
    overlay_part = (overlay_img * (1 / 255.0)) * (overlay_mask * (1 / 255.0))

    # And finally just add them together, and rescale it back to an 8bit integer image    
    overlay= np.uint8(cv2.addWeighted(face_part, 255.0, overlay_part, 255.0, 0.0))
    return overlay
###############################################################################################
#################################################################################################
#### Main Programe #######################
#initialize a PiCam object
cam=PiCamera()
#Reads the plantation image
plantation=cv2.imread('Plantation.png')
#Sets the frame rate
cam.framerate=60
#Starts the timer and stores it in start
start=time.time()
#create a RGB Array of PiCam storage type
raw_cap=PiRGBArray(cam,(640,480))
#to capture video coninuously create a video object
for frame in cam.capture_continuous(raw_cap,format="bgr",use_video_port=True,splitter_port=2,resize=(640,480)):
    shapecolor=['None','None']  #Initializing a array which stores the color marker shape and color
    end_line=0  
    cm_count=0
    black_contour_count=0
    black_array=[0,0]
    #Reads the frame
    image=frame.array
    #Taking the ROI of the required portion 
    img=image[200:450,100:540]
    # The value of threshold,blur kernel size, suits our lighting condition. it may be changed later depending on the lighting condition
    blur=cv2.blur(img,(11,11))
    gray=cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
    r,thresh=cv2.threshold(gray,40,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) #Thresholding using BINARY_INV and OTSU filterring
    _,cont,hier=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    # This block counts the black_contours of more than 20000pixel area #########
    for cnt in cont:
        area=cv2.contourArea(cnt)
        s,y,w,h=cv2.boundingRect(cnt)
        cx=int(s+w/2)
        cy=int(y+h/2)
        if line_color(cx,cy)=='black':
            if area>20000:
                black_array[black_contour_count]=cx #black_array stores the x co-ordinate of the centre of the bounding rect
                black_contour_count+=1
    #############################################################################
                
    #if more than 1 black contours are found(as in case of Inverted Plain)############
    if black_contour_count>1:
        r,thresh=cv2.threshold(gray,40,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) #here BINARY thresholding is taken
        _,cont,hier=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    ##################################################################################

    ###### We have taken a array of lines at the top of the frame and taking the sum of their pixel values ######
    for e in range(100,340):
        end_line+=thresh[30,e]
    #############################################################################################################
    for cnt in cont:
        area=cv2.contourArea(cnt)
        s,t,w,h=cv2.boundingRect(cnt)
        cx=int(s+w/2)
        cy=int(t+h/2)
        if black_contour_count<2:
            cx=cx
        else:
            cx=int((black_array[0]+black_array[1])*0.5) ## If BINARY thresholding not working then take the mid point of two black contours
        if line_color(cx,cy)=='black':
            cv2.circle(img,(cx,cy),2,(255,0,255),-1)
        if area>70000:
            if end_line<8000: ## It means the line(end_line) has more white pixels. Normally it should be 0 but for worst case scenario we have taken 8000
                stop()
                stop_count+=1
            else:
                move_forward()
                time.sleep(0.01)
            break
        ## This block finds the Zone indicator by comparing the area and end_line value
        elif 58000>area>40000 and black_contour_count<2: 
            if 60000>end_line>50000:
                start=time.time()
                ## After the process of overlaying at ZI if within next 2seconds again this condition is satisfied, then it is ignored 
                if abs(start-end)>2:  
                    zi_count+=1
                    stop()
                    ## Two pictures taken and processed for color markers 
                    for counter in range(0,2):
                        cam.capture('color.jpg') 
                    color_marker_capture=cv2.imread('color.jpg')
                    cropped=color_marker_capture[0:650,150:1100]
                    color_marker_gray=cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY)
                    ### The blur kernel,kernel1,kernel2,thresholding value suit best for our lighting condition. The values may be changed later on depending on the lighting condition#### 
                    blured=cv2.blur(color_marker_gray,(1,1))
                    ret,color_marker_thresh=cv2.threshold(blured,120,255,cv2.THRESH_BINARY_INV)
                    kernel1=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
                    kernel2=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
                    erosion=cv2.erode(color_marker_thresh,kernel1,1)
                    dilation=cv2.dilate(erosion,kernel2,1)
                    ___,cm_cont,cm_hier=cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                    for c in cm_cont:
                        area=cv2.contourArea(c)
                        ### After processing the color markers are separated out based on the area. This may also change later on depending on the lighting condition ####
                        ### This range of area elliminates the noise and also the path from consideration ###
                        if 8000>area>1800:
                            cm_count+=1
                            Cx,Cy=centroid(c)
                            Shape=detect_cm_shape(c)
                            Color=detyect_cm_color(Cx,Cy)
                            blinking(Color,0.5)
                            ##If the shapecolor[0] that is the shape of previous contour is None then it stores the shape and color in shapecolor[] of the present color marker ###
                            if shapecolor[0]=='None':   
                                shapecolor[0]=str(Shape)
                                shapecolor[1]=str(Color)
                            else:
                                break
                        
                    blink_append(shapecolor[1])
                    if cm_count>0 and shapecolor[1]!='None' and shapecolor[0]!='None' and zi_count<=4:
                        stringflower=flower_select(Shape,Color)
                        ### Reads the flower whose name is given by stringflower ###
                        flower=cv2.imread(stringflower,-1)
                        start_x,start_y,xi,yi,dist_fact=zone_select(zi_count,cm_count)
                        if flower=='None':
                            break
                        else:
                            rflower=cv2.resize(flower,(start_x,start_y))  ##Resizes the flower image ##
                            for cin in range(1,cm_count+1):  ## Overlays the flower image cm_count times ##
                                ### cin iterates the strating point of the flower to be overlayed ####
                                plantation[yi:yi+start_y,(xi+cin*dist_fact)+(cin-1)*start_x:(xi+cin*dist_fact)+cin*start_x,:] = blend_transparent(plantation[yi:yi+start_y,(xi+cin*dist_fact)+(cin-1)*start_x:(xi+cin*dist_fact)+cin*start_x,:],rflower)
                            cv2.imshow('frame',plantation)
                            cv2.waitKey(1)
                        
                    move_forward()
                    time.sleep(0.5)
                    end=time.time()
                    break
        ######### this block is for path following which impliments PD control ############        
        if 40000>area>4000:
            error=frame_mid-cx   ### Error is found by comparing frame_mid point and the x-cordinate of contour ####
            if error==0:
                last_error=0
                 
            power=error*kp+kd*(error-last_error)
            ### the duty cycle adjustment for not making it negative and greater than 100 ###
            if power>initil_speed:  
                power=initial_speed
            if power<-initial_speed:
                power=-initial_speed
            ### error value when out of 80(upper) and -80(lower)range,normal adjustment won't be enough. So sharp turns are taken.###  
            if error<-80:
                sharp_right()
            elif error>80:
                sharp_left()
            else:
                right_enable.ChangeDutyCycle(initial_speed+power)
                left_enable.ChangeDutyCycle(initial_speed-power)
                gpio.output(left_motor_forward,True)
                gpio.output(left_motor_backward,False)
                gpio.output(right_motor_forward,True)
                gpio.output(right_motor_backward,False)
        last_error=error
            
    frame_count+=1
    raw_cap.truncate(0)
    if frame_count>5:
        cv2.imshow('frame',plantation)
        if stop_count<1:
            cv2.waitKey(5)
    if stop_count>=1:
        right_enable.start(0)
        left_enable.start(0)
        time.sleep(1)
        end_blinking(blink_sequence)
        gpio.cleanup()
        break

cv2.destroyAllWindows()

    

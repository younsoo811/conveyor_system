import numpy as np
import cv2
import RPi.GPIO as IO
import time
from multiprocessing import Process, Manager
from adafruit_servokit import ServoKit

#py to sql
import socket
import mysql.connector
from datetime import datetime

#socket
server_addr=('192.168.0.12',12345)
client_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
client_socket.connect(server_addr)

m=Manager()
checkD=m.Value('c', 0)

checkCam=m.Value('c', 0)

#send to data
def send_to_db(msg):
    client_socket.send(str(msg).encode('utf-8'))

# 초음파센서 동작 함수
def distance_check():
    try:
        #hc-sr04
        IO.setmode(IO.BCM)
        IO.setwarnings(False)

        TRIG = 23 #23
        ECHO = 24 #24
        TRIG2 = 5 #23, 5
        ECHO2 = 6 #24, 6
        print("start")

        IO.setup(TRIG, IO.OUT)
        IO.setup(ECHO, IO.IN)
        IO.setup(TRIG2, IO.OUT)
        IO.setup(ECHO2, IO.IN)

        IO.output(TRIG, False)
        IO.output(TRIG2, False)
        time.sleep(2)
        #---------------------------------------
        
        start, stop, check_error=0, 0, 0

        while True:
            IO.output(TRIG,False)
            IO.output(TRIG,True)
            time.sleep(0.00001)    
            IO.output(TRIG, False)
        
            while IO.input(ECHO)==0:
                start = time.time()    
            
            while IO.input(ECHO)==1:
                stop = time.time()     
            
            check_time = stop - start
            dis = round(check_time * 34300 / 2, 2)

            if dis<15.5:
                if checkCam.value=='t1': # 빨간색 물류라면
                    print("check data__1")
                    time.sleep(2) # 2초 후
                    checkD.value='t' # 로봇팔 닫을수 있게 명령 내림
                else: # 빨간색 물류가 아니라면
                    print("pass Data")
                    checkD.value='t3' # 2번째 초음파 센서로 측정 시작
            elif dis>100:
                print("error distance!!!!")
            else:
                time.sleep(0.4)

            # 카메라에 물류가 감지되었는데 4초 이상 초음파에 탐지가 안될때
            if checkCam=='t' and checkD.value=='f':
                check_error = check_error + 1
                if check_error>=5:
                    dc_motor(0)
                    check_error=1
            # 에러가 발견된 상태에서 초음파에 물류가 탐지 되었을 때
            if check_error == 1 and checkD.value=='t':
                dc_motor(40) # 다시 모터를 동작시킴
                check_error=0               

            while checkD.value=='t3':
                IO.output(TRIG2, False)
                IO.output(TRIG2,True)
                time.sleep(0.00001)    
                IO.output(TRIG2, False)
        
                while IO.input(ECHO2)==0:
                    start = time.time()    
            
                while IO.input(ECHO2)==1:
                    stop = time.time()     
            
                check_time = stop - start
                dis = round(check_time * 34300 / 2, 2)
                
                if checkCam.value=='f' and checkD.value=='t3':
                    check_error = check_error + 1
                    if check_error >= 10:
                        checkD.value='f'
                        print('=====Reset checkD=====')
                        check_error=0
                        break

                if dis<15.5:
                    print("check data__2")
                    time.sleep(2)
                    checkD.value='t2'
                    check_error=0
                    break
                elif dis>100:
                    print("error distance!!!!")
                else:
                    time.sleep(0.3)
            
    except Exception as e:
        print('Exception!!')
        restart_process()
    
def restart_process():
    thread2 = Process(target=cam, args=())
    thread = Process(target=distance_check, args=())
    thread2.start()
    thread.start()  
    thread2.join()
    thread.join()

# 카메라와 서보모터 동작 함수
def cam():
    checkD.value='f'
    checkCam.value='f'
    
    roi_x, roi_y, roi_w, roi_h = 85, 40, 150, 150
    
    checkR, checkB = 0, 0
    
    Wdata,Tdata=0,0
    
    colorList=[]

    kit = ServoKit(channels=16)
    
    for i in range(4):
        kit.servo[i].angle=90

    try:
        print('camera on')
        cap = cv2.VideoCapture(0)
        cap.set(3,320)
        cap.set(4,240)
    except:
        print('camera failed')

    dc_motor(40) #motor on

    while True:
        ret, frame = cap.read()
        
        # 프레임에서 ROI(region of interest: 관심영역) 추출
        roi = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        
        # BGR -> HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # hsv 범위 지정
        lower_blue = np.array([90, 50, 100])
        upper_blue = np.array([130, 255, 255])      
        lower_red1 = np.array([0, 50, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 50, 100])
        upper_red2 = np.array([180, 255, 255])     
      
        # HSV img -> blue, red mask (색상 영역 추출)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red=mask_red1+mask_red2
        
        # mask 영역에서 서로 공통으로 겹치는 부분 출력
        res1 = cv2.bitwise_and(roi, roi, mask=mask_blue)
        res3 = cv2.bitwise_and(roi, roi, mask=mask_red)

        # 물류의 크기, 모양 측정
        for mask, color in [(mask_red, (0,0,255)), (mask_blue, (255,0,0))]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                epsilon=0.04*cv2.arcLength(contour, True)
                approx=cv2.approxPolyDP(contour,epsilon,True)
                x,y,w,h=cv2.boundingRect(approx)           
                if len(approx)==4 and w>=20:
                    W=w/12
                    cv2.drawContours(roi,[approx],0,color,2)
                    cv2.putText(frame, f"Width: %.1f cm"%W, (85,30),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
                    Wdata=w
                elif len(approx)==3 and w>=20:
                    W=w/12
                    cv2.drawContours(roi,[approx],0,color,2)
                    cv2.putText(frame, f"Width: %.1f cm"%W, (85,30),cv2.FONT_HERSHEY_SIMPLEX,0.5,color,2)
                    Tdata=w
        #print(np.sum(mask_blue > 0))
        if checkR<10 and np.sum(mask_red > 0) > 1300: # 물류 색상 확인
            checkR+=1 # 인식 오류 방지를 위한 확인용 데이터
            if checkR>9: # 정상적인 인식이라면
                if Wdata>=20 and Wdata<70: # 물류 크기 분석
                    print('add R1')                    
                    colorList.append('R1')
                    print(colorList)
                    checkCam.value='t1'
                    msg="color=RED, size=SMALL(>=20,<70)"
                    send_to_db(msg)
                elif Wdata>=70 and Wdata<130:
                    print('add R2')        
                    colorList.append('R2')
                    print(colorList)
                    checkCam.value='t1'
                    msg="color=RED, size=SMALL(>=70,<130)"
                    send_to_db(msg)
        elif checkB<10 and np.sum(mask_blue > 0) > 1300:
            checkB+=1
            if checkB>9:
                if Wdata>=20 and Wdata<70:
                    print('add B1')                   
                    colorList.append('B1')
                    print(colorList)
                    checkCam.value='t2'
                    msg="color=BLUE, size=SMALL(>=20,<70)"
                    send_to_db(msg)
                elif Wdata>=70 and Wdata<130:
                    print('add B2')
                    print(Tdata)
                    colorList.append('B2')
                    print(colorList)
                    checkCam.value='t2'
                    msg="color=BLUE, size=SMALL(>=70,<130)"
                    send_to_db(msg)
        
                    
        if len(colorList)>0 and checkD.value=='f':

            if colorList[0]=='R1':
                kit.servo[0].angle = 170
                #checkR=0
            elif colorList[0]=='R2':
                kit.servo[1].angle = 10
                #checkR=0
            elif colorList[0]=='B1':
                kit.servo[2].angle = 170
                #checkB=0
            elif colorList[0]=='B2':
                kit.servo[3].angle = 10
                #checkB=0

        
        if checkB>9 and np.sum(mask_blue > 0) < 1000:
            checkB=0
            print('============clear B')
        elif checkR>9 and np.sum(mask_red > 0) < 1000:
            checkR=0
            print('============clear R')
        
        
        if len(colorList)>0 and checkD.value=='t':
            #check HC-SR04 NO.1
            if colorList[0]=='R1':
                colorList.pop(0)
                print(colorList)                 
                kit.servo[0].angle = 90
                print('close R1')
                checkD.value='f'
            
            elif colorList[0]=='R2':
                colorList.pop(0)
                print(colorList)
                kit.servo[1].angle = 90
                checkD.value='f'               
        elif len(colorList)>0 and checkD.value=='t2':
            #check HC-SR04 NO.2
            if colorList[0]=='B1':
                colorList.pop(0)
                print(colorList)
                kit.servo[2].angle = 90
                checkD.value='f'                        
            elif colorList[0]=='B2':
                colorList.pop(0)
                print(colorList)
                kit.servo[3].angle = 90
                checkD.value='f'                       
        elif len(colorList)==0 and checkCam.value=='f' and checkD.value=='t2':
            print('============= clear T')
            checkD.value='f'
            msg="color=nothing, size=nothing"
            send_to_db(msg)

        # error scan===================================
        # 카메라 인식 초기화 시키기
        if (checkCam.value=='t1' or checkCam.value=='t2') and len(colorList)==0 and checkD.value=='f':
            checkCam.value='f'
            print('=====delete checkCam=====')
        #================================================
        
        # 물류 감지 범위 시각적으로 표현
        cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0,255,0), 2)
        
        # 영상 출력 
        cv2.imshow('original', frame)
        cv2.imshow('BLUE', res1)
        cv2.imshow('RED', res3)
        # 윈도우 위치 설정
        cv2.moveWindow('original', 50, 100)
        cv2.moveWindow('BLUE', 400, 100)
        cv2.moveWindow('RED', 400, 350)
        
        #k = cv2.waitKey(1) & 0xFF
        if cv2.waitKey(1) == ord('q'): #esc key
            print('exit')
            break
        
    cv2.destroyAllWindows()

    for i in range(4):
        kit.servo[i].angle=90

    print("DC motor and Servo motor off")
    IO.cleanup()

# dc 모터 동작 함수    
def dc_motor(motor_speed):

    # motor 2 GPIO pin assignment
    pwmPin2 = 12
    dirPin2 = 16
    # PWM setting 
    IO.setwarnings(False)
    IO.setmode(IO.BCM)
    IO.setup(pwmPin2, IO.OUT)
    IO.setup(dirPin2, IO.OUT)
    p2 = IO.PWM(pwmPin2, 100)
    p2.start(0)
    
    if motor_speed==5:
        p2.ChangeDutyCycle(0)
        print("DC motor test completed")
        IO.output(dirPin2, False) 
        IO.output(pwmPin2, False)
    
    print('motor_on')
    IO.output(dirPin2, 1)
    p2.ChangeDutyCycle(motor_speed)
    

if __name__=="__main__":
    thread2 = Process(target=cam, args=())
    thread = Process(target=distance_check, args=())
    thread2.start()
    thread.start()
    thread2.join()
    thread.join()

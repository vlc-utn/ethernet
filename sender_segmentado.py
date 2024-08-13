import tkinter as tk
from tkinter import ttk
import cv2, imutils, socket
import numpy as np
import time
import base64
import threading, wave, pyaudio,pickle,struct
import sys
import queue
import os

def cambio_flag():
    
    global flag_tx

    if(flag_tx==False):
        flag_tx = True
    elif(flag_tx==True):
        flag_tx = False

def video_stream_gen():
    
    WIDTH=600
    while(vid.isOpened()):
        try:
            _,frame = vid.read()
            frame = imutils.resize(frame,width=WIDTH)
            q.put(frame)
        except:
            os._exit(1)
    print('Player closed')
    BREAK=True
    vid.release()
	

def video_stream():
    global TS
    fps,st,frames_to_count,cnt = (0,0,15,0)
    fragment_size = 2048                                    # Tamaño de cada fragmento en vitis
    #sender_socket.sendto(message,(host_ip,port))
    cv2.namedWindow('TRANSMITTING VIDEO')        
    cv2.moveWindow('TRANSMITTING VIDEO', 10,30)
        
    while(True):

        frame = q.get()
        encoded,buffer = cv2.imencode('.jpeg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
        message = base64.b64encode(buffer)
        for i in range(0, len(message), fragment_size):
            fragment = message[i:i + fragment_size]
            sender_socket.sendto(fragment, (host_ip,port))
        frame = cv2.putText(frame,'FPS: '+str(round(fps,1)),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
        if cnt == frames_to_count:
            try:
                fps = (frames_to_count/(time.time()-st))
                st=time.time()
                cnt=0
                if fps>FPS:
                    TS+=0.001
                elif fps<FPS:
                    TS-=0.001
                else:
                    pass
            except:
                pass
        cnt+=1

        msg,client_addr = sender_socket.recvfrom(16)
        print('GOT connection from',client_addr, 'said',msg)
            
        cv2.imshow('TRANSMITTING VIDEO', frame)
        key = cv2.waitKey(int(1000*TS)) & 0xFF	
        if key == ord('q'):
            os._exit(1)
            TS=False
            break	

def graficar():

    root = tk.Tk()
    root.geometry("+30+500")
    root.config(width=300, height=300)
    root.title("Play transmisor")

    img_boton = tk.PhotoImage(file="gui/play.resized.png")

    boton = ttk.Button(image=img_boton, command=cambio_flag)
    boton.place(x=86, y=86)
    root.mainloop()

#---------------------------------------------------------------------------------------------------------------------#
q = queue.Queue(maxsize=50)
flag_tx=False

BUFF_SIZE = 65536
host_ip = '192.168.1.10'
port = 5001
# filename = Completar con nombre de archivo de video 
message = b'Hello'

#---------------------------------------------------------------------------------------------------------------------#
command = "ffmpeg -i {} -ab 160k -ac 2 -ar 44100 -vn {}".format(filename,'temp.wav')
os.system(command)

#------------------------------------------------------UDP Socket-----------------------------------------------------#
sender_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sender_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)
#---------------------------------------------------------------------------------------------------------------------#

vid = cv2.VideoCapture(filename)
FPS = vid.get(cv2.CAP_PROP_FPS)
global TS
TS = (0.5/FPS)
BREAK=False
totalNoFrames = int(vid.get(cv2.CAP_PROP_FRAME_COUNT))
durationInSeconds = float(totalNoFrames) / float(FPS)
d=vid.get(cv2.CAP_PROP_POS_MSEC)

from concurrent.futures import ThreadPoolExecutor
with ThreadPoolExecutor(max_workers=3) as executor:
    executor.submit(graficar)
    while True:
        time.sleep(1)
        if(flag_tx==True):
            executor.submit(video_stream)
            executor.submit(video_stream_gen)
            flag_tx= False
            break

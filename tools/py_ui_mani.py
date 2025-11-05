#!/usr/bin/env python3
'''
Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============


======================================================'''
import socket
import tkinter as tk
import struct
from threading import Thread
import time
import sys

ip='192.168.1.201'
# ip='192.168.54.110'
# ip='0.0.0.0'
port=8000

KeyEn=1
KeyDis=13
KeyManiIdle=62
KeyManiDamp=63
KeyManiRc=64
KeyManiAct=65
KeyManiMani=66


sk=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

cmd=bytearray([63,0,0,0,   20,0,0,0,   1,0,0,0,
				0,0,0,0,
				0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0])

def skLoop():
    while(1):
        sk.sendto(cmd,(ip,port))
        time.sleep(0.5)

vx=0;vy=0;wz=0
def clearV():
    global vx,vy,wz
    vx=0;vy=0;wz=0
    updateVcmd()
def updateVcmd():
    cmd[16:20]=struct.pack('f',vx)
    cmd[20:24]=struct.pack('f',vy)
    cmd[24:28]=struct.pack('f',wz)
    
def setCmd(key):
    cmd[12:16]=struct.pack("i",key)
def setCmdV0(key):
    clearV()
    cmd[12:16]=struct.pack("i",key)

def butDis():
    clearV()
    cmd[12]=13
    sk.sendto(cmd,(ip,port))
def butClearErr():
    clearV()
    tmp=cmd[12]
    cmd[12]=29
    sk.sendto(cmd,(ip,port))
    cmd[12]=tmp

def keyPress(ev):
    global vx,vy,wz
    key=ev.keysym
    print(key)
    if(key=='KP_7'):
        setCmdV0(KeyEn)
    elif(key=='KP_8'):
        setCmdV0(KeyManiRc)
    elif(key=='KP_9'):
        setCmdV0(KeyManiIdle)
    elif(key=='KP_1'):
        setCmdV0(KeyManiAct)
    elif(key=='KP_2'):
        setCmdV0(KeyManiMani)
    elif(key=='KP_3'):
        setCmdV0(KeyManiDamp)
    elif(key=='KP_0'):
        butDis()
    elif(key=='space'):
        clearV()

root=tk.Tk()
root.configure(background='#ccc')
if sys.platform.startswith('win'):
    root.geometry('800x400')
    length=100
else:
    if(root.winfo_screenwidth()>5000):
        root.geometry('1200x600+4600+1000')
        length=120
    else:
        root.geometry('1000x500+2600+200')
        length=100

tk.Button(root,text='清错',command=butClearErr).place(anchor='center',relx=0.1,rely=0.1,width=length,height=length)
tk.Label(root,text="方括号数字是发送值,-数字是键盘").place(anchor='center',relx=0.5,rely=0.05)

tk.Button(root,text='[1]Y\nen-7',command=lambda:setCmdV0(KeyEn)).place(anchor='center',relx=0.7,rely=0.2,width=length,height=length)
tk.Button(root,text='[2]X\nrc-8',command=lambda:setCmdV0(KeyManiRc)).place(anchor='center',relx=0.6,rely=0.35,width=length,height=length)
tk.Button(root,text='[3]A\nidle-9',command=lambda:setCmdV0(KeyManiIdle)).place(anchor='center',relx=0.7,rely=0.5,width=length,height=length)
tk.Button(root,text='[4]B\nwk-4').place(anchor='center',relx=0.8,rely=0.35,width=length,height=length)

tk.Button(root,text='[6]↑\nstart-5').place(anchor='center',relx=0.3,rely=0.2,width=length,height=length)
tk.Button(root,text='[7]←\nstop-6').place(anchor='center',relx=0.2,rely=0.35,width=length,height=length)
tk.Button(root,text='[13]↓\ndis-0',command=butDis).place(anchor='center',relx=0.3,rely=0.5,width=length,height=length)
tk.Button(root,text='[20]→\ndamp-3',command=lambda:setCmdV0(KeyManiDamp)).place(anchor='center',relx=0.4,rely=0.35,width=length,height=length)

tk.Button(root,text='[1]\nen',command=lambda:setCmdV0(KeyEn)).place(anchor='center',relx=0.1,rely=0.7,width=length,height=length)
tk.Button(root,text='[13]\ndis',command=lambda:setCmdV0(KeyDis)).place(anchor='center',relx=0.2,rely=0.7,width=length,height=length)
tk.Button(root,text='[62]\nidle',command=lambda:setCmdV0(KeyManiIdle)).place(anchor='center',relx=0.3,rely=0.7,width=length,height=length)
tk.Button(root,text='[63]\ndamp',command=lambda:setCmdV0(KeyManiDamp)).place(anchor='center',relx=0.4,rely=0.7,width=length,height=length)
tk.Button(root,text='[64]\nrc',command=lambda:setCmdV0(KeyManiRc)).place(anchor='center',relx=0.4,rely=0.7,width=length,height=length)
tk.Button(root,text='[65]\nact',command=lambda:setCmdV0(KeyManiAct)).place(anchor='center',relx=0.5,rely=0.7,width=length,height=length)
tk.Button(root,text='[66]\nmani',command=lambda:setCmdV0(KeyManiMani)).place(anchor='center',relx=0.6,rely=0.7,width=length,height=length)
tips=['','暂停','启动','回正/遥操','抱拳/遥操','左抬/遥操','右抬/遥操','左挥/遥操','右挥/遥操','握拳/遥操','','','','','','','','','']
from functools import partial
for i in range(1,10):
    # tk.Button(root,text=f'[10{i}]\n{tips[i]}',command=lambda:setCmdV0(100+i)).place(anchor='center',relx=0.1*i-0.05,rely=0.9,width=length,height=length)
    tk.Button(root,text=f'[10{i}]\n{tips[i]}',command=partial(setCmdV0, 100+i)).place(anchor='center',relx=0.1*i-0.05,rely=0.9,width=length,height=length)

root.bind('<Key>',keyPress)
# root.bind('<FocusIn>',focus)
th=Thread(target=skLoop)
th.daemon=1
th.start()
tk.mainloop()


# struct cmdStruct{
# 	int head=99;
# 	int size;//32
# 	int id;
# 	int key;
# 	float joy[4];
# };

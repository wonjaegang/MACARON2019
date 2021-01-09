#!/usr/bin/env python
#-*-coding:utf-8-*_

## talker demo that published std_msgs/ColorRGBA messages
## to the 'color' topic. To see these messages, type:
## rostopic echo color
## this demo shows some of the more advanced APIs in rospy
import sys,time
from scipy.integrate import quad
from scipy.interpolate import InterpolatedUnivariateSpline
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
import math
import roslib
roslib.load_manifest('macaron')

import rospy

import matplotlib.pyplot as plt
from macaron.msg import Floats

a=np.loadtxt("catkin_ws/src/macaron/scripts/eightlight/8line.txt",delimiter=',',dtype='double')
line_name="catkin_ws/src/macaron/scripts/eightlight/8line_npy"+".npy"
np.save(line_name,a)
b=np.load(line_name)

print(b)
plt.plot(b[:,0],b[:,1])
plt.grid(True)
plt.show()

#from std_msgs.msg import ColorRGBA
P=np.array(([0,10,30,40,50,70,90,90,80,70,60],[0,10,10,0,0,10,30,40,50,50,50]),dtype="double")
leng=int(P.size/2)
PI=math.acos(-1)
print(PI)
#macaron property
wheel_base = 1.040
tread = 0.985
width = 1.160
angle_now = 0                              
d_camera_p = 1

candidate_num = 7
look_ahead_num = 100
w_offset = 1
w_safety = 1
w_consistency = 1


alpha = 0.5 # 접선벡터의 가중치. 이 값에 따라 곡선의 장력이 결정된다.
dp = np.zeros((2, leng))
print(dp)

for xy in range(0,2):
    for a in range(1,leng-1):
        dp[xy, a] = (1 - alpha)*(P[xy, a+1] - P[xy, a-1])

print(dp)
dt = 0.1
Nsample = int(1/ dt)
p = np.zeros((2, Nsample * (leng-1) + 1))
poly=np.zeros((4,11,2))

print(p.shape)
for big_node in range(0,leng -1):
    for xy in range(0,2):
        t = 0
        poly[0][big_node][xy]= +2*P[xy,big_node] -2*P[xy,big_node+1]+1*dp[xy,big_node]+1*dp[xy,big_node + 1]
        poly[1][big_node][xy]= -3*P[xy,big_node] +3*P[xy,big_node+1]-2*dp[xy,big_node]-1*dp[xy,big_node + 1]
        poly[2][big_node][xy] = dp[xy,big_node]
        poly[3][big_node][xy] = P[xy,big_node]
        for small_node in range(0,Nsample):
            p[xy][small_node + Nsample * big_node] = poly[0][big_node][xy]*t*t*t + poly[1][big_node][xy]*t*t + poly[2][big_node][xy]*t + poly[3][big_node][xy]
            t=t+dt

print(poly)

p[0, Nsample * (leng-1)] = P[0, leng-1]
p[1, Nsample * (leng-1)] = P[1, leng-1]

plt.plot(p[0][:],p[1][:], 'r', label='f(x)')
plt.grid(True)

x=p[0,:]
y=p[1,:]

Fsum=0
err_sum=0
x=P[0,:]
y=P[1,:]
st=[0]
'''
for big_node in range(0,leng-1):
        ft = lambda t: ((3* poly[0][big_node][1]*t*t + 2*poly[1][big_node][1]*t + poly[2][big_node][1])**2+(3* poly[0][big_node][0]*t*t + 2*poly[1][big_node][0]*t + poly[2][big_node][0])**2)**0.5
                for big_node in range(0,leng-1):
                F,err=quad(ft, float(big_node)/10,float(big_node+1)/10)
                Fsum=Fsum+F
                s.append(Fsum)

        fpx=np.poly1d(np.polyfit(s,p[0][0:(Nsample+1)],3))
        fpy=np.poly1d(np.polyfit(s,p[1][0:(Nsample+1)],3))
        t=np.arange(0,14,1)
        ix=fpx(t)
        iy=fpy(t)

        print(ix)
        print(iy)
plt.plot(x,y, 'r', label='f(x)')
plt.plot(ix, iy, 'bo', label='lins')

print((s))


plt.grid(True)
plt.show()


for big_node in range(0,leng-1):
        ft = lambda t: ((3* poly[0][big_node][1]*t*t + 2*poly[1][big_node][1]*t + poly[2][big_node][1])**2+(3* poly[0][big_node][0]*t*t + 2*poly[1][big_node][0]*t + poly[2][big_node][0])**2)**0.5
        l_hat=np.linspace(0,14.4,10)
        t1=0.
        t2=1.
        tmid=(t1+t2)/2
        result=100.
        for node in range(1,11):
                while abs(result)>0.001:
                        result=14.4-quad(ft,t1,tmid)
                        if(result>0):
                                tmid=(t2+tmid)/2
                        else :
                                tmid=(t1+tmid)/2
                st.append(tmid+big_node*100)
                Fsum=Fsum+quad( ft, t1,tmid)


print(st)
print(s)



itx=InterpolatedUnivariateSpline(s,x,k=4)
ity=InterpolatedUnivariateSpline(s,y,k=4)

ix=itx()
iy=ity(np.linspace(0,155,1))
plt.plot(x,y, 'r', label='f(x)')
plt.plot(ix, iy, 'b', label='interpolation')
plt.legend(loc=0)
plt.grid(True)
plt.show()







V = [ [0:0.01:10]', x.', y.' ] 
L = Fsum;
s0 = 0;
N = length(s);
Xq = np.linspace(s0,L,(leng-1)*Nsample); % equally spaced indices

 

Vq = interp1(X,V(1:1000,:),Xq);

xs = Vq(:,2);

ys = Vq(:,3);
'''
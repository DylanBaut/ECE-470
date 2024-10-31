import numpy as np
xWgrip = .1
yWgrip = .1
zWgrip = .15
yaw_WgripDegree = np.pi
l1 = .152
l2 = .12
l6 = .083
l3 = .244
l5 = .213
l7 = .083
xgrip = xWgrip+.150 
ygrip = yWgrip-.150
zgrip = zWgrip-.010

xcen = xgrip - np.cos(yaw_WgripDegree)*.0535
ycen = ygrip - np.sin(yaw_WgripDegree)*.0535
zcen = zgrip
# atheta = np.arccos((l2**2-(l3+l5+l7)**2-(xcen**2+ycen**2))/(-2*(l3+l5+l7)*(np.sqrt(xcen**2+ycen**2)))) 
# c1 = np.sqrt((l7**2)+(.027+l6)**2)
xtheta = 2*np.pi- np.arctan((.027+l6)/(l7))
ktheta = np.pi - np.arctan((l7)/(.027+l6))
atheta = 2*np.pi - ktheta-xtheta

theta1 = np.arctan2(ycen,xcen)-atheta

x3 = .137*np.sin(.6405-theta1)-xcen
y3 = .137*np.cos(.6405-theta1)-ycen
z3 = .059+ .082
th2C = np.sqrt((z3-l1)**2+(x3**2+y3**2))

tsmall = (l5**2-th2C**2-l3**2)/(2*th2C*l3)

smalltheta2 = np.arccos(tsmall)
theta2 = (np.arctan((z3-l1)/(np.sqrt(x3**2+y3**2)))+smalltheta2)
theta3 = np.pi-np.arccos((th2C**2-l3**2-l5**2)/(2*l5*l3))
theta4 = np.pi-smalltheta2+theta3-(np.pi/2)
theta5 = -np.pi/2
theta6 = np.pi/2+theta1-yaw_WgripDegree 

print(theta1,"\n",theta2,"\n",theta3,"\n",theta4,"\n",theta5,"\n",theta6)
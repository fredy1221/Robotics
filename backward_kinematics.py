import numpy as np
import math

a1_z=650
a2_x=400
a2_z=680
a3_z=1100
a4_x=766
a4_z=230
a5_x=345
a6_x=244

def eulerAnglesToRotationMatrix(theta) :

    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])



    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])


    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

J=np.array([0,0,0,0,0,0],dtype="float64")
print(J.dtype)
TCP=np.array([[1500],[1000],[2000]])
rotation=np.array([0*math.pi/180,0*math.pi/180,0*math.pi/180])


R=eulerAnglesToRotationMatrix(rotation)
print("R=",R,"\n")

x_hat=np.dot(R,[[1],[0],[0]])
print("x hat = ",x_hat,"\n")
WP=TCP-a6_x*x_hat
print("WP= ",WP,"\n")

if(WP[1,0]==0 or WP[0,0]==0):
    singularity=1
    print("singularity","\n")
    J[0]=0
else:
    J[0]=math.atan(WP[1,0]/WP[0,0])
    print("not singularity","\n")

WPxy=pow(pow(WP[0,0],2)+pow(WP[1,0],2),0.5)
L=WPxy-a2_x
H=WP[2,0]-a1_z-a2_z
Ro=pow(pow(H,2)+pow(L,2),0.5)
b4_x=pow(pow(a4_z,2)+pow(a4_x+a5_x,2),0.5)

if(Ro>a3_z+b4_x or Ro<abs(a3_z-b4_x)):
    print("position too far","\n")

alfa=math.atan(H/L)
beta=math.acos((pow(Ro,2)+pow(a3_z,2)-pow(b4_x,2))/(2*Ro*a3_z))
J[1]=(math.pi)/2-alfa-beta
#J[1]=(math.pi)/2-alfa+beta if we want the down position

gamma=math.acos((pow(a3_z,2)+pow(b4_x,2)-pow(Ro,2))/(2*b4_x*a3_z))
delta=math.atan((a4_x+a5_x)/a4_z)
J[2]=math.pi-gamma-delta

R1=np.array([[math.cos(J[0]),-math.sin(J[0]),0],[math.sin(J[0]),math.cos(J[0]),0],[0,0,1]])
R2=np.array([[math.cos(J[1]),0,math.sin(J[1])],[0,1,0],[-math.sin(J[1]),0,math.cos(J[1])]])
R3=np.array([[math.cos(J[2]),0,math.sin(J[2])],[0,1,0],[-math.sin(J[2]),0,math.cos(J[2])]])

print("R1 = ",R1)

R_arm=np.matmul(R1,R2)
R_arm=np.matmul(R_arm,R3)
print("R_arm = ",R_arm,"\n")

R_arm_transposed=np.transpose(R_arm)
print("R_arm_transposed = ",R_arm_transposed,"\n")

R_wrist=np.dot(R_arm_transposed,R)
print("R wrist = ",R_wrist,"\n")

J[4]=math.atan((pow(1-pow(R_wrist[0,0],2),0.5))/R_wrist[0,0])
#J[4]=math.acos(R_wrist[0,0])

if(J[4]==1):
    J[3]=0 #wrist singularity
    J[5]=math.atan(R[2,1]/R[2,2])-J[3] ##J4 and J6 are dependant, we can choose the current value of J4
else:
    J[3]=math.atan(-R_wrist[1,0]/R_wrist[2,0])
    J[5]=math.atan(R_wrist[0,1]/R_wrist[0,2])

for j in J:
    print (j*(180/math.pi))


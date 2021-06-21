##calculate position and orientation of TCP from joints
import numpy as np
import math
Z=[[0,0,0]]
pi=math.pi

x0=0
y0=0
z0=0

#change joints angles here, enter value in degrees:
J1=124.3*(pi/180)
J2=-24.7*(pi/180)
J3=43.6*(pi/180)
J4=-42.8*(pi/180)
J5=65.6*(pi/180)
J6=140.8*(pi/180)

a1_z=650
a2_x=400
a2_z=680
a3_z=1100
a4_x=766
a4_z=230
a5_x=345
a6_x=244

R1=np.array([[math.cos(J1),-math.sin(J1),0],[math.sin(J1),math.cos(J1),0],[0,0,1]])
T1=np.append(R1,Z,axis=0)
T1=np.append(T1,[[0],[0],[a1_z],[1]],axis=1)
print(R1)

R2=np.array([[math.cos(J2),0,math.sin(J2)],[0,1,0],[-math.sin(J2),0,math.cos(J2)]])
T2=np.append(R2,Z,axis=0)
T2=np.append(T2,[[a2_x],[0],[a2_z],[1]],axis=1)

R3=np.array([[math.cos(J3),0,math.sin(J3)],[0,1,0],[-math.sin(J3),0,math.cos(J3)]])
T3=np.append(R3,Z,axis=0)
T3=np.append(T3,[[0],[0],[a3_z],[1]],axis=1)

R4=np.array([[1,0,0],[0,math.cos(J4),-math.sin(J4)],[0,math.sin(J4),math.cos(J4)]])
T4=np.append(R4,Z,axis=0)
T4=np.append(T4,[[a4_x],[0],[a4_z],[1]],axis=1)

R5=np.array([[math.cos(J5),0,math.sin(J5)],[0,1,0],[-math.sin(J5),0,math.cos(J5)]])
T5=np.append(R5,Z,axis=0)
T5=np.append(T5,[[a5_x],[0],[0],[1]],axis=1)

R6=np.array([[1,0,0],[0,math.cos(J6),-math.sin(J6)],[0,math.sin(J6),math.cos(J6)]])
T6=np.append(R6,Z,axis=0)
T6=np.append(T6,[[a6_x],[0],[0],[1]],axis=1)



T=np.matmul(T1,T2)
T=np.matmul(T,T3)
T=np.matmul(T,T4)
T=np.matmul(T,T5)
T=np.matmul(T,T6)


R=np.matmul(R1,R2)
R=np.matmul(R,R3)

#print("R arm = ",R)
R=np.matmul(R,R4)
R=np.matmul(R,R5)
R=np.matmul(R,R6)
#R=R*(180/pi)
#print("R=",R)

P=np.array([[0],[0],[0],[1]])
TCP=np.matmul(T,P)



print("X=",TCP[0,0])
print("Y=",TCP[1,0])
print("Z=",TCP[2,0],"\n")


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles in radiants
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])



Rot=np.array(rotationMatrixToEulerAngles(R)*180/pi)
print("A=",Rot[0])
print("B=",Rot[1])
print("C=",Rot[2])

#R=np.matmul(R4,R5)
#R=np.matmul(R,R6)
#print(R)

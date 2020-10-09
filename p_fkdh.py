import numpy as np
import math
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import p_trigd as tr

#initial parameter
#[j0 j1 j2 j3;d0 d1 d2 d3;a0 a1 a2 a3;t0 t1 t2 t3]
#theta3=360-theta1-theta2
def dh_par(j1,j2,j3):
    j4=360-(j2+j3)
    #j=np.array([[j1, j2, j3, j4],[17.5, 0, 0, 0],[3, 22.3, 31.5, 14],[90, 0, 0, 0]], dtype=object)
    j=np.array([[j1, j2, j3, j4],[10, 0, 0, 0],[5, 25, 25, 10],[90, 0, 0, 0]], dtype=object)
    return j

def dh(theta,d,a,alpha):
    dhm=np.array([[tr.cosd(theta),-tr.sind(theta)*tr.cosd(alpha),tr.sind(theta)*tr.sind(alpha),a*tr.cosd(theta)],
    [tr.sind(theta),tr.cosd(theta)*tr.cosd(alpha),-tr.cosd(theta)*tr.sind(alpha),a*tr.sind(theta)],
    [0,tr.sind(alpha),tr.cosd(alpha),d],
    [0,0,0,1]])
    return dhm


def dh_kine(j): #dh_kine(dh_par)
    #collum 1=joint angle, 2=joint offset, 3=link lenght, 4=twist angle
    T01=dh(j[0,0],j[1,0],j[2,0],j[3,0])
    T12=dh(j[0,1],j[1,1],j[2,1],j[3,1])
    T23=dh(j[0,2],j[1,2],j[2,2],j[3,2])
    T34=dh(j[0,3],j[1,3],j[2,3],j[3,3])
    
    T02=T01@T12
    T03=T02@T23
    T04=T03@T34
    return np.hstack((T01, T02, T03, T04))

def el_xyzpos(m): #ee_xyzpos(dh_kine(j))
    Q1=np.array([0,0,m[0,3],m[0,7],m[0,11],m[0,15]])
    Q2=np.array([0,0,m[1,3],m[1,7],m[1,11],m[1,15]])
    Q3=np.array([0,m[2,3],m[2,3],m[2,7],m[2,11],m[2,15]])
    Q=np.vstack((Q1,Q2,Q3))
    return Q

def el_pos2base(Q):
    X,Y,Z=Q[0,:],Q[1,:],Q[2,:]
    p0=np.array([X[0],Y[0],Z[0]]) #base
    p1=np.array([X[1],Y[1],Z[1]])
    p2=np.array([X[2],Y[2],Z[2]])
    p3=np.array([X[3],Y[3],Z[3]])
    p4=np.array([X[4],Y[4],Z[4]])
    p5=np.array([X[5],Y[5],Z[5]]) #end effector
    return p0,p1,p2,p3,p4,p5

def fk_draw(Q):
    X,Y,Z=Q[0,:],Q[1,:],Q[2,:]
    fig = plt.figure(num=None, figsize=(14, 20), dpi=80, facecolor='w', edgecolor='k')
    #plt.rcParams["figure.figsize"] = [30, 30]
    ax = fig.add_subplot(111, projection='3d')
    
    #ax.plot(X, Y, Z, marker='o')
    #ax.plot(X,Y,Z, 'go--', linewidth=2, markersize=12)
    ax.plot(X,Y,Z, color='green', marker='o', linestyle='dashed', linewidth=2, markersize=12)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-100,100])
    ax.set_ylim([0,100])
    ax.set_zlim([0,50])
    ax.set_title('dh-fk')
    ax.view_init(30, 120)
    plt.show()
    return X,Y,Z

def fk_draw2(Q):
    X1,Y1,Z1=Q[0,0:3],Q[1,0:3],Q[2,0:3]
    X2,Y2,Z2=Q[0,2:4],Q[1,2:4],Q[2,2:4]
    X3,Y3,Z3=Q[0,3:5],Q[1,3:5],Q[2,3:5]
    X4,Y4,Z4=Q[0,4:6],Q[1,4:6],Q[2,4:6]
    fig = plt.figure(num=None, figsize=(14, 20), dpi=80, facecolor='w', edgecolor='k')
    #plt.rcParams["figure.figsize"] = [30, 30]
    ax = fig.add_subplot(111, projection='3d')
    
    #ax.plot(X, Y, Z, marker='o')
    #ax.plot(X,Y,Z, 'go--', linewidth=2, markersize=12)
    ax.plot(X1,Y1,Z1, color='green', marker='o', linestyle='solid', linewidth=5, markersize=20)
    ax.plot(X2,Y2,Z2, color='red', marker='o', linestyle='solid', linewidth=5, markersize=20)
    ax.plot(X3,Y3,Z3, color='blue', marker='o', linestyle='solid', linewidth=5, markersize=20)
    ax.plot(X4,Y4,Z4, color='black', marker='o', linestyle='solid', linewidth=5, markersize=10)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-100,100])
    ax.set_ylim([0,100])
    ax.set_zlim([0,50])
    ax.set_title('dh-fk')
    ax.view_init(20, 120)
    plt.show()
    return X1,Y1,Z1,X2,Y2,Z2,X2,Y2,Z2

def jacobian(dhkine):
    Z0=np.array([[0],[0],[1]])
    O=np.array([[0],[0],[0]])
    O4=dhkine[0:3,15:]
    Jac1=np.cross(Z0,(O4-O), axis=0)

    Z1=dhkine[0:3,2:3]
    O1=dhkine[0:3,3:4]
    Jac2=np.cross(Z1,(O4-O1), axis=0)

    Z2=dhkine[0:3,6:7]
    O2=dhkine[0:3,7:8]
    Jac3=np.cross(Z2,(O4-O2), axis=0)

    Z3=dhkine[0:3,10:11]
    O3=dhkine[0:3,11:12]
    Jac4=np.cross(Z3,(O4-O3), axis=0)
    return np.hstack((Jac1,Jac2,Jac3,Jac4))

def PinvJac(jacobian):
    return np.linalg.pinv(jacobian)

def divelo(ptarget, pstart):
    dXYZ=ptarget-pstart
    dX=dXYZ[0]
    dY=dXYZ[1]
    dZ=dXYZ[2]
    EucXY=np.sqrt(np.power(dX,2)+np.power(dY,2))#phytagoras x dan y target dan start sehingga didapat jarak x-y
    EucXYZ=np.sqrt(np.power(EucXY,2)+np.power(dZ,2))
    XYang=np.rad2deg(np.arctan2(dY,dX)) #nilai sudut (drajat)
    Zang=np.rad2deg(np.arctan2(dZ,EucXY))
    dx=tr.cosd(XYang)*EucXY
    dy=tr.sind(XYang)*EucXY
    dz=tr.sind(Zang)*EucXYZ
    return np.vstack([dx,dy,dz,EucXY,EucXYZ,Zang])

dh_kine.__doc__="homogeneous transformation matrix for dh parameters"
#def XYZKine
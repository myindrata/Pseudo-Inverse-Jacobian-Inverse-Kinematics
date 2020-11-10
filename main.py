from  matplotlib.backends.backend_qt5agg  import  ( NavigationToolbar2QT  as  NavigationToolbar ) 
import numpy as np
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import time
import random
import sys
from threading import Event
import json

#sys.path.append(".")
import p_fkdh as fk
from armgui_ik import *


class ikfk_arm(Ui_MainWindow):
    j1,j2,j3,j4,x4,y4,z4,Tm=0,0,0,0,0,0,0,0
    xt,yt,zt=0,0,0
    re_set=0
    objek={}
    def __init__(self, window):
        self.setupUi(window)
        #init
        self.initState()
        #process
        self.fkmotion()
        self.pushButton_reset.clicked.connect(self.initState)
        self.pushButton_runik.clicked.connect(self.start_ik)
        self.pushButton_rand.clicked.connect(self.randomtarget)
        self.pushGButton.clicked.connect(self.read_json)
        self.pushButton_Robot.clicked.connect(self.konek2robot)

    def konek2robot(self):
        desc="Detected.. "
        for x in range(len(self.objek)):
            desc = desc+"\n"+str(x)+": objek: "+str(self.objek[x]['nama'])+" Pos: "+str(self.objek[x]['centerbbox'])
        #self.label_RStatus.setText(str(len(self.objek)))
        self.label_RStatus.setText(desc)
        print(self.objek)

    def initState(self):
        self.re_set=1
        self.j1=90
        self.j2=60
        self.j3=-90
        self.j4=-self.j2-self.j3
        self.doubleSpinBox1.setProperty("value", self.j1)
        self.doubleSpinBox2.setProperty("value", self.j2)
        self.doubleSpinBox3.setProperty("value", self.j3)
        self.label_j1.setText(str(round(self.j1,2)))
        self.label_j2.setText(str(round(self.j2,2)))
        self.label_j3.setText(str(round(self.j3,2)))
        self.label_j4.setText(str(round(self.j4,2)))
        self.MplWidget.canvas.axes.view_init(20, 320)
        self.drawfk(self.j1,self.j2,self.j3)

    def randomtarget(self):
        self.doubleSpinBox_ik1.setValue(random.randint(-40, 40))
        self.doubleSpinBox_ik2.setValue(random.randint(5, 40))
        self.doubleSpinBox_ik3.setValue(random.randint(5, 40))

    def fkmotion(self):
        self.doubleSpinBox1.valueChanged.connect(self.eepos_fk)
        self.doubleSpinBox2.valueChanged.connect(self.eepos_fk)
        self.doubleSpinBox3.valueChanged.connect(self.eepos_fk)   
        
    def eepos_fk(self):
        self.j1=self.doubleSpinBox1.value()
        self.j2=self.doubleSpinBox2.value()
        self.j3=self.doubleSpinBox3.value()
        self.j4=-self.j2-self.j3
        self.label_j1.setText(str(round(self.j1,2)))
        self.label_j2.setText(str(round(self.j2,2)))
        self.label_j3.setText(str(round(self.j3,2)))
        self.label_j4.setText(str(round(self.j4,2)))
        
        self.drawfk(self.j1,self.j2,self.j3)

    def start_ik(self):
        self.xt=self.doubleSpinBox_ik1.value()
        self.yt=self.doubleSpinBox_ik2.value()
        self.zt=self.doubleSpinBox_ik3.value()
        #self.xt=-53.77
        #self.yt=31.04
        #self.zt=15.17
        self.re_set=0
        self.pinj_ik()

    def pinj_ik(self):
        if self.re_set==0:
            x_start=self.x4
            y_start=self.y4
            z_start=self.z4
            ptarget=np.vstack([self.xt, self.yt, self.zt])
            pstart=np.vstack([x_start,y_start,z_start])
            delta=fk.divelo(ptarget, pstart)
            step=5
            EucXYZ=delta[4]
            if abs(EucXYZ)>5:
                pembagi_step=abs(EucXYZ)/step
                dXYZ=delta[0:3]/pembagi_step
            else:
                dXYZ=delta[0:3]
                
            if abs(EucXYZ)<=0.01:
                self.MplWidget.msgwarning()
            else:
                Jac=fk.jacobian(self.Tm)
                Jac_Inv=fk.PinvJac(Jac)
                dTheta=Jac_Inv@dXYZ
                dTheta1=np.rad2deg(dTheta[0])
                dTheta2=np.rad2deg(dTheta[1])
                dTheta3=np.rad2deg(dTheta[2])
                self.j1=self.j1+dTheta1
                self.j2=self.j2+dTheta2
                self.j3=self.j3+dTheta3
                self.j4=-self.j2-self.j3
                self.label_j1.setText(str(round(self.j1[0],2)))
                self.label_j2.setText(str(round(self.j2[0],2)))
                self.label_j3.setText(str(round(self.j3[0],2)))
                self.label_j4.setText(str(round(self.j4[0],2)))
                self.drawfk(self.j1,self.j2,self.j3)
                QApplication.processEvents()
                self.pinj_ik()


    def drawfk(self,a,b,c): 
        j=fk.dh_par(a,b,c)
        self.Tm=fk.dh_kine(j)
        ee=fk.el_xyzpos(self.Tm)
        p0,p1,p2,p3,p4,p5=fk.el_pos2base(self.Tm)
        X1,Y1,Z1=ee[0,0:3],ee[1,0:3],ee[2,0:3]
        X2,Y2,Z2=ee[0,2:4],ee[1,2:4],ee[2,2:4]
        X3,Y3,Z3=ee[0,3:5],ee[1,3:5],ee[2,3:5]
        X4,Y4,Z4=ee[0,4:6],ee[1,4:6],ee[2,4:6]
        
        self.x4=X4[1]
        self.y4=Y4[1]
        self.z4=Z4[1]

        #self.doubleSpinBox_ik1.setValue(round(self.x4,2))
        #self.doubleSpinBox_ik2.setValue(round(self.y4,2))
        #self.doubleSpinBox_ik3.setValue(round(self.z4,2))

        self.label_x4.setText(str(round(self.x4,2)))
        self.label_y4.setText(str(round(self.y4,2)))
        self.label_z4.setText(str(round(self.z4,2)))

        self.MplWidget.canvas.axes.clear () 
        self.MplWidget.canvas.axes.plot (X1,Y1,Z1, color='green', marker='o', linestyle='solid', linewidth=5, markersize=10)
        self.MplWidget.canvas.axes.plot (X2,Y2,Z2, color='red', marker='o', linestyle='solid', linewidth=5, markersize=10)
        self.MplWidget.canvas.axes.plot (X3,Y3,Z3, color='blue', marker='o', linestyle='solid', linewidth=5, markersize=10)
        self.MplWidget.canvas.axes.plot (X4,Y4,Z4, color='purple', marker="h", linestyle='solid', linewidth=5, markersize=10)


        self.MplWidget.canvas.axes.text(self.x4,self.y4,self.z4,'({:.2f}, {:.2f}, {:.2f})'.format(self.x4,self.y4,self.z4), weight='bold', fontsize=12,)
        self.MplWidget.defcanvas()
        self.MplWidget.canvas.draw()
    
    def read_json(self):
        # read file
        with open('dataobjek.json', 'r') as myfile:
            data=myfile.read()
        # parse file
        obj = json.loads(data)
        self.objek=obj['objek']
        self.label_RStatus.setText("json loaded")
    
    def write_json(self,data, filename='dataobjek.json'):
        with open(filename,'w') as f: 
            json.dump(data, f) 
      
    def emptyjson(self, filename='dataobjek.json'):
        dataobj={}
        json_object = json.dumps(dataobj) 
        write_json(json_object,filename)
                    
      
class kinematic():
      def __init__(self):
            self.a=12
            self.b=3

app = QtWidgets.QApplication(sys.argv)
MainWindow=QtWidgets.QMainWindow()
ui=ikfk_arm(MainWindow)
MainWindow.show()
app.exec_()


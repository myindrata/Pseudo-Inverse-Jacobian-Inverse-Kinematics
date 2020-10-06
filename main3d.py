from  matplotlib.backends.backend_qt5agg  import  ( NavigationToolbar2QT  as  NavigationToolbar ) 

import numpy as np

import sys
#sys.path.append(".")
import p_fkdh as fk
from armgui_ik import *

class ikfk_arm(Ui_MainWindow):
    def __init__(self, window):
        self.setupUi(window)
        #init
        self.initState()
        self.drawfk()
        #process
        self.fkmotion()
        self.pushButton_reset.clicked.connect(self.initState)
    
    def initState(self):
        j1=90
        j2=60
        j3=-60
        self.doubleSpinBox1.setProperty("value", j1)
        self.doubleSpinBox2.setProperty("value", j2)
        self.doubleSpinBox3.setProperty("value", j3)
        self.MplWidget.canvas.axes.view_init(20, -120)

    def fkmotion(self):
        self.doubleSpinBox1.valueChanged.connect(self.drawfk)
        self.doubleSpinBox2.valueChanged.connect(self.drawfk)
        self.doubleSpinBox3.valueChanged.connect(self.drawfk)

    def drawfk(self): 
        j1=self.doubleSpinBox1.value()
        j2=self.doubleSpinBox2.value()
        j3=self.doubleSpinBox3.value()
        j=fk.dh_par(j1,j2,j3)
        Tm=fk.dh_kine(j)
        ee=fk.el_xyzpos(Tm)
        p0,p1,p2,p3,p4,p5=fk.el_pos2base(Tm)
        X1,Y1,Z1=ee[0,0:3],ee[1,0:3],ee[2,0:3]
        X2,Y2,Z2=ee[0,2:4],ee[1,2:4],ee[2,2:4]
        X3,Y3,Z3=ee[0,3:5],ee[1,3:5],ee[2,3:5]
        X4,Y4,Z4=ee[0,4:6],ee[1,4:6],ee[2,4:6]
        
        x4=X4[1]
        y4=Y4[1]
        z4=Z4[1]

        self.doubleSpinBox_ik1.setValue(round(x4,2))
        self.doubleSpinBox_ik2.setValue(round(y4,2))
        self.doubleSpinBox_ik3.setValue(round(z4,2))

        self.MplWidget.canvas.axes.clear () 
        self.MplWidget.canvas.axes.plot (X1,Y1,Z1, color='green', marker='o', linestyle='solid', linewidth=10, markersize=20)
        self.MplWidget.canvas.axes.plot (X2,Y2,Z2, color='red', marker='o', linestyle='solid', linewidth=10, markersize=20)
        self.MplWidget.canvas.axes.plot (X3,Y3,Z3, color='blue', marker='o', linestyle='solid', linewidth=10, markersize=20)
        self.MplWidget.canvas.axes.plot (X4,Y4,Z4, color='goldenrod', marker='o', linestyle='solid', linewidth=10, markersize=20)


        self.MplWidget.canvas.axes.text(x4,y4,z4,'({:.2f}, {:.2f}, {:.2f})'.format(x4,y4,z4), weight='bold', fontsize=12,)
        self.MplWidget.canvas.axes.set_xlabel('x')
        self.MplWidget.canvas.axes.set_ylabel('y')
        self.MplWidget.canvas.axes.set_zlabel('z')
        
        self.MplWidget.canvas.axes.set_xlim([-120,120])
        self.MplWidget.canvas.axes.set_ylim([0,120])
        self.MplWidget.canvas.axes.set_zlim([0,40])     
        
        #self.MplWidget.canvas.axes.legend (( 'cosinus', 'sinus' ), loc = 'upper right' ) 
        #self.MplWidget.canvas.axes.set_title ( ' Cosinus - Sinus Signal' ) 
        self.MplWidget.canvas.draw() 



app = QtWidgets.QApplication(sys.argv)
MainWindow=QtWidgets.QMainWindow()
ui=ikfk_arm(MainWindow)
MainWindow.show()
app.exec_()


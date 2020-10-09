# ------------------------------------------------- ----- 
# -------------------- mplwidget.py -------------------- 
# -------------------------------------------------- ---- 
from  PyQt5.QtWidgets  import * 
from  matplotlib.backends.backend_qt5agg  import  FigureCanvas 
from  matplotlib.figure  import  Figure 
    
class  MplWidget(QWidget):  
    def  __init__ ( self ,  parent  =  None ): 
        QWidget.__init__ ( self, parent)
        self.canvas  =  FigureCanvas ( Figure ()) 
        vertical_layout =  QVBoxLayout () 
        vertical_layout.addWidget (self.canvas ) 
        self.canvas.axes =  self.canvas.figure.add_subplot (111, projection='3d',position=[0.01, 0.01, 1, 1] ) 
        self.setLayout(vertical_layout )

    def defcanvas(self):
        self.canvas.axes.set_xlim([-120,120])
        self.canvas.axes.set_ylim([0,120])
        self.canvas.axes.set_zlim([0,40]) 
        self.canvas.axes.set_xlabel('x')
        self.canvas.axes.set_ylabel('y')
        self.canvas.axes.set_zlabel('z')
        #self.canvas.axes.view_init(20, 320)

    def msgwarning(self):
        QMessageBox.about(self, "Finish", "Set new target position")

        
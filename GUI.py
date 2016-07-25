import sys
import math

from PyQt4 import QtCore, QtGui
from Arduino import *



class GUI(QtGui.QWidget):

    locked = False
    cerrada = True
    angulo_garra=90
    angulo_cerrado = 95
    bsize = 40
    position =[0,15,4]
    maxY=48
    maxZ=18
    scrollcount=0
    uarm_status=0
    theta_2_thres=0
    theta_3_thres=0
    MATH_PI = 3.141592653589793238463
    ALTURA = (9.8)
    GARRA = 8.3
    BRAZO1 = 14.3
    BRAZO2 = 16.02
    CODA = 5.5

    def __init__(self):
        super(GUI, self).__init__()
        self.initUI()

    def initUI(self):

        self.resize(1000+self.bsize*2, 500+self.bsize*2)
        self.center()
        self.setWindowTitle('uArm')
        self.posLabel = QtGui.QLabel(self)
        self.posLabel.setText('Unarmed')
        self.posLabel.move(15,10)

        self.gripLabel = QtGui.QLabel(self)
        self.gripLabel.setText('Unarmed')
        self.gripLabel.move(self.width()-150, 10)

        self.box = QtGui.QFrame(self)
        self.box.setFrameShape(QtGui.QFrame.Box)

        self.theta2 = QtGui.QDoubleSpinBox(self)
        self.theta2.setRange(-90.0, 90.0)
        self.theta2.setDecimals(1)
        self.theta2.setSingleStep(0.1)
        self.theta2.move(40,self.height()-self.theta2.height()-5)
        self.theta2.valueChanged.connect(self.setTheta2)
        self.theta2.setValue(self.theta_2_thres)

        self.theta3 = QtGui.QDoubleSpinBox(self)
        self.theta3.setRange(-90.0, 90.0)
        self.theta3.setDecimals(1)
        self.theta3.setSingleStep(0.1)
        self.theta3.move(self.theta2.x()+self.theta2.width()+20, self.height() - self.theta3.height() - 5)
        self.theta3.valueChanged.connect(self.setTheta3)
        self.theta3.setValue(self.theta_3_thres)

        self.adjustBox(self.position[2])

        self.show()
        self.raise_()
        self.ser=getSerialConnect()

    def setTheta2(self):
        self.theta_2_thres = self.theta2.value()

    def setTheta3(self):
        self.theta_3_thres = self.theta3.value()

    def center(self):

        qr = self.frameGeometry()
        cp = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def mouseToPosition(self,xM,yM):

        xp = (xM-(self.width()/2))*self.maxY/(self.box.width()/2)
        yp = (self.box.y()+self.box.height()-yM)*self.maxY/self.box.height()
        return [xp,yp]

    def positionToMouse(self,xp,yp):
        xm = (self.box.width()/2)*xp/self.maxY+self.width()/2
        ym = self.box.y()+self.box.height()-(yp*self.box.height()/self.maxY)
        return [xm,ym]

    def startLock(self):
       cursor = self.cursor()  # or event.getCursor()
       initPos= self.positionToMouse(self.position[0],self.position[1])
       self.setGarra(self.angulo_cerrado,self.angulo_garra)
       self.moveTo(self.position[0], self.position[1], self.position[2])
       cursor.setPos(initPos[0]+self.x(), initPos[1]+self.y())
       self.box.setFocus()

    def wheelEvent(self, event):
        if self.locked and abs(self.scrollcount)>10:
            new_z=self.position[2]
            if self.scrollcount>0:
                new_z -= 1
            else:
                new_z += 1
            if new_z> self.maxZ and new_z>0:
                new_z = self.maxZ
            if new_z < -self.maxZ and new_z < 0:
                new_z = -self.maxZ
            self.adjustBox(new_z)
            self.moveTo(self.position[0],self.position[1],new_z)
            self.scrollcount = 0
        else:
            if self.locked :
                if event.delta() >= 0:
                    self.scrollcount -= 1
                else:
                    self.scrollcount += 1

    def mouseMoveEvent(self,event):
      if self.locked:
        mx=event.x()
        my=event.y()
        armPos= self.mouseToPosition(mx,my)
        self.moveTo(armPos[0],armPos[1],self.position[2])

    def mousePressEvent(self, event):
        if self.locked:
            z=self.position[2]
            self.abrirGarra()
            time.sleep(0.5)
            self.moveToTime(self.position[0],self.position[1],-0.01,1)
            time.sleep(1.5)
            self.cerrarGarra()
            time.sleep(1)
            self.moveToTime(self.position[0], self.position[1], z,0.5)

    def moveTo(self, x, y, z):
        self.moveToTime(x,y,z,0)

    def moveToTime(self, x, y, z, timeSpend):

        if self.uarm_status == 0:
            self.uarm_status = 1
            interpol = {}
            curXYZ = self.position
            x_arr = {}
            y_arr = {}
            z_arr = {}

            if time > 0:
                interpol=self.interpolation(curXYZ[0], x)
                for n in range(0, 50):
                    x_arr[n] = interpol[n]

                interpol =self.interpolation(curXYZ[1], y)
                for n in range(0, 50):
                    y_arr[n] = interpol[n]

                interpol =self.interpolation(curXYZ[2], z)
                for n in range(0, 50):
                    z_arr[n] = interpol[n]

                for n in range(0, 50):
                    self.setPosition(x_arr[n], y_arr[n], z_arr[n])
                    time.sleep(timeSpend / 50.0)

            elif time == 0:
                self.setPosition(x, y, z)

            self.uarm_status = 0

    def interpolation(self, init_val, final_val):

        g_interpol_val_arr = {}
        l_time_total = 1

        l_a_0 = init_val;
        l_a_1 = 0;
        l_a_2 = (3 * (final_val - init_val)) / (l_time_total * l_time_total);
        l_a_3 = (-2 * (final_val - init_val)) / (l_time_total * l_time_total * l_time_total);

        i = 0
        while i < 51:
            l_t_step = (l_time_total / 50.0) * i
            g_interpol_val_arr[i] = l_a_0 + l_a_1 * (l_t_step) + l_a_2 * (l_t_step * l_t_step) + l_a_3 * (
            l_t_step * l_t_step * l_t_step);
            i += 1
        return g_interpol_val_arr


    def setPosition(self,new_x,new_y,new_z):
        self.position[0] = new_x
        self.position[1] = new_y
        self.position[2] = new_z
        [alpha, beta, gamma] = self.calAngles(new_x, new_y, new_z)
        self.ser.write(chr(0xFF))
        self.ser.write(chr(0xAA))
        self.ser.write(chr((alpha & 0xFF00) >> 8))
        self.ser.write(chr(alpha & 0x00FF))
        self.ser.write(chr((beta & 0xFF00) >> 8))
        self.ser.write(chr(beta & 0x00FF))
        self.ser.write(chr((gamma & 0xFF00) >> 8))
        self.ser.write(chr(gamma & 0x00FF))

        self.posLabel.setText(
            "x: " + str(self.position[0]) + " y: " + str(self.position[1]) + " z: " + str(self.position[2]))
        self.posLabel.adjustSize()

    def setGarra(self, angulocerrado, angulogarra):

        if self.uarm_status == 0:
            self.uarm_status = 1
            self.angulo_garra = min(max(angulogarra,0),180)
            self.angulo_cerrado = min(max(angulocerrado,30),95)

            anguloreal= int(self.angulo_garra*160.0/180.0)
            anguloreal2 = int(self.angulo_cerrado)
            self.ser.write(chr(0xFF))
            self.ser.write(chr(0xBB))
            self.ser.write(chr((anguloreal & 0xFF00) >> 8))
            self.ser.write(chr(anguloreal & 0x00FF))
            self.ser.write(chr((anguloreal2 & 0xFF00) >> 8))
            self.ser.write(chr(anguloreal2 & 0x00FF))
            if anguloreal2== 95:
                self.gripLabel.setText(
                    "angle: " + str(self.angulo_garra) + " closed")
            else:
                if anguloreal2 == 30:
                    self.gripLabel.setText(
                        "angle: " + str(self.angulo_garra) + " open")
                else:
                    self.gripLabel.setText(
                        "angle: " + str(self.angulo_garra) + " claw: " + str(self.angulo_cerrado) )
            self.gripLabel.adjustSize()
            self.uarm_status = 0

    def abrirGarra(self):
        self.setGarra(30, self.angulo_garra)
        self.cerrada = False

    def cerrarGarra(self):
        self.setGarra(95, self.angulo_garra)
        self.cerrada = True

    def calAngles(self, x, y, z):

        g_theta_1 = math.atan2(y, x) * 180 / math.pi
        z_in = z + self.GARRA - self.ALTURA
        y_rot = -math.sin((g_theta_1 - 90) * self.MATH_PI / 180) * x + math.cos((g_theta_1 - 90) * self.MATH_PI / 180) * y
        y_in = y_rot - self.CODA
        S = math.sqrt(math.pow(z_in, 2) + math.pow(y_in, 2))
        g_theta_2 = (math.acos(max(min(
            (math.pow(S, 2) + math.pow(self.BRAZO1, 2) - math.pow(self.BRAZO2, 2)) / (2 * S * self.BRAZO1),1),-1)) + math.asin(
            z_in / S)) * 180 / self.MATH_PI
        g_theta_3 = (math.acos(max(min(
            (math.pow(self.BRAZO1, 2) + math.pow(self.BRAZO2, 2) - math.pow(S, 2)) / (2 * self.BRAZO1 * self.BRAZO2),1),-1))) * 180 / self.MATH_PI
        g_theta_3 = g_theta_3 - (90 - g_theta_2)

        # g_theta_1 = min(max(1.0435534997619*g_theta_1-16.915067002245, 0), 180)
        # g_theta_2 = min(max(1.4965517241379+1.0825615763547*g_theta_2 , 0), 140)
        # g_theta_3 = max(min(118.73333333333-0.98*g_theta_3, 125), 20)

        g_theta_1 = min(max((g_theta_1+16.915067002245)/1.0435534997619, 0), 180)
        g_theta_2 = min(max((-1.4965517241379+g_theta_2)/1.0825615763547 , 0), 140)
        g_theta_3 = max(min((118.73333333333-g_theta_3)/0.98, 125), 20)

        # g_theta_2 = min(max(g_theta_2-10, 0), 140)
        # g_theta_3 = max(min(90-g_theta_3+30, 125), 20)

        return [int(g_theta_1), int(g_theta_2+self.theta_2_thres), int(g_theta_3+self.theta_3_thres)]

    def adjustBox(self, z):
        self.maxY= max(min(48-23*abs(z)/18,48),1)
        self.box.resize((self.width()-self.bsize*2)*self.maxY/48,(self.height()-self.bsize*2)*self.maxY/48)
        self.box.move(self.bsize+(self.width()-self.bsize*2)*(48-self.maxY)/96,self.bsize+(self.height()-self.bsize*2)*(48-self.maxY)/48)

    def keyPressEvent(self, event):
       key = event.key()
       if key == QtCore.Qt.Key_Return:
          if self.locked is False:
            self.setCursor(QtGui.QCursor(QtCore.Qt.CrossCursor))
            self.startLock()
            self.setMouseTracking(True)
            self.box.setMouseTracking(True)
            self.locked = True
          else:
            self.unsetCursor()
            self.setMouseTracking(False)
            self.box.setMouseTracking(False)
            self.locked = False
            self.posLabel.setText('Unarmed')
            self.posLabel.adjustSize()
            self.gripLabel.setText('Unarmed')
            self.gripLabel.adjustSize()
       if self.locked:
           if key == QtCore.Qt.Key_Space:
               if self.cerrada is False:
                   self.cerrarGarra()
               else:
                   self.abrirGarra()
           if key == QtCore.Qt.Key_Z:
               self.setGarra(self.angulo_cerrado-5,self.angulo_garra)
           if key == QtCore.Qt.Key_X:
               self.setGarra(self.angulo_cerrado + 5, self.angulo_garra)
           if key == QtCore.Qt.Key_A:
               self.setGarra(self.angulo_cerrado, self.angulo_garra - 5)
           if key == QtCore.Qt.Key_D:
               self.setGarra(self.angulo_cerrado, self.angulo_garra + 5)
           if key == QtCore.Qt.Key_W:
               self.moveTo(self.position[0],self.position[1],max(min(self.position[2]+1,self.maxZ),-self.maxZ))
           if key == QtCore.Qt.Key_S:
               self.moveTo(self.position[0], self.position[1], max(min(self.position[2] - 1, self.maxZ), -self.maxZ))
           if key == QtCore.Qt.Key_J:
               self.moveToTime(0,12,5,1)
               self.moveToTime(0, 21, 5,1.5)
           if key == QtCore.Qt.Key_K:
               self.moveToTime(0, 15, 15,1)
               self.moveToTime(0, 15, 0, 1.5)
               self.moveToTime(0, 15, 15, 1)
           if key == QtCore.Qt.Key_L:
               self.moveTo(-5, 15, 5)
               self.moveToTime(5, 15, 5, 1.5)



def main():

    app = QtGui.QApplication(sys.argv)
    ex = GUI()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()


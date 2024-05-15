import serial
import sys, time, re, struct
from PyQt4 import QtGui, QtCore
from DeviceController import *
import numpy
from scipy.optimize import leastsq


class RigWidget(QtGui.QWidget):
    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.ArduinoExists = False
        try:
            self.sp = serial.Serial('/dev/tty.usbserial-A800euOE', baudrate=115200, timeout=3)
            self.ArduinoExists = True
            self.ui.ArduinoConnectedText.setText('Arduino: Connected    ')
        except serial.SerialException:
            self.ui.ArduinoConnectedText.setText('Arduino: Not Connected')
            
        self.timer = QtCore.QTimer()
        self.LambdaExists = False
        try:
            self.lambdasp = serial.Serial('/dev/ttyUSB2', baudrate=9600, timeout=3)
            self.LambdaExists = True
            self.LambdaInit()
            self.ui.LambdaConnectedText.setText('Lambda: Connected    ')
        except serial.SerialException:
            self.ui.LambdaConnectedText.setText('Lambda: Not Connected')
        self.Lambda_OpenShutterA = 170
        self.Lambda_CloseShutterA = 172
        self.Lambda_OpenShutterB = 186
        self.Lambda_CloseShutterB = 188
        self.LambdaFilter = 7 # initial filter
        self.LambdaSpeed = 5 # initial speed
        self.LambdaWheel = 0 # wheel A is 0, wheel B is 1 (we don't have a B wheel)
        self.Lambda_CMD = self.SetLambdaCmd(self.LambdaFilter, speed=self.LambdaSpeed, wheel=0)
        
        
        # Temperature
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL('timeout()'), self.update)
        #QtCore.QObject.connect(self.ui.addPointBtn, QtCore.SIGNAL('clicked()'), self.addPoint)
        #QtCore.QObject.connect(self.ui.delPointBtn, QtCore.SIGNAL('clicked()'), self.delPoint)
        QtCore.QObject.connect(self.ui.sendCalBtn, QtCore.SIGNAL('clicked()'), self.sendCalibration)
        QtCore.QObject.connect(self.ui.setTempCheck, QtCore.SIGNAL('clicked()'), self.heaterCheckClicked)
        QtCore.QObject.connect(self.ui.targetSpin, QtCore.SIGNAL('valueChanged(double)'), self.targetTempChanged)
        QtCore.QObject.connect(self.ui.maxTempSpin, QtCore.SIGNAL('valueChanged(double)'), self.maxTempChanged)
        QtCore.QObject.connect(self.ui.delaySpin, QtCore.SIGNAL('valueChanged(double)'), self.delayChanged)
        # QtCore.QObject.connect(self.ui.calPointList, QtCore.SIGNAL('itemChanged(QTreeWidgetItem *, int)'), self.itemChanged)
        
        # Pump
        QtCore.QObject.connect(self.ui.pumpRunCheck, QtCore.SIGNAL('clicked()'), self.pumpRun);
        QtCore.QObject.connect(self.ui.pumpSpeedSpin, QtCore.SIGNAL('valueChanged(double)'), self.pumpSpeed);
        
        # Rig Lights
        QtCore.QObject.connect(self.ui.rigLightsOff, QtCore.SIGNAL('clicked()'), self.rigLightsOff);
        QtCore.QObject.connect(self.ui.rigLightRed, QtCore.SIGNAL('clicked()'), self.rigLightRed);
        QtCore.QObject.connect(self.ui.rigLightDimWhite, QtCore.SIGNAL('clicked()'), self.rigLightDimWhite);
        QtCore.QObject.connect(self.ui.rigLightBrightWhite, QtCore.SIGNAL('clicked()'), self.rigLightBrightWhite);

        # Valve Selection
        QtCore.QObject.connect(self.ui.valveAllOff, QtCore.SIGNAL('clicked()'), self.valveAllOff);
        QtCore.QObject.connect(self.ui.valveSelect1, QtCore.SIGNAL('clicked()'), self.valveSelect1);
        QtCore.QObject.connect(self.ui.valveSelect2, QtCore.SIGNAL('clicked()'), self.valveSelect2);
        QtCore.QObject.connect(self.ui.valveSelect3, QtCore.SIGNAL('clicked()'), self.valveSelect3);
        QtCore.QObject.connect(self.ui.valveSelect4, QtCore.SIGNAL('clicked()'), self.valveSelect4);
        QtCore.QObject.connect(self.ui.computerCheck, QtCore.SIGNAL('clicked()'), self.valveComputerChecked);
#        QtCore.QObject.connect(self.ui.valveCleaning, QtCore.SIGNAL('clicked()'), self.valveCleaning);

        # Lambda 10-2
        QtCore.QObject.connect(self.ui.LambdaShutterCheckA, QtCore.SIGNAL('clicked()'), self.LambdaShutter);
        QtCore.QObject.connect(self.ui.LambdaFilterSpinA, QtCore.SIGNAL('valueChanged(double)'), self.LambdaFilterA);
        
        devices = self.getQuery() # find out which devices are enabled
        firstDevice = []
        print devices
        maplist = {'temp': 0, 'valves': 1, 'pump': 2, 'leds': 3, 'riglights': 4, 'timers': 5, 'lambda': 6}
        for device, v in devices.items():
            if v == 1:
                self.ui.tabWidget.setTabEnabled(maplist[device], True)
                firstDevice.append(maplist[device])
            else:
                self.ui.tabWidget.setTabEnabled(maplist[device], False)
        if firstDevice is not []:
            firstDevice = min(firstDevice)
            self.ui.tabWidget.setCurrentIndex(firstDevice)
        # if devices['valves'] == 0:
        #     self.ui.tabWidget.setTabEnabled(1, False)
        # if devices['pump'] == 0:
        #     self.ui.tabWidget.setTabEnabled(2, False)
        # if devices['leds'] == 0:
        #     self.ui.tabWidget.setTabEnabled(3, False)
        # if devices['riglights'] == 0:
        #     self.ui.tabWidget.setTabEnabled(4, False)
        # if devices['timers'] == 0:
        #     self.ui.tabWidget.setTabEnabled(5, False)
        # if devices['lambda'] == 0:
        #     self.ui.tabWidget.setTabEnabled(6, False)
        # 
        # self.ui.tabWidget.setTabEnabled(firstDevice, True)
        
        
        self.timer.start(1000)
        self.pointList = []

    def ArduinoWrite(self, cmd):
        if self.ArduinoExists:
            self.sp.write(cmd)
    
    def ArduinoReadline(self):
        if self.ArduinoExists:
            return(self.sp.readline())
        else:
            return None
            
    def LambdaWrite(self, cmd):
        if self.LambdaExists:
            self.lambdasp.write(cmd)

    def LambdaRead(self):
        if self.LambdaExists:
            return(self.lambdasp.readline())
        else:
            return None
            
    def update(self):
        state = self.getState()
        if state is None:
            return
        self.ui.tempLabel.setText('%0.1f C' % state['t1'])
        self.ui.heaterLabel.setText('%0.1f C' % state['t2'])
        
        self.ui.targetSpin.blockSignals(True)
        self.ui.targetSpin.setValue(state['target'])
        self.ui.targetSpin.blockSignals(False)
        
        self.ui.maxTempSpin.blockSignals(True)
        self.ui.maxTempSpin.setValue(state['heatMax'])
        self.ui.maxTempSpin.blockSignals(False)
        
        self.ui.delaySpin.blockSignals(True)
        self.ui.delaySpin.setValue(state['delay'])
        self.ui.delaySpin.blockSignals(False)
        
        if state['enabled'] == 1:
            self.ui.heaterLevelLabel.setText("%d%% (%d)" % (state['output']*100, state['outputPWM']))
            self.ui.setTempCheck.setChecked(True)
        else:
            self.ui.heaterLevelLabel.setText('- off -')
            self.ui.setTempCheck.setChecked(False)
            

    def getQuery(self):
        if not self.ArduinoExists:
            return None
        for i in range(3):
            s = self.sp.read(self.sp.inWaiting())
            if len(s) > 0:
                print "::", s.strip()
            self.sp.write('q') # send query
            res = self.sp.readline()
            if res == '':
                print "Timed out getting state from controller."
            try:
                (temp_used, valves_used, pump_used, leds_used, riglights_used, timers_used, lambda_used) = res.split(',')
                return {
                    'temp': int(temp_used), 'valves': int(valves_used), 
                    'pump': int(pump_used), 'leds': int(leds_used), 
                    'riglights': int(riglights_used), 'timers': int(timers_used), 'lambda': int(lambda_used)
                }
            except:
                pass
        return None

    def getState(self): # temperature controller status
        if not self.ArduinoExists:
            return None
        for i in range(3):
            s = self.sp.read(self.sp.inWaiting())
            if len(s) > 0:
                print "::", s.strip()
            self.sp.write('?') # send query
            res = self.sp.readline()
            if res == '':
                print "Timed out getting state from controller."
            try:
                (t1, t2, v1, v2, target, hMax, delay, output, outputPWM, enable) = res.split(',')
                return {
                    't1': float(t1), 't2': float(t2), 
                    'v1': float(v1), 'v2': float(v2), 
                    'target': float(target), 'heatMax': float(hMax), 'delay': float(delay),
                    'output': float(output), 'outputPWM': float(outputPWM), 'enabled': int(enable)
                }
            except:
                pass
        return None
        
    def heaterCheckClicked(self):
        v = int(self.ui.setTempCheck.isChecked())
        cmd = 'e' + chr(v)
        self.ArduinoWrite(cmd)
        #print self.sp.readline()
        
    def targetTempChanged(self):
        v = self.ui.targetSpin.value()
        #print "target changed:", v
        cmd = 's' + struct.pack('f', v)
        self.ArduinoWrite(cmd)
        #print self.sp.readline()
        
    def maxTempChanged(self):
        v = self.ui.maxTempSpin.value()
        cmd = 'm' + struct.pack('f', v)
        self.ArduinoWrite(cmd)
        #print self.sp.readline()
        
    def delayChanged(self):
        v = self.ui.delaySpin.value()
        cmd = 'd' + struct.pack('f', v)
        self.ArduinoWrite(cmd)
        #print self.sp.readline()
        
        
    def addPoint(self):
        state = self.getState()
        item = QtGui.QTreeWidgetItem(['', '%0.2f'%state['v1'], '%0.2f'%state['v2']])
        self.pointList.append((item, [0, state['v1'], state['v2']]))
        item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsEnabled)
        self.ui.calPointList.addTopLevelItem(item)
        self.ui.calPointList.editItem(item)
        self.updateSendBtn()
        
    def delPoint(self):
        for item in self.ui.calPointList.selectedItems():
            for i in range(len(self.pointList)):
                if self.pointList[i][0] is item:
                    self.pointList.pop(i)
                    break
            self.ui.calPointList.takeTopLevelItem(self.ui.calPointList.indexOfTopLevelItem(item))
        self.updateSendBtn()
        
    def itemChanged(self, item, column):
        for i in range(len(self.pointList)):
            if self.pointList[i][0] is item:
                self.pointList[i][1][column] = float(item.text(column))
                break
        #item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
        
    def updateSendBtn(self):
        self.ui.sendCalBtn.setEnabled(self.ui.calPointList.topLevelItemCount() >= 4)

    def sendCalibration(self):
        n = len(self.pointList)
        data = numpy.empty((3, n))
        for i in range(n):
            data[:,i] = self.pointList[i][1]
            
        print "Data:", data
        
        ## find coefficients for function T = f(V)
        #     R = R1 * ((1023 / V) - 1)
        #     1/T = A + B log(R) + C log(R)^3   ## the parameter R1 gets absorbed by A, so there are only 3 variables.
            
        def fn(v, x):
            R = (1023. / x) - 1.
            lnR = numpy.log(R)
            return 1.0 / (v[0] + v[1] * lnR + v[2] * lnR * lnR * lnR)
            
        def erf(v, x, y):
            return fn(v, x)-y
            
        guess = numpy.array([0.04, 0.03, -0.01]) 
        fit1 = leastsq(erf, guess, args=(data[1], data[0]))
        fit2 = leastsq(erf, guess, args=(data[2], data[0]))
        
        print "Fit:"
        print fit1
        print fit2
        
        cmd = 'c' + fit1[0].astype(numpy.float32).tostring() + fit2[0].astype(numpy.float32).tostring()
        self.ArduinoWrite(cmd)
        #print self.sp.readline()
        

# Valve Controller

    def valveComputerChecked(self):
        pass
        
    def valveOneAtATime(self):
        pass
        
    def valveSelect(self):
        pass

    def valvesOff(self):
        pass
        
    def valveCleaning(self):
        pass


# Pump Controller
    def pumpRun(self):
        pass
        
    def pumpSpeed(self):
        pass

# Rig Lighting
    def rigLightsOff(self):
        for i in range(1,4):
            cmd = 'l' + str(i) + '0'
            self.ArduinoWrite(cmd)
    
    def rigLightRed(self):
        self.rigLightsOff;
        cmd = 'l11'
        self.ArduinoWrite(cmd);
        
    def rigLightDimWhite(self):
        self.rigLightsOff;
        cmd = 'l21'
        self.ArduinoWrite(cmd);
        
    def rigLightBrightWhite(self):
        self.rigLightsOff;
        cmd = 'l31'
        self.ArduinoWrite(cmd);

# LED Flourescence Lighting
    def ledAllOff(self):
        for i in range(1,4):
            cmd = 'L' + str(i) + '0'
            self.ArduinoWrite(cmd)

    def ledGreenSelect(self):
        self.rigLightsOff;
        cmd = 'L11'
        self.ArduinoWrite(cmd);

    def ledBlueSelect(self):
        self.rigLightsOff;
        cmd = 'L21'
        self.ArduinoWrite(cmd);

    def ledRedSelect(self):
        self.rigLightsOff;
        cmd = 'L31'
        self.ArduinoWrite(cmd);
       
# Valves
    def valveAllOff(self):
        for i in range(1,4):
            cmd = 'L' + str(i) + '0'
            self.ArduinoWrite(cmd)

    def valveSelect1(self):
        self.valveAllOff;
        cmd = 'V11'
        self.ArduinoWrite(cmd);

    def valveSelect2(self):
        self.valveAllOff;
        cmd = 'V21'
        self.ArduinoWrite(cmd);

    def valveSelect3(self):
        self.valveAllOff;
        cmd = 'V31'
        self.ArduinoWrite(cmd);                

    def valveSelect4(self):
        self.valveAllOff;
        cmd = 'V41'
        self.ArduinoWrite(cmd);                

# Lambda 10-2 Control (direct from this program, not through Arduino)
    def LambdaInit(self):
        self.LambdaWrite(238) # request serial port control from Lambda 10-2
    
    def SetLambdaCmd(self, filterselect, speed=5, wheel=0): # default is 470 nm, slow, wheel A
        return(filterselect + 16 * speed + 128*wheel)
        
    def LambdaShutter(self):
        v = int(self.ui.LambdaShutterCheckA.isChecked())
        if v == 1:
            self.LambdaWrite(self.Lambda_OpenShutterA)
        else:
            self.LambdaWrite(self.Lambda_CloseShutterA)

    def LambdaFilterA(self):
        v = int(self.ui.LambdaFilterSpinA.value())
        cmd = LambdaCmd(self, v)
        self.LambdaWrite(cmd)


app = QtGui.QApplication([])

win = QtGui.QMainWindow()
cw = RigWidget()
win.setCentralWidget(cw)
win.show()

app.exec_()

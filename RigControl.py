import serial
import sys, time, re, struct
from PySide import QtGui, QtCore
from PySide.QtUiTools import QUiLoader
import numpy
from scipy.optimize import leastsq
import pdb

class RigWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        super(RigWidget, self).__init__(parent)
        file = QtCore.QFile("DeviceController.ui")
        file.open(QtCore.QFile.ReadOnly)
        loader = QUiLoader()
        self.ui = loader.load(file, self)
        file.close()
        super(RigWidget, self).setWindowTitle('Rig Control Widget')
        self.ui.show()
        self.maplist = {'temp': 0, 'valves': 1, 'pump': 2, 'leds': 3, 'riglights': 4, 'timers': 5, 'lambda': 6}

        # connect to the Arduino (adjust port if needed...)
        self.ArduinoExists = False
        try:
            self.sp = serial.Serial('/dev/tty.usbserial-A800euOE', baudrate=115200, timeout=3)
            self.ArduinoExists = True
            self.ui.ArduinoConnectedText.setText('Arduino: Connected    ')
        except serial.SerialException:
            self.ui.ArduinoConnectedText.setText('Arduino: Not Connected')
        
        # get a timer for the timer/stopwatch function
        self.timer1 = QtCore.QTimer()
        self.timer2 = QtCore.QTimer()
        self.timer3 = QtCore.QTimer()
        self.connect(self.timer1, QtCore.SIGNAL("timeout()"), self.timer1_end);
        self.connect(self.timer2, QtCore.SIGNAL("timeout()"), self.timer2_end);
        self.connect(self.timer3, QtCore.SIGNAL("timeout()"), self.timer3_end);
        
        # connect to the Lambda 10/2 (change port if needed)
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
        
        # set up the Temperature Controller interface
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL('timeout()'), self.update)
        self.ui.addPointBtn.clicked.connect(self.addPoint)
        self.ui.delPointBtn.clicked.connect(self.delPoint)
        self.ui.sendCalBtn.clicked.connect(self.sendCalibration)
        self.ui.setTempCheck.clicked.connect(self.heaterCheckClicked)
        self.ui.targetSpin.valueChanged.connect(self.targetTempChanged)
        self.ui.maxTempSpin.valueChanged.connect(self.maxTempChanged)
        self.ui.delaySpin.valueChanged.connect(self.delayChanged)
#        self.ui.calPointList.itemChanged.connect(self.itemChanged)
        
        # Pump controller interface
        self.ui.pumpRunCheck.clicked.connect(self.pumpRun)
        self.ui.pumpSpeedSpin.valueChanged.connect(self.pumpSpeed)
        
        # Rig Lights interface
        self.ui.rigLightsOff.clicked.connect(self.rigLightsOff)
        self.ui.rigLightRed.clicked.connect(self.rigLightRed)
        self.ui.rigLightDimWhite.clicked.connect(self.rigLightDimWhite)
        self.ui.rigLightBrightWhite.clicked.connect(self.rigLightBrightWhite)

        # Valve Selection interface
        # provides manual selection of valves (or all valves off)
        # provides automated cleaning sequence
        self.ui.valveSelect0.clicked.connect(self.valveAllOff)
        self.ui.valveSelect1.clicked.connect(self.valveSelect1)
        self.ui.valveSelect2.clicked.connect(self.valveSelect2)
        self.ui.valveSelect3.clicked.connect(self.valveSelect3)
        self.ui.valveSelect4.clicked.connect(self.valveSelect4)
        self.ui.computerCheck.clicked.connect(self.valveComputerChecked)
        self.ui.valveCleaning.clicked.connect(self.valveCleaning)
        self.ui.tabWidget.setCurrentIndex(1) # start out by selecting the valve tab.
        
        # Timer interface
        # provides 3 timers that count down or up, and can execute one action at the end of the time
        
        # Lambda 10-2 (Sutter) Interface
        # provides selection of filter in wheel A and shutter open/close
        self.ui.LambdaShutterCheckA.clicked.connect(self.LambdaShutter)
        self.ui.LambdaFilterSpinA.currentIndexChanged.connect(self.LambdaFilterA)
        
        devices = self.getQuery() # find out which devices are enabled
        if devices is not None:
            firstDevice = []
            for device, v in devices.items():
                if v == 1:
                    self.ui.tabWidget.setTabEnabled(self.maplist[device], True)
                    self.ui.tabWidget.tabBar().setTabTextColor(self.maplist[device], 'white')
                    firstDevice.append(self.maplist[device])
                else:
                    self.ui.tabWidget.setTabEnabled(self.maplist[device], False)
                    self.ui.tabWidget.tabBar().setTabTextColor(self.maplist[device], 'grey')
            if firstDevice is not []:
                firstDevice = min(firstDevice)
                self.ui.tabWidget.setCurrentIndex(firstDevice)        
        
        self.timer.start(1000)
        self.pointList = []

# timers first:
    def timer1_end(self):
        pass

    def timer2_end(self):
        pass

    def timer3_end(self):
        pass

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

    def getQuery(self): # query of which devices are enabled in the Arduino code.
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
        self.rigLightsOff
        cmd = 'l11'
        self.ArduinoWrite(cmd)
        
    def rigLightDimWhite(self):
        self.rigLightsOff
        cmd = 'l21'
        self.ArduinoWrite(cmd)
        
    def rigLightBrightWhite(self):
        self.rigLightsOff
        cmd = 'l31'
        self.ArduinoWrite(cmd)

# LED Flourescence Lighting
    def ledAllOff(self):
        for i in range(1,4):
            cmd = 'L' + str(i) + '0'
            self.ArduinoWrite(cmd)

    def ledGreenSelect(self):
        self.ledAllOff
        cmd = 'L11'
        self.ArduinoWrite(cmd)

    def ledBlueSelect(self):
        self.ledAllOff
        cmd = 'L21'
        self.ArduinoWrite(cmd)

    def ledRedSelect(self):
        self.ledAllOff
        cmd = 'L31'
        self.ArduinoWrite(cmd)
       
# Valve controller

    def valveAllOff(self):
        #print "valve all off"
        cmd = 'V00'
        self.ArduinoWrite(cmd)
        self.ui.valveLabel.setText('Off')
        
    def valveSelect1(self):
        self.valveAllOff()
        cmd = 'V11'
        self.ArduinoWrite(cmd)
        self.ui.valveLabel.setText('1')

    def valveSelect2(self):
        self.valveAllOff()
        cmd = 'V21'
        self.ArduinoWrite(cmd)
        self.ui.valveLabel.setText('2')

    def valveSelect3(self):
        self.valveAllOff()
        cmd = 'V31'
        self.ArduinoWrite(cmd)
        self.ui.valveLabel.setText('3')

    def valveSelect4(self):
        self.valveAllOff()
        cmd = 'V41'
        self.ArduinoWrite(cmd) 
        self.ui.valveLabel.setText('4')

    def valveComputerChecked(self):
        pass

    def valveCleaning(self):
         pass

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

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    cw = RigWidget()
    cw.show()
    sys.exit(app.exec_())

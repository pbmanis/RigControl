import serial
import sys, time, re, struct
from PyQt6 import QtWidgets, uic, QtCore
import pyqtgraph as pg
import DeviceController as DC
import numpy
from scipy.optimize import leastsq

N_VALVES = 4
N_RIG_LIGHTS = 3
N_LEDS = 3


class RigWidget(QtWidgets.QMainWindow):
    def __init__(self):
        super(RigWidget, self).__init__()
        # pg.QtGui.QWidget.__init__(self)
        self.ui = uic.loadUi("DeviceController.ui", self)
        # uic.Ui_Form()
        # self.ui.setupUi(self)
        self.testmode: bool = True

        self.ArduinoExists = False
        try:
            self.sp = serial.Serial("/dev/cu.usbmodem2201", baudrate=115200, timeout=5)
            self.ArduinoExists = True
            self.ui.ArduinoConnectedText.setText("Arduino: Connected    ")
        except serial.SerialException:
            self.ui.ArduinoConnectedText.setText("Arduino: Not Connected")
            raise Exception("Arduino not connected")

        self.timer = pg.QtCore.QTimer()
        self.LambdaExists = False
        try:
            self.lambdasp = serial.Serial("/dev/ttyUSB2", baudrate=9600, timeout=3)
            self.LambdaExists = True
            self.LambdaInit()
            self.ui.LambdaConnectedText.setText("Lambda: Connected    ")
        except serial.SerialException:
            self.ui.LambdaConnectedText.setText("Lambda: Not Connected")
        self.Lambda_OpenShutterA = 170
        self.Lambda_CloseShutterA = 172
        self.Lambda_OpenShutterB = 186
        self.Lambda_CloseShutterB = 188
        self.LambdaFilter = 7  # initial filter
        self.LambdaSpeed = 5  # initial speed
        self.LambdaWheel = 0  # wheel A is 0, wheel B is 1 (we don't have a B wheel)
        self.Lambda_CMD = self.SetLambdaCmd(self.LambdaFilter, speed=self.LambdaSpeed, wheel=0)

        # Temperature
        self.timer.timeout.connect(self.update)
        # self.ui.addPointBtn.clicked.connect(self.addPoint)
        # self.ui.delPointBtn.clicked.connect(self.delPoint)
        self.ui.sendCalBtn.clicked.connect, (self.sendCalibration)
        self.ui.setTempCheck.clicked.connect(self.heaterCheckClicked)
        self.ui.targetSpin.valueChanged.connect(self.targetTempChanged)
        self.ui.maxTempSpin.valueChanged.connect(self.maxTempChanged)
        self.ui.delaySpin.valueChanged.connect(self.delayChanged)
        # self.ui.calPointList, QtCore.SIGNAL('itemChanged(QTreeWidgetItem *, int)'), self.itemChanged)

        # Recirculation Pump
        self.ui.pumpRunCheck.clicked.connect(self.pumpRun)
        self.ui.pumpSpeedSpin.valueChanged.connect(self.pumpSpeed)

        # Rig Lights
        # dictionary holds, for each action, the ui element and the action to take
        self.ui.RL = {
            "Off": {"ui": self.ui.rigLightsOff, "action": self.rigLightsOffAction},
            "Red": {"ui": self.ui.rigLightRed, "action": self.rigLightOnAction},
            "DimWhite": {"ui": self.ui.rigLightDimWhite, "action": self.rigLightOnAction},
            "BrightWhite": {
                "ui": self.ui.rigLightBrightWhite,
                "action": self.rigLightOnAction,
            },
        }
        for k, v in self.ui.RL.items():
            v["ui"].clicked.connect(v["action"])

        # Valve Selection
        self.ui.VALVE = {
            "ALL OFF": {"ui": self.ui.valveAllOff, "action": self.valveAllClosedAction},
            "Valve 1": {"ui": self.ui.valveSelect1, "action": self.valveOpenAction},
            "Valve 2": {"ui": self.ui.valveSelect2, "action": self.valveOpenAction},
            "Valve 3": {"ui": self.ui.valveSelect3, "action": self.valveOpenAction},
            "Valve 4": {"ui": self.ui.valveSelect4, "action": self.valveOpenAction},
        }
        for k, v in self.ui.VALVE.items():
            if k.startswith("Valve"):
                v["ui"].clicked.connect(v["action"])
            else:
                v["ui"].clicked.connect(v["action"])

        self.ui.computerCheck.clicked.connect(self.valveComputerChecked)
        #        self.ui.valveCleaning.clicked.connect(self.valveCleaning);

        # Rig LED Selection including brightness

        # Lambda 10-2
        self.ui.LambdaShutterCheckA.clicked.connect(self.LambdaShutter)
        self.ui.LambdaFilterSpinA.currentIndexChanged.connect(self.LambdaFilterA)

        devices = self.getQuery()  # find out which devices are enabled
        firstDevice = []
        print("Devices: ", devices)
        maplist = {
            "temp": 0,
            "valves": 1,
            "pump": 2,
            "leds": 3,
            "riglights": 4,
            "timers": 5,
            "lambda": 6,
        }
        for device, v in list(devices.items()):
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

        self.timer.start(100)
        self.pointList = []

    def ArduinoWrite(self, cmd):
        if self.ArduinoExists:
            self.sp.write(cmd)

    def ArduinoReadline(self):
        if self.ArduinoExists:
            return self.sp.readline()
        else:
            return None

    def LambdaWrite(self, cmd):
        if self.LambdaExists:
            self.lambdasp.write(cmd)

    def LambdaRead(self):
        if self.LambdaExists:
            return self.lambdasp.readline()
        else:
            return None

    def update(self):
        state = self.getState()
        if state is None:
            return
        self.ui.tempLabel.setText("%0.1f C" % state["t1"])
        self.ui.heaterLabel.setText("%0.1f C" % state["t2"])

        self.ui.targetSpin.blockSignals(True)
        self.ui.targetSpin.setValue(state["target"])
        self.ui.targetSpin.blockSignals(False)

        self.ui.maxTempSpin.blockSignals(True)
        self.ui.maxTempSpin.setValue(state["heatMax"])
        self.ui.maxTempSpin.blockSignals(False)

        self.ui.delaySpin.blockSignals(True)
        self.ui.delaySpin.setValue(state["delay"])
        self.ui.delaySpin.blockSignals(False)

        if state["enabled"] == 1:
            self.ui.heaterLevelLabel.setText(
                "%d%% (%d)" % (state["output"] * 100, state["outputPWM"])
            )
            self.ui.setTempCheck.setChecked(True)
        else:
            self.ui.heaterLevelLabel.setText("- off -")
            self.ui.setTempCheck.setChecked(False)

    def getQuery(self):
        if not self.ArduinoExists and not self.testmode:
            return None
        print("Checking devices")
        for i in range(3):
            s = self.sp.read(self.sp.inWaiting())
            print("s: ", s)
            if len(s) > 0:
                print(("::", s.strip()))
            self.sp.write(b"q")  # send query
            res = self.sp.readline()
            print("res: ", res)
            if res == "":
                print("Timed out getting state from controller.")
            try:
                (
                    temp_used,
                    valves_used,
                    pump_used,
                    leds_used,
                    riglights_used,
                    timers_used,
                    lambda_used,
                ) = res.split(b",")
                return {
                    "temp": int(temp_used),
                    "valves": int(valves_used),
                    "pump": int(pump_used),
                    "leds": int(leds_used),
                    "riglights": int(riglights_used),
                    "timers": int(timers_used),
                    "lambda": int(lambda_used),
                }
            except:
                pass
        return None

    def getState(self):  # temperature controller status
        if not self.ArduinoExists:
            return None
        for i in range(3):
            s = self.sp.read(self.sp.inWaiting())
            if len(s) > 0:
                print(("::", s.strip()))
            self.sp.write(b"?")  # send query
            res = self.sp.readline()
            if res == "":
                print("Timed out getting state from controller.")
            try:
                (t1, t2, v1, v2, target, hMax, delay, output, outputPWM, enable) = res.split(b",")
                return {
                    "t1": float(t1),
                    "t2": float(t2),
                    "v1": float(v1),
                    "v2": float(v2),
                    "target": float(target),
                    "heatMax": float(hMax),
                    "delay": float(delay),
                    "output": float(output),
                    "outputPWM": float(outputPWM),
                    "enabled": int(enable),
                }
            except:
                pass
        return None

    def heaterCheckClicked(self):
        v = int(self.ui.setTempCheck.isChecked())
        cmd = b"e" + chr(v)
        self.ArduinoWrite(cmd)
        # print self.sp.readline()

    def targetTempChanged(self):
        v = self.ui.targetSpin.value()
        # print "target changed:", v
        cmd = b"s" + struct.pack("f", v)
        self.ArduinoWrite(cmd)
        # print self.sp.readline()

    def maxTempChanged(self):
        v = self.ui.maxTempSpin.value()
        cmd = b"m" + struct.pack("f", v)
        self.ArduinoWrite(cmd)
        # print self.sp.readline()

    def delayChanged(self):
        v = self.ui.delaySpin.value()
        cmd = b"d" + struct.pack("f", v)
        self.ArduinoWrite(cmd)
        # print self.sp.readline()

    def addPoint(self):
        state = self.getState()
        item = QtGui.QTreeWidgetItem(["", "%0.2f" % state["v1"], "%0.2f" % state["v2"]])
        self.pointList.append((item, [0, state["v1"], state["v2"]]))
        item.setFlags(
            QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsEnabled
        )
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
        # item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)

    def updateSendBtn(self):
        self.ui.sendCalBtn.setEnabled(self.ui.calPointList.topLevelItemCount() >= 4)

    def sendCalibration(self):
        n = len(self.pointList)
        data = numpy.empty((3, n))
        for i in range(n):
            data[:, i] = self.pointList[i][1]

        print("Data:", data)

        ## find coefficients for function T = f(V)
        #     R = R1 * ((1023 / V) - 1)
        #     1/T = A + B log(R) + C log(R)^3   ## the parameter R1 gets absorbed by A, so there are only 3 variables.

        def fn(v, x):
            R = (1023.0 / x) - 1.0
            lnR = numpy.log(R)
            return 1.0 / (v[0] + v[1] * lnR + v[2] * lnR * lnR * lnR)

        def erf(v, x, y):
            return fn(v, x) - y

        guess = numpy.array([0.04, 0.03, -0.01])
        fit1 = leastsq(erf, guess, args=(data[1], data[0]))
        fit2 = leastsq(erf, guess, args=(data[2], data[0]))

        print("Fit:")
        print(fit1)
        print(fit2)

        cmd = (
            b"c"
            + fit1[0].astype(numpy.float32).tostring()
            + fit2[0].astype(numpy.float32).tostring()
        )
        self.ArduinoWrite(cmd)
        # print self.sp.readline()

   

    # Pump Controller
    def pumpRun(self):
        pass

    def pumpSpeed(self):
        pass

    # Rig Lighting

    def rigLightsOffAction(self):
        for i, k in enumerate(self.ui.RL.keys()):
            cmd = f"l{i:d}0".encode("utf8")
            print("rig lights command:", cmd)
            self.ArduinoWrite(cmd)
            self.ui.RL[k]["ui"].setChecked(False)

    def rigLightOnAction(self):
        for i, k in enumerate(self.ui.RL.keys()):
            if self.ui.RL[k]["ui"].isChecked():
                cmd = f"l{i:d}1".encode("utf8")
                print("rig lights command on:", cmd)
                self.ArduinoWrite(cmd)
            else:
                self.ui.RL[k]["ui"].setChecked(False)
                cmd = f"l{i:d}0".encode("utf8")
                print("rig lights command off:", cmd)
                self.ArduinoWrite(cmd)

    # LED Flourescence Lighting

    def ledAllOffAction(self):
        for i in range(1, 4):
            cmd = f"L{i:d}0".encode("utf")
            self.ArduinoWrite(cmd)

    def ledGreenSelectAction(self):
        self.rigLightsOffAction()
        cmd = b"L11"
        self.ArduinoWrite(cmd)

    def ledBlueSelectAction(self):
        self.rigLightsOffAction()
        cmd = b"L21"
        self.ArduinoWrite(cmd)

    def ledRedSelectAction(self):
        self.rigLightsOffAction()
        cmd = b"L31"
        self.ArduinoWrite(cmd)

    # Valves
 # Valve Controller

    def valveComputerChecked(self):
        pass

    def valveCleaning(self):
        pass

    def valveAllClosedAction(self):
        for i, k in enumerate(self.ui.VALVE.keys()):
            if k == "ALL OFF":
                continue
            cmd = f"V{i:d}0".encode("utf8")
            print("valve closing command:", k, cmd)
            self.ArduinoWrite(cmd)
            self.ui.VALVE[k]["ui"].setChecked(False)

    def valveOpenAction(self):
        for i, k in enumerate(self.ui.VALVE.keys()):
            if self.ui.VALVE[k]["ui"].isChecked():
                cmd = f"V{i:d}1".encode("utf8")
                print("valve open command:", cmd)
                self.ArduinoWrite(cmd)
            else:
                self.ui.VALVE[k]["ui"].setChecked(False)
                cmd = f"V{i:d}0".encode("utf8")
                print("valve close command:", cmd)
                self.ArduinoWrite(cmd)
            
    # def valveState(self, button):
    #     self.valveAllOffAction()
    #     match button.text():
    #         case "Valve 1":
    #             if button.isChecked():
    #                 self.valveSelect1Action()
    #         case "Valve 2":
    #             if button.isChecked():
    #                 self.valveSelect2Action()
    #         case "Valve 3":
    #             if button.isChecked():
    #                 self.valveSelect3Action()
    #         case "Valve 4":
    #             if button.isChecked():
    #                 self.valveSelect4Action()
    #         case _:
    #             pass

    # def valveAllOffAction(self):
    #     for i in range(1, 4):
    #         cmd = f"L{i:d}0".encode("utf")
    #         print("valve cmd: ", cmd)
    #         self.ArduinoWrite(cmd)
    #         self.ui.valveSelect1.setChecked(False)
    #         self.ui.valveSelect2.setChecked(False)
    #         self.ui.valveSelect3.setChecked(False)
    #         self.ui.valveSelect4.setChecked(False)

    # def valveSelect1Action(self):
    #     self.valveAllOffAction()
    #     cmd = b"V11"
    #     print("")
    #     self.ArduinoWrite(cmd)

    # def valveSelect2Action(self):
    #     self.valveAllOffAction()
    #     cmd = b"V21"
    #     self.ArduinoWrite(cmd)

    # def valveSelect3Action(self):
    #     self.valveAllOffAction()
    #     cmd = b"V31"
    #     self.ArduinoWrite(cmd)

    # def valveSelect4Action(self):
    #     self.valveAllOffAction()
    #     cmd = b"V41"
    #     self.ArduinoWrite(cmd)

    # Lambda 10-2 Control (direct from this program, not through Arduino)
    def LambdaInit(self):
        self.LambdaWrite(238)  # request serial port control from Lambda 10-2

    def SetLambdaCmd(self, filterselect, speed=5, wheel=0):  # default is 470 nm, slow, wheel A
        return filterselect + 16 * speed + 128 * wheel

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


app = QtWidgets.QApplication([])
win = pg.QtWidgets.QWidget()
cw = RigWidget()
# win.setCentralWidget(cw)
cw.show()

app.exec()

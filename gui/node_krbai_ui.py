#!/usr/bin/env python3
import os
import signal
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets
from krbai_ui import *
from robotic_sas_auv_ros.msg import Sensor, ArduinoSensor
import rospy

# Variabel global untuk menyimpan proses yang dijalankan
ros_process = None

# def customSetupCode():
#     global voltage
#     ui.pushButton_Start.clicked.connect(runRobot)
#     ui.pushButton_Stop.clicked.connect(stopRobot)

# def runRobot():
#     global ros_process
#     # Jalankan proses roslaunch dan simpan objek subprocess.Popen
#     ros_process = subprocess.Popen(["roslaunch", "rostu_sas", "all.launch"])
#     ui.lcdNumber_LoadVoltage.display(100)
#     print(f"Running with PID {ros_process.pid}")

# def stopRobot():
#     global ros_process
#     if ros_process:
#         # Hentikan proses dengan PID yang disimpan
#         os.kill(ros_process.pid, signal.SIGINT)
#         print(f"Stopped process with PID {ros_process.pid}")
#         ros_process = None
#     else:
#         print("No running process to stop")

def callback_Wheel(data: Sensor):
    global voltage
    voltage = data.loadvoltage
    ui.lcdNumber_LoadVoltage.display(data.loadvoltage)

if __name__ == "__main__":
    import sys
    rospy.init_node("krbai")
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    # customSetupCode()

    rospy.Subscriber("/rosserial/sensor", ArduinoSensor, lambda msg: ui.Sensor_label.setText(str(msg)), queue_size=10)
    rospy.Subscriber("/rosserial/sensor", ArduinoSensor, lambda msg: ui.lcdNumber_Voltage.display(msg.loadvoltage), queue_size=10)
    rospy.Subscriber("/rosserial/sensor", ArduinoSensor, lambda msg: ui.lcdNumber_Current.display(msg.current_mA), queue_size=10)
    rospy.Subscriber("/rosserial/sensor", ArduinoSensor, lambda msg: ui.lcdNumber_Depth.display(msg.depth), queue_size=10)
    # rospy.spin()

    Dialog.show()
    sys.exit(app.exec_())

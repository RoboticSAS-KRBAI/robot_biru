import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QImage
import cv2  # OpenCV untuk menangkap video
from sensor_ui import Ui_MainWindow  # File UI hasil pyuic5
from robotic_sas_auv_ros.msg import Sensor, ArduinoSensor


class MainWindow(QMainWindow):
    update_gui_signal = pyqtSignal(Sensor)
    update_arduino_signal = pyqtSignal(ArduinoSensor)

    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # ROS Initialization
        rospy.init_node('sensor_ui_node', anonymous=True)
        rospy.Subscriber("sensor_msg", Sensor, self.callback_sensor)
        rospy.Subscriber("arduino_sensor", ArduinoSensor, self.callback_arduino_sensor)

        # Connect signal to GUI update methods
        self.update_gui_signal.connect(self.update_gui)
        self.update_arduino_signal.connect(self.update_arduino)

        # Kamera Setup
        self.camera = cv2.VideoCapture(0)  # 0 untuk kamera default
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_camera)
        self.timer.start(30)  # 30ms interval untuk mendapatkan ~33 FPS

        # Label untuk menampilkan video di frame_camera
        self.camera_label = QLabel(self.ui.frame_camera)
        layout = QVBoxLayout(self.ui.frame_camera)
        layout.addWidget(self.camera_label)

    def callback_sensor(self, data: Sensor):
        self.update_gui_signal.emit(data)

    def callback_arduino_sensor(self, data: ArduinoSensor):
        self.update_arduino_signal.emit(data)

    def update_gui(self, data: Sensor):
        self.ui.label_pitch.setText(str(data.pitch))
        self.ui.label_roll.setText(str(data.roll))
        self.ui.label_yaw.setText(str(data.yaw))
        self.ui.label_depth.setText(str(data.depth))

    def update_arduino(self, data: ArduinoSensor):
        self.ui.label_busvoltage.setText(f"{data.busvoltage:.2f} V")
        self.ui.label_shuntvoltage.setText(f"{data.shuntvoltage:.2f} mV")
        self.ui.label_loadvoltage.setText(f"{data.loadvoltage:.2f} V")
        self.ui.label_current.setText(f"{data.current_mA:.2f} mA")
        self.ui.label_power.setText(f"{data.power_mW:.2f} mW")

    def update_camera(self):
        # Capture frame-by-frame
        ret, frame = self.camera.read()
        if ret:
            # Convert BGR (OpenCV format) to RGB (Qt format)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)

            # Set pixmap ke QLabel
            self.camera_label.setPixmap(pixmap)
            self.camera_label.setScaledContents(True)

    def closeEvent(self, event):
        # Release camera and shut down ROS
        self.camera.release()
        rospy.signal_shutdown("GUI closed")
        event.accept()


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

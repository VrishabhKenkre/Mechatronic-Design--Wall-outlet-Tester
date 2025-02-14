import sys
import re
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLCDNumber, QCheckBox, QSlider, QGroupBox
)
from PyQt5.QtCore import QTimer, Qt
import serial

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Motors Lab: Team D - Old Monks")
        self.resize(800, 400)

        # Create LCD displays for each sensor reading
        self.yaw_lcd = QLCDNumber()
        self.current_value_lcd = QLCDNumber()
        self.count_lcd = QLCDNumber()

        # Create labels for sensor readings
        yaw_label = QLabel("IMU Yaw (in degree)")
        current_value_label = QLabel("Potentiometer")
        count_label = QLabel("Rotary Encoder (in degree)")

        # Layout for sensor readouts
        sensor_layout = QHBoxLayout()
        sensor_layout.addLayout(self._create_display_group(yaw_label, self.yaw_lcd))
        sensor_layout.addLayout(self._create_display_group(current_value_label, self.current_value_lcd))
        sensor_layout.addLayout(self._create_display_group(count_label, self.count_lcd))

        # Create a checkbox for control mode
        self.control_checkbox = QCheckBox("Control by GUI")
        self.control_checkbox.toggled.connect(self.controlToggled)

        # Create sliders for control values
        self.pot_slider = QSlider(Qt.Horizontal)
        self.pot_slider.setRange(0, 1023)
        self.pot_slider.setValue(512)
        self.pot_slider.valueChanged.connect(self.sendControlValues)

        self.encoder_slider = QSlider(Qt.Horizontal)
        self.encoder_slider.setRange(0, 180)
        self.encoder_slider.setValue(90)
        self.encoder_slider.valueChanged.connect(self.sendControlValues)

        self.imu_slider = QSlider(Qt.Horizontal)
        self.imu_slider.setRange(-90, 90)
        self.imu_slider.setValue(0)
        self.imu_slider.valueChanged.connect(self.sendControlValues)

        # Labels for the sliders
        pot_label = QLabel("Pot")
        encoder_label = QLabel("Encoder")
        imu_label = QLabel("IMU")

        # Layout for the control group (checkbox + sliders)
        control_layout = QVBoxLayout()
        control_layout.addWidget(self.control_checkbox)
        control_layout.addLayout(self._create_slider_group(pot_label, self.pot_slider))
        control_layout.addLayout(self._create_slider_group(encoder_label, self.encoder_slider))
        control_layout.addLayout(self._create_slider_group(imu_label, self.imu_slider))

        # Group box for controls (optional for visual grouping)
        control_group = QGroupBox("GUI Control Panel")
        control_group.setLayout(control_layout)

        # Main layout: sensor readouts on top, control panel below
        main_layout = QVBoxLayout()
        main_layout.addLayout(sensor_layout)
        main_layout.addWidget(control_group)

        # Set the central widget
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Initialize serial connection on COM14 at 9600 baud.
        try:
            self.ser = serial.Serial('COM14', 9600, timeout=1)
            print("Serial port opened successfully.")
        except Exception as e:
            print("Error opening serial port:", e)
            self.ser = None

        # Set up a timer to read serial data every 100 ms.
        self.timer = QTimer()
        self.timer.setInterval(100)  # 100 ms
        self.timer.timeout.connect(self.readSerial)
        self.timer.start()

        # Initially disable sliders if control mode is off.
        self.setControlWidgetsEnabled(False)

    def _create_display_group(self, label_widget, lcd_widget):
        """Helper to create a vertical layout with a label and LCD display."""
        layout = QVBoxLayout()
        layout.addWidget(label_widget)
        layout.addWidget(lcd_widget)
        return layout

    def _create_slider_group(self, label_widget, slider_widget):
        """Helper to create a vertical layout with a label and slider."""
        layout = QVBoxLayout()
        layout.addWidget(label_widget)
        layout.addWidget(slider_widget)
        return layout

    def controlToggled(self, checked):
        """
        Called when the control checkbox is toggled.
        Enables/disables the sliders and (optionally) sends a command to the Arduino.
        """
        print("Control mode:", checked)
        self.setControlWidgetsEnabled(checked)
        if checked:
            # When enabling control, send the current slider values.
            self.sendControlValues()
        else:
            # Optionally, send a command indicating manual control is off.
            self.sendControlDisable()

    def setControlWidgetsEnabled(self, enable):
        """Enable or disable the slider widgets."""
        self.pot_slider.setEnabled(enable)
        self.encoder_slider.setEnabled(enable)
        self.imu_slider.setEnabled(enable)

    def sendControlValues(self):
        """
        If control mode is enabled, send the current slider values to the Arduino.
        The format is an example. Adjust it as needed for your Arduino code.
        """
        if self.control_checkbox.isChecked() and self.ser:
            pot_val = self.pot_slider.value()
            encoder_val = self.encoder_slider.value() 
            imu_val = self.imu_slider.value()
            # Create a command string. For example:
            command = f"P:{pot_val},E:{encoder_val},I:{imu_val}\n"
            try:
                self.ser.write(command.encode('utf-8'))
                print("Sent control command:", command.strip())
            except Exception as e:
                print("Error sending control command:", e)

    def sendControlDisable(self):
        """
        Optional: Send a command indicating that GUI control is disabled.
        """
        if self.ser:
            try:
                self.ser.write("CONTROL:OFF\n".encode('utf-8'))
                print("Sent control disable command")
            except Exception as e:
                print("Error sending control disable command:", e)  

    def readSerial(self):
        """
        Reads a line from the serial port.
        Expects a line with the format:
            current_yaw: <value>, currentValue: <value>, count: <value>
        Splits the line and updates the three LCD displays.
        """
        if self.ser and self.ser.in_waiting:
            try:
                # Read one line from the serial port
                line = self.ser.readline().decode('utf-8').strip()
                print("Received:", line)

                # Use regular expressions to extract values.
                # The regex looks for: <key>: <number>
                # For example: current_yaw: 12.34
                pattern = r'current_yaw:\s*([-+]?[0-9]*\.?[0-9]+).*currentValue:\s*([-+]?[0-9]*\.?[0-9]+).*count:\s*(\d+)'
                match = re.search(pattern, line)
                if match:
                    yaw_val = float(match.group(1))
                    current_val = float(match.group(2))
                    count_val = int(match.group(3))

                    self.yaw_lcd.display(yaw_val)
                    self.current_value_lcd.display(current_val)
                    self.count_lcd.display(count_val)
            except Exception as e:
                print("Error processing serial data:", e)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
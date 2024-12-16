#!/usr/bin/env python3
# encoding: utf-8

import sys
import yaml
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, QFileDialog
from PyQt5.QtCore import Qt
import ros_robot_controller_sdk as rrc

# 创建全局的 Board 实例(create global Board instance)
board = rrc.Board()

class MainWindow(QWidget):
    def __init__(self, board):
        super(MainWindow, self).__init__()
        self.board = board  # 使用传入的 Board 实例(use the passed Board instance)
        self.language = "EN"
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Servo Offset Control')
        layout = QVBoxLayout()

        self.sliders = {}
        self.labels = {}
        self.servo_values = {}

        for i in range(1, 5):
            label = QLabel(f'Servo {i} Offset:', self)
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(-128)
            slider.setMaximum(127)
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, idx=i: self.sliderChanged(value, idx))

            self.labels[i] = label
            self.sliders[i] = slider
            self.servo_values[i] = 0

            layout.addWidget(label)
            layout.addWidget(slider)

        self.center_button = QPushButton('Set Center Position', self)
        self.center_button.clicked.connect(self.setCenterPosition)
        layout.addWidget(self.center_button)

        self.save_button = QPushButton('Save Offsets', self)
        self.save_button.clicked.connect(self.saveOffsets)
        layout.addWidget(self.save_button)

        self.load_button = QPushButton('Load Offsets', self)
        self.load_button.clicked.connect(self.loadOffsets)
        layout.addWidget(self.load_button)

        self.language_button = QPushButton('Switch to Chinese', self)
        self.language_button.clicked.connect(self.toggleLanguage)
        layout.addWidget(self.language_button)

        self.setLayout(layout)
        self.setGeometry(300, 300, 350, 250)

        self.updateTexts()

    def sliderChanged(self, value, idx):
        self.servo_values[idx] = value
        self.board.pwm_servo_set_offset(idx, value)
        self.labels[idx].setText(f'{self.getLabelText()} {idx} {self.getOffsetText()}: {value}')

    def setCenterPosition(self):
        for i in range(1, 5):
            self.sliders[i].setValue(0)
            self.sliderChanged(0, i)

    def saveOffsets(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Offsets", "", "YAML Files (*.yaml);;All Files (*)", options=options)
        if file_name:
            with open(file_name, 'w') as file:
                yaml.dump(self.servo_values, file)

    def loadOffsets(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Load Offsets", "", "YAML Files (*.yaml);;All Files (*)", options=options)
        if file_name:
            with open(file_name, 'r') as file:
                self.servo_values = yaml.safe_load(file) or {}
                for i in range(1, 5):
                    self.sliders[i].setValue(self.servo_values.get(i, 0))
                    self.sliderChanged(self.servo_values.get(i, 0), i)

    def toggleLanguage(self):
        self.language = "CN" if self.language == "EN" else "EN"
        self.updateTexts()

    def updateTexts(self):
        if self.language == "EN":
            self.setWindowTitle('Servo Offset Control')
            self.center_button.setText('Set Center Position')
            self.save_button.setText('Save Offsets')
            self.load_button.setText('Load Offsets')
            self.language_button.setText('language')
            for i in range(1, 5):
                self.labels[i].setText(f'Servo {i} Offset: {self.servo_values.get(i, 0)}')
        else:
            self.setWindowTitle('舵机偏差控制')
            self.center_button.setText('设为中间位置')
            self.save_button.setText('保存偏差')
            self.load_button.setText('加载偏差')
            self.language_button.setText('切换到英文')
            for i in range(1, 5):
                self.labels[i].setText(f'舵机 {i} 偏差: {self.servo_values.get(i, 0)}')

    def getLabelText(self):
        return 'Servo' if self.language == "EN" else '舵机'

    def getOffsetText(self):
        return 'Offset' if self.language == "EN" else '偏差'

    def closeEvent(self, event):
        self.board.port.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow(board)  # 将全局 Board 实例传递给 MainWindow(pass the global Board instance to MainWindow)
    window.show()
    sys.exit(app.exec_())


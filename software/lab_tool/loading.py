#!/usr/bin/env python3
# encoding: utf-8
# Data:2022/09/12
import time, os
from PyQt5 import QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel, QDesktopWidget, QApplication, QDialog

class LoadingWindow(QDialog):
    def __init__(self):
        super(LoadingWindow, self).__init__()
        self.initUI()
        self.main_window = None

    def set_window_center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())   

    def initUI(self):
        self.resize(640, 320)
        self.set_window_center()
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog | Qt.WindowStaysOnTopHint)

        self.setAttribute(Qt.WA_TranslucentBackground)
        gif_path = os.path.dirname(__file__)
        if gif_path == '':
            gif_path = '.'
        self.loading_gif = QtGui.QMovie(gif_path + '/resource/loading.gif')
        self.loading_label = QLabel(self)
        self.loading_label.setMovie(self.loading_gif)
        self.loading_gif.finished.connect(self.closed)
        self.loading_gif.start()
    
    def closed(self):
        self.loading_gif.stop()
        self.accept()
        #time.sleep(0.1)
        self.main_window.show()

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QDialog

import message
import message_tips

class Message():
    def __init__(self):
        self.message_info = None
        self.message_close = False
        self.message_timer = QTimer(timeout=self.message_create)

    def message_create(self):
        if self.message_close:
            self.message_close = False
            self.message_timer.stop()
            self.qdi.accept()
        else:
            self.qdi = QDialog()
            self.d = message_tips.Ui_Dialog()
            self.qdi.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)

            self.d.setupUi(self.qdi)
            self.d.label.setText(self.message_info)
            self.qdi.show()
            self.message_close = True
            self.message_timer.stop()
            self.message_timer.start(1000)
    
    def tips(self, msg):
        self.message_info = msg
        self.message_timer.start(50)

    def info(self, msg):
        self.qdi = QDialog()
        self.d = message.Ui_Dialog()
        self.qdi.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)

        self.d.setupUi(self.qdi)
        self.qdi.show()
        self.d.label.setText(msg)
        self.d.pushButton_ok.pressed.connect(self.closeqdialog)

    def closeqdialog(self):
        self.qdi.accept()

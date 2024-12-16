#!/usr/bin/env python3
# encoding: utf-8
# Date:2022/10/20
# Author:aiden
import os
import re
import sys
import distro
import psutil
import platform
import importlib
from PyQt5.QtCore import Qt, QThread
from PyQt5.QtWidgets import QWidget, QApplication, QMessageBox, QDesktopWidget
if __name__ == '__main__':
    import ui
    import message_main
else:
    from tool import ui
    from tool import message_main

class ResetServiceThread(QThread):
    def __init__(self):
        super(ResetServiceThread,self).__init__()
    
    def run(self):
        os.system('sudo systemctl restart start_app_node.service')

HW_WIFI_AP_SSID = ""
class MainWindow(QWidget, ui.Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.set_window_position()
        self.config_file_name = "hiwonder_wifi_conf.py"
        self.external_config_file_dir_path = '/home/ubuntu/wifi_manager'
        self.external_config_file_path = os.path.join(self.external_config_file_dir_path, self.config_file_name)
        self.haved_save = False
        self.reset_service_thread = ResetServiceThread()
        self.message = message_main.Message()
        
        self.pushButton_apply.pressed.connect(lambda: self.button_clicked('apply'))
        self.pushButton_save.pressed.connect(lambda: self.button_clicked('save'))
        self.pushButton_exit.pressed.connect(lambda: self.button_clicked('exit'))
        self.pushButton_apply.setToolTip("restart start_app_node.service")

        self.radioButton_zn.toggled.connect(lambda: self.change_language(self.radioButton_zn))
        self.radioButton_en.toggled.connect(lambda: self.change_language(self.radioButton_en))        
        self.chinese = True

        try:
            if os.environ['ASR_LANGUAGE'] == 'English':
                self.chinese = False
                self.radioButton_en.setChecked(True)
            else:               
                self.radioButton_zn.setChecked(True)
        except:
            self.radioButton_zn.setChecked(True)	
        
        self.path = os.path.split(os.path.realpath(__file__))[0]
        self.typerc_path = os.path.join(self.path, "../../ros2_ws/.typerc")
        self.language = None
        self.depth_camera = None
        self.lidar = None
        self.machine = None
        self.version = None
        self.get_typerc()

        depth_camera_list = ['ascamera', 'usb_cam']
        lidar_list = ['LD19']
        machine_list = ['JetRover_Mecanum',  'JetRover_Acker','ROSMentor_Mecanum','MentorPi_Acker']
        language_list = ['Chinese', 'English']
        self.comboBox_depth_camera.addItems(depth_camera_list)
        self.comboBox_lidar.addItems(lidar_list)
        self.comboBox_machine.addItems(machine_list)
        #self.comboBox_asr.addItems(language_list)
        self.comboBox_depth_camera.setCurrentIndex(depth_camera_list.index(self.depth_camera))
        self.comboBox_lidar.setCurrentIndex(lidar_list.index(self.lidar))
        self.comboBox_machine.setCurrentIndex(machine_list.index(self.machine))
        #self.comboBox_asr.setCurrentIndex(language_list.index(self.language))

        self.lineEdit_version.setFocusPolicy(Qt.NoFocus)
        
        self.label_platform.setText(self.get_platform())
        self.label_kernel.setText(self.get_kernel())
        self.label_disk.setText(self.get_disk_space())
        self.label_memory.setText(self.get_memory())
        self.label_wlan.setText(self.get_wlan())
        self.label_eth.setText(self.get_eth())
        self.lineEdit_wifi.setText(self.get_hw())
        self.lineEdit_version.setText(self.version)
        
    def set_window_position(self):
        # 窗口居中(put the window in the middle)
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def change_language(self, name):
        if self.radioButton_zn.isChecked() and name.text() == '中文':
            self.chinese = True
            self.label_depth_camera.setText('深度摄像头')
            self.label_lidar.setText('雷达')
            self.label_machine.setText('机器类型')
            #self.label_asr.setText('语音功能')
            self.label_version.setText('版本')
            self.label_platform1.setText('操作系统')
            self.label_kernel1.setText('内核版本')
            self.label_disk1.setText('磁盘容量')
            self.label_memory1.setText('内存占用')
            self.label_wlan1.setText('无线IP地址')
            self.label_eth1.setText('有线IP地址')
            self.label_wifi.setText('热点名称')
            self.pushButton_save.setText('保存')
            self.pushButton_apply.setText('生效')
            self.pushButton_exit.setText('退出')
            self.label_wlan.setText(self.get_wlan())
            self.label_eth.setText(self.get_eth())
        elif self.radioButton_en.isChecked() and name.text() == 'English':
            self.chinese = False
            self.label_depth_camera.setText('Depth Camera')
            self.label_lidar.setText('Lidar')
            self.label_machine.setText('Machine')
            #self.label_asr.setText('ASR')
            self.label_version.setText('Version')
            self.label_platform1.setText('Operating System')
            self.label_kernel1.setText('Kernel Version')
            self.label_disk1.setText('Disk Capacity')
            self.label_memory1.setText('Memory')
            self.label_wlan1.setText('WLAN')
            self.label_eth1.setText('Ethemet')
            self.label_wifi.setText('AP Name')
            self.pushButton_save.setText('Save')
            self.pushButton_apply.setText('Apply')
            self.pushButton_exit.setText('Quit')
            self.label_wlan.setText(self.get_wlan())
            self.label_eth.setText(self.get_eth())

    # 弹窗提示函数(window prompt window)
    def message_from(self, string):
        try:
            QMessageBox.about(self, '', string)
        except:
            pass
    
    # 窗口退出(exit the window)
    def closeEvent(self, e):    
        result = QMessageBox.question(self,
                                    "Prompt box",
                                    "quit?",
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.No)
        if result == QMessageBox.Yes:
            # 退出前先把节点退出(exit the node before closing the program)
            QWidget.closeEvent(self, e)
        else:
            e.ignore()
   
    def update_globals(self, module):
        if module in sys.modules:
            mdl = importlib.reload(sys.modules[module])
        else:
            mdl = importlib.import_module(module)
        if "__all" in mdl.__dict__:
            names = mdl.__dict__["__all__"]
        else:
            names = [x for x in mdl.__dict__ if not x.startswith("_")]
        globals().update({k: getattr(mdl, k) for k in names})

    def get_hw(self):
        global HW_WIFI_AP_SSID

        # address = "/sys/class/net/eth0/address"
        # if os.path.exists(address):
            # with open(address, 'r') as f:
                # serial_num = f.read().replace('\n', '').replace(':', '').upper()
                # serial_num = serial_num[-8:]
        # else:
        device_serial_number = open("/proc/device-tree/serial-number")
        serial_num = device_serial_number.readlines()[0][-10:-1]

        HW_WIFI_AP_SSID = ''.join(["HW-", serial_num[0:8]])
        if os.path.exists(self.external_config_file_path):
            sys.path.insert(0, self.external_config_file_dir_path)
            self.update_globals(os.path.splitext(self.config_file_name)[0])
        
        return HW_WIFI_AP_SSID

    def get_platform(self):

        with open("/etc/os-release") as f:
            content = f.readlines()

        content = [x.strip().split('=') for x in content]
        os_info = {x[0]:x[1].strip('\"') for x in content if len(x)==2}
        return '{} {}+ ROS2 {}'.format(os_info['NAME'], os_info['VERSION'].replace('(', ' ').replace(')', ' '), 'Humble')
    
    def get_kernel(self):
        result = platform.uname()
        return '{}_{}_{}'.format(result.system, result.release, result.machine)

    def get_memory(self):
        mem = psutil.virtual_memory()
        mem_total = round(mem.total / 1024 / 1024 / 1024, 2)
        mem_free = round(mem.available / 1024 / 1024 / 1024, 2)

        return 'Total:{}G  Free:{}G'.format(mem_total, mem_free)

    def get_disk_space(self):
        disk = psutil.disk_usage('/')
        disk_total = round(disk.total / 1024/ 1024 / 1024, 2)
        disk_free = round(disk.free / 1024 / 1024 / 1024, 2)

        return 'Total:{}G  Free:{}G'.format(disk_total, disk_free)
    
    def get_wlan(self):
        ip = ''
        info = psutil.net_if_addrs()
        for k, v in info.items():
            if 'wlan0' in k:
                for i in v:
                    if i[2] is not None:
                        ip = i[1]
                        break
                    else:
                        ip = None

        if ip != '' and ip is not None:
            return ip
        elif ip is None:
            if self.chinese:
                return '未连接无线网络'
            else:
                return 'Not connected to wireless network' 
        else:
            if self.chinese:
                return '未检测到无线网卡'
            else:
                return 'Wireless card not detected'

    def get_eth(self):
        ip = ''
        info = psutil.net_if_addrs()
        for k, v in info.items():
            if 'eth' in k:
                for j in v:
                    if j[1] != '127.0.0.1' and j[2] is not None:
                        ip = j[1]
                        break
                if ip != '':
                    break
        if ip != '':
            return ip
        else:
            if self.chinese:
                return '未连接有线网络'
            else:
                return 'Wired network not connected'

    def get_typerc(self):
        with open(self.typerc_path, "r") as f:
            data = f.read()
            self.version = re.findall(r'export VERSION.*?\n', data)[0].split('=')[1].replace('\n', '').split('|')
            self.version = self.version[1] + '  ' + self.version[3]
            self.language = re.findall(r'export ASR_LANGUAGE.*?\n', data)[0].split('=')[1].replace('\n', '')
            self.lidar = re.findall(r'export LIDAR_TYPE.*?\n', data)[0].split('=')[1].replace('\n', '')
            
            self.depth_camera = re.findall(r'export DEPTH_CAMERA_TYPE.*?\n', data)[0].split('=')[1].replace('\n', '')
            
            self.machine = re.findall(r'export MACHINE_TYPE.*?\n', data)[0].split('=')[1].replace('\n', '')
            f.close()
    
    def set_typerc(self):
        with open(self.typerc_path, "r") as f:
            data = f.read()
            
            depth_camera = self.comboBox_depth_camera.currentText()
            data = data.replace("=" + self.depth_camera, "=" + depth_camera)

            lidar = self.comboBox_lidar.currentText()
            data = data.replace("=" + self.lidar, "=" + lidar)

            machine = self.comboBox_machine.currentText()
            data = data.replace("=" + self.machine, "=" + machine)

            #language = self.comboBox_asr.currentText()
            #data = data.replace("=" + self.language, "=" + language)

            f.close()
        with open(self.typerc_path, "w") as f:
            f.write(data)
            f.close()

    def button_clicked(self, name):
        if name == 'save':
            self.set_typerc()
            self.haved_save = True
            if self.chinese:
                self.message.tips("保存成功", 500)
            else:
                self.message.tips("Save Success", 500)
        elif name == 'apply':
            if not self.haved_save:
                self.set_typerc()
                self.haved_save = False 
            self.reset_service_thread.start()
            if self.chinese:
                self.message.tips("正在重启服务...", 15000)
            else:
                self.message.tips("Restart Service...", 15000)
        elif name == 'exit':
            sys.exit(0)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myshow = MainWindow()
    myshow.show()
    sys.exit(app.exec_())

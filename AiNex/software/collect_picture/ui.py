# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(700, 650)
        Form.setMinimumSize(QtCore.QSize(700, 650))
        Form.setMaximumSize(QtCore.QSize(700, 650))
        Form.setSizeIncrement(QtCore.QSize(700, 650))
        self.widget = QtWidgets.QWidget(Form)
        self.widget.setGeometry(QtCore.QRect(0, 0, 700, 700))
        self.widget.setMinimumSize(QtCore.QSize(700, 700))
        self.widget.setMaximumSize(QtCore.QSize(700, 700))
        self.widget.setStyleSheet("QWidget#widget {\n"
"background-color: #EBEBEB;\n"
"}\n"
"")
        self.widget.setObjectName("widget")
        self.label_display = QtWidgets.QLabel(self.widget)
        self.label_display.setGeometry(QtCore.QRect(30, 50, 640, 480))
        self.label_display.setMinimumSize(QtCore.QSize(640, 480))
        self.label_display.setMaximumSize(QtCore.QSize(640, 480))
        self.label_display.setSizeIncrement(QtCore.QSize(0, 0))
        self.label_display.setStyleSheet("background-color: rgb(50, 50, 60);")
        self.label_display.setText("")
        self.label_display.setObjectName("label_display")
        self.pushButton_exit = QtWidgets.QPushButton(self.widget)
        self.pushButton_exit.setGeometry(QtCore.QRect(600, 560, 70, 71))
        self.pushButton_exit.setStyleSheet("QPushButton{\n"
"background-color: #A2A2A2;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_exit.setObjectName("pushButton_exit")
        self.label_camera = QtWidgets.QLabel(self.widget)
        self.label_camera.setGeometry(QtCore.QRect(290, 20, 91, 17))
        self.label_camera.setAlignment(QtCore.Qt.AlignCenter)
        self.label_camera.setObjectName("label_camera")
        self.pushButton_save = QtWidgets.QPushButton(self.widget)
        self.pushButton_save.setGeometry(QtCore.QRect(360, 560, 70, 71))
        self.pushButton_save.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_save.setObjectName("pushButton_save")
        self.pushButton_delete = QtWidgets.QPushButton(self.widget)
        self.pushButton_delete.setGeometry(QtCore.QRect(440, 560, 70, 71))
        self.pushButton_delete.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_delete.setObjectName("pushButton_delete")
        self.layoutWidget = QtWidgets.QWidget(self.widget)
        self.layoutWidget.setGeometry(QtCore.QRect(30, 560, 321, 70))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_resolution = QtWidgets.QLabel(self.layoutWidget)
        self.label_resolution.setAlignment(QtCore.Qt.AlignCenter)
        self.label_resolution.setObjectName("label_resolution")
        self.horizontalLayout_3.addWidget(self.label_resolution)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_width = QtWidgets.QLabel(self.layoutWidget)
        self.label_width.setAlignment(QtCore.Qt.AlignCenter)
        self.label_width.setObjectName("label_width")
        self.horizontalLayout.addWidget(self.label_width)
        self.lineEdit_width = QtWidgets.QLineEdit(self.layoutWidget)
        self.lineEdit_width.setMaximumSize(QtCore.QSize(50, 16777215))
        self.lineEdit_width.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_width.setObjectName("lineEdit_width")
        self.horizontalLayout.addWidget(self.lineEdit_width)
        self.horizontalLayout_3.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_height = QtWidgets.QLabel(self.layoutWidget)
        self.label_height.setAlignment(QtCore.Qt.AlignCenter)
        self.label_height.setObjectName("label_height")
        self.horizontalLayout_2.addWidget(self.label_height)
        self.lineEdit_height = QtWidgets.QLineEdit(self.layoutWidget)
        self.lineEdit_height.setMaximumSize(QtCore.QSize(50, 16777215))
        self.lineEdit_height.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_height.setObjectName("lineEdit_height")
        self.horizontalLayout_2.addWidget(self.lineEdit_height)
        self.horizontalLayout_3.addLayout(self.horizontalLayout_2)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_save_path = QtWidgets.QLabel(self.layoutWidget)
        self.label_save_path.setAlignment(QtCore.Qt.AlignCenter)
        self.label_save_path.setObjectName("label_save_path")
        self.horizontalLayout_4.addWidget(self.label_save_path)
        self.lineEdit_path = QtWidgets.QLineEdit(self.layoutWidget)
        self.lineEdit_path.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.lineEdit_path.setObjectName("lineEdit_path")
        self.horizontalLayout_4.addWidget(self.lineEdit_path)
        self.pushButton_select = QtWidgets.QPushButton(self.layoutWidget)
        self.pushButton_select.setMinimumSize(QtCore.QSize(50, 0))
        self.pushButton_select.setMaximumSize(QtCore.QSize(50, 16777215))
        self.pushButton_select.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:1px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_select.setObjectName("pushButton_select")
        self.horizontalLayout_4.addWidget(self.pushButton_select)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.radioButton_zn = QtWidgets.QRadioButton(self.widget)
        self.radioButton_zn.setGeometry(QtCore.QRect(30, 10, 61, 27))
        self.radioButton_zn.setObjectName("radioButton_zn")
        self.radioButton_en = QtWidgets.QRadioButton(self.widget)
        self.radioButton_en.setGeometry(QtCore.QRect(100, 10, 81, 27))
        self.radioButton_en.setObjectName("radioButton_en")
        self.pushButton_camera_change = QtWidgets.QPushButton(self.widget)
        self.pushButton_camera_change.setGeometry(QtCore.QRect(520, 560, 70, 71))
        self.pushButton_camera_change.setStyleSheet("QPushButton{\n"
"background-color: #A2A2A2;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_camera_change.setObjectName("pushButton_camera_change")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "collection 1.0"))
        self.pushButton_exit.setText(_translate("Form", "退出"))
        self.label_camera.setText(_translate("Form", "摄像头画面"))
        self.pushButton_save.setText(_translate("Form", "保存\n"
"(space)"))
        self.pushButton_delete.setText(_translate("Form", "删除\n"
"(d)"))
        self.label_resolution.setText(_translate("Form", "图片分辨率："))
        self.label_width.setText(_translate("Form", "宽"))
        self.lineEdit_width.setText(_translate("Form", "640"))
        self.label_height.setText(_translate("Form", "高"))
        self.lineEdit_height.setText(_translate("Form", "480"))
        self.label_save_path.setText(_translate("Form", "保存路径："))
        self.pushButton_select.setText(_translate("Form", "选择"))
        self.radioButton_zn.setText(_translate("Form", "中文"))
        self.radioButton_en.setText(_translate("Form", "English"))
        self.pushButton_camera_change.setText(_translate("Form", "深度相机"))


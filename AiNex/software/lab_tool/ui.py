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
        Form.setWindowModality(QtCore.Qt.NonModal)
        Form.setEnabled(True)
        Form.resize(1340, 760)
        Form.setMinimumSize(QtCore.QSize(0, 0))
        Form.setMaximumSize(QtCore.QSize(1370, 760))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(11)
        Form.setFont(font)
        Form.setFocusPolicy(QtCore.Qt.NoFocus)
        Form.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        Form.setAcceptDrops(False)
        Form.setAutoFillBackground(False)
        Form.setStyleSheet("background-color: #EBEBEB;")
        self.widget_12 = QtWidgets.QWidget(Form)
        self.widget_12.setGeometry(QtCore.QRect(10, 500, 701, 140))
        self.widget_12.setStyleSheet("QWidget {\n"
"background-color: rgb(255, 255, 255);\n"
"border-radius:6px;}")
        self.widget_12.setObjectName("widget_12")
        self.label_25 = QtWidgets.QLabel(self.widget_12)
        self.label_25.setGeometry(QtCore.QRect(10, 5, 15, 31))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label_25.setFont(font)
        self.label_25.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_25.setAlignment(QtCore.Qt.AlignCenter)
        self.label_25.setObjectName("label_25")
        self.label_26 = QtWidgets.QLabel(self.widget_12)
        self.label_26.setGeometry(QtCore.QRect(10, 55, 15, 31))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label_26.setFont(font)
        self.label_26.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_26.setAlignment(QtCore.Qt.AlignCenter)
        self.label_26.setObjectName("label_26")
        self.label_27 = QtWidgets.QLabel(self.widget_12)
        self.label_27.setGeometry(QtCore.QRect(10, 110, 15, 21))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label_27.setFont(font)
        self.label_27.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_27.setAlignment(QtCore.Qt.AlignCenter)
        self.label_27.setObjectName("label_27")
        self.horizontalSlider1 = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider1.setGeometry(QtCore.QRect(80, 5, 180, 30))
        self.horizontalSlider1.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider1.setMaximum(255)
        self.horizontalSlider1.setPageStep(1)
        self.horizontalSlider1.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider1.setObjectName("horizontalSlider1")
        self.horizontalSlider3 = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider3.setGeometry(QtCore.QRect(80, 55, 180, 30))
        self.horizontalSlider3.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider3.setMaximum(255)
        self.horizontalSlider3.setPageStep(1)
        self.horizontalSlider3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider3.setObjectName("horizontalSlider3")
        self.horizontalSlider5 = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider5.setGeometry(QtCore.QRect(80, 105, 180, 30))
        self.horizontalSlider5.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider5.setMaximum(255)
        self.horizontalSlider5.setPageStep(1)
        self.horizontalSlider5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider5.setObjectName("horizontalSlider5")
        self.label1 = QtWidgets.QLabel(self.widget_12)
        self.label1.setGeometry(QtCore.QRect(310, 10, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label1.setFont(font)
        self.label1.setStyleSheet("color: rgb(0, 0, 0);")
        self.label1.setAlignment(QtCore.Qt.AlignCenter)
        self.label1.setObjectName("label1")
        self.label3 = QtWidgets.QLabel(self.widget_12)
        self.label3.setGeometry(QtCore.QRect(310, 60, 51, 21))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label3.setFont(font)
        self.label3.setStyleSheet("color: rgb(0, 0, 0);")
        self.label3.setAlignment(QtCore.Qt.AlignCenter)
        self.label3.setObjectName("label3")
        self.label5 = QtWidgets.QLabel(self.widget_12)
        self.label5.setGeometry(QtCore.QRect(310, 110, 51, 21))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label5.setFont(font)
        self.label5.setStyleSheet("color: rgb(0, 0, 0);")
        self.label5.setAlignment(QtCore.Qt.AlignCenter)
        self.label5.setObjectName("label5")
        self.horizontalSlider2 = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider2.setGeometry(QtCore.QRect(420, 5, 181, 30))
        self.horizontalSlider2.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider2.setMaximum(255)
        self.horizontalSlider2.setPageStep(1)
        self.horizontalSlider2.setProperty("value", 255)
        self.horizontalSlider2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider2.setObjectName("horizontalSlider2")
        self.horizontalSlider4 = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider4.setGeometry(QtCore.QRect(420, 55, 181, 30))
        self.horizontalSlider4.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider4.setMaximum(255)
        self.horizontalSlider4.setPageStep(1)
        self.horizontalSlider4.setProperty("value", 255)
        self.horizontalSlider4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider4.setObjectName("horizontalSlider4")
        self.horizontalSlider6 = QtWidgets.QSlider(self.widget_12)
        self.horizontalSlider6.setGeometry(QtCore.QRect(420, 105, 181, 30))
        self.horizontalSlider6.setStyleSheet("QSlider::groove:horizontal {\n"
"    background: transparent;\n"
"    height:10px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 30px;\n"
"height:30px;\n"
"margin-top: -10px;\n"
"margin-bottom: -10px;\n"
"border-radius:15px;\n"
"background: rgba(220, 220, 220);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: rgb(70, 70, 70);\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF7826;\n"
"}")
        self.horizontalSlider6.setMaximum(255)
        self.horizontalSlider6.setPageStep(1)
        self.horizontalSlider6.setProperty("value", 255)
        self.horizontalSlider6.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider6.setObjectName("horizontalSlider6")
        self.label2 = QtWidgets.QLabel(self.widget_12)
        self.label2.setGeometry(QtCore.QRect(640, 10, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label2.setFont(font)
        self.label2.setStyleSheet("color: rgb(0, 0, 0);")
        self.label2.setAlignment(QtCore.Qt.AlignCenter)
        self.label2.setObjectName("label2")
        self.label4 = QtWidgets.QLabel(self.widget_12)
        self.label4.setGeometry(QtCore.QRect(640, 60, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label4.setFont(font)
        self.label4.setStyleSheet("color: rgb(0, 0, 0);")
        self.label4.setAlignment(QtCore.Qt.AlignCenter)
        self.label4.setObjectName("label4")
        self.label6 = QtWidgets.QLabel(self.widget_12)
        self.label6.setGeometry(QtCore.QRect(640, 110, 51, 20))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.label6.setFont(font)
        self.label6.setStyleSheet("color: rgb(0, 0, 0);")
        self.label6.setAlignment(QtCore.Qt.AlignCenter)
        self.label6.setObjectName("label6")
        self.pushButton_reduce1 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_reduce1.setGeometry(QtCore.QRect(40, 5, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(30)
        self.pushButton_reduce1.setFont(font)
        self.pushButton_reduce1.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_reduce1.setObjectName("pushButton_reduce1")
        self.pushButton_plus1 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_plus1.setGeometry(QtCore.QRect(270, 5, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(25)
        self.pushButton_plus1.setFont(font)
        self.pushButton_plus1.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_plus1.setObjectName("pushButton_plus1")
        self.pushButton_reduce3 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_reduce3.setGeometry(QtCore.QRect(40, 55, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(30)
        self.pushButton_reduce3.setFont(font)
        self.pushButton_reduce3.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_reduce3.setObjectName("pushButton_reduce3")
        self.pushButton_reduce5 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_reduce5.setGeometry(QtCore.QRect(40, 105, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(30)
        self.pushButton_reduce5.setFont(font)
        self.pushButton_reduce5.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_reduce5.setObjectName("pushButton_reduce5")
        self.pushButton_plus3 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_plus3.setGeometry(QtCore.QRect(270, 55, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(25)
        self.pushButton_plus3.setFont(font)
        self.pushButton_plus3.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_plus3.setObjectName("pushButton_plus3")
        self.pushButton_plus5 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_plus5.setGeometry(QtCore.QRect(270, 105, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(25)
        self.pushButton_plus5.setFont(font)
        self.pushButton_plus5.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_plus5.setObjectName("pushButton_plus5")
        self.pushButton_reduce2 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_reduce2.setGeometry(QtCore.QRect(380, 5, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(30)
        self.pushButton_reduce2.setFont(font)
        self.pushButton_reduce2.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_reduce2.setObjectName("pushButton_reduce2")
        self.pushButton_reduce4 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_reduce4.setGeometry(QtCore.QRect(380, 55, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(30)
        self.pushButton_reduce4.setFont(font)
        self.pushButton_reduce4.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_reduce4.setObjectName("pushButton_reduce4")
        self.pushButton_reduce6 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_reduce6.setGeometry(QtCore.QRect(380, 105, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(30)
        self.pushButton_reduce6.setFont(font)
        self.pushButton_reduce6.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_reduce6.setObjectName("pushButton_reduce6")
        self.pushButton_plus2 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_plus2.setGeometry(QtCore.QRect(610, 5, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(25)
        self.pushButton_plus2.setFont(font)
        self.pushButton_plus2.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_plus2.setObjectName("pushButton_plus2")
        self.pushButton_plus4 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_plus4.setGeometry(QtCore.QRect(610, 55, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(25)
        self.pushButton_plus4.setFont(font)
        self.pushButton_plus4.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_plus4.setObjectName("pushButton_plus4")
        self.pushButton_plus6 = QtWidgets.QPushButton(self.widget_12)
        self.pushButton_plus6.setGeometry(QtCore.QRect(610, 105, 30, 30))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(25)
        self.pushButton_plus6.setFont(font)
        self.pushButton_plus6.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:2px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_plus6.setObjectName("pushButton_plus6")
        self.widget_15 = QtWidgets.QWidget(Form)
        self.widget_15.setGeometry(QtCore.QRect(1190, 670, 141, 81))
        self.widget_15.setStyleSheet("QWidget {\n"
"background-color: rgb(255, 255, 255);\n"
"border-radius:6px;}")
        self.widget_15.setObjectName("widget_15")
        self.pushButton_exit = QtWidgets.QPushButton(self.widget_15)
        self.pushButton_exit.setGeometry(QtCore.QRect(17, 20, 110, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.pushButton_exit.setFont(font)
        self.pushButton_exit.setStyleSheet("QPushButton{\n"
"background-color: #A2A2A2;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_exit.setObjectName("pushButton_exit")
        self.widget_2 = QtWidgets.QWidget(Form)
        self.widget_2.setGeometry(QtCore.QRect(10, 5, 1320, 490))
        self.widget_2.setStyleSheet("QWidget {\n"
"background-color: rgb(255, 255, 255);\n"
"border-radius:6px;}")
        self.widget_2.setObjectName("widget_2")
        self.label_orign = QtWidgets.QLabel(self.widget_2)
        self.label_orign.setGeometry(QtCore.QRect(20, 5, 1280, 480))
        self.label_orign.setText("")
        self.label_orign.setAlignment(QtCore.Qt.AlignCenter)
        self.label_orign.setObjectName("label_orign")
        self.widget_16 = QtWidgets.QWidget(Form)
        self.widget_16.setGeometry(QtCore.QRect(1030, 500, 151, 251))
        self.widget_16.setStyleSheet("QWidget {\n"
"background-color: rgb(255, 255, 255);\n"
"border-radius:6px;}")
        self.widget_16.setObjectName("widget_16")
        self.comboBox_color = QtWidgets.QComboBox(self.widget_16)
        self.comboBox_color.setGeometry(QtCore.QRect(20, 40, 111, 22))
        self.comboBox_color.setStyleSheet("background-color: #FF7826;\n"
"selection-color: rgb(0, 0, 0);\n"
"selection-background-color: #FFA500;")
        self.comboBox_color.setObjectName("comboBox_color")
        self.pushButton_addcolor = QtWidgets.QPushButton(self.widget_16)
        self.pushButton_addcolor.setGeometry(QtCore.QRect(30, 70, 90, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.pushButton_addcolor.setFont(font)
        self.pushButton_addcolor.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_addcolor.setObjectName("pushButton_addcolor")
        self.pushButton_deletecolor = QtWidgets.QPushButton(self.widget_16)
        self.pushButton_deletecolor.setGeometry(QtCore.QRect(30, 130, 90, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.pushButton_deletecolor.setFont(font)
        self.pushButton_deletecolor.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_deletecolor.setObjectName("pushButton_deletecolor")
        self.label = QtWidgets.QLabel(self.widget_16)
        self.label.setGeometry(QtCore.QRect(40, 10, 67, 17))
        self.label.setObjectName("label")
        self.pushButton_labWrite = QtWidgets.QPushButton(self.widget_16)
        self.pushButton_labWrite.setGeometry(QtCore.QRect(30, 190, 90, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.pushButton_labWrite.setFont(font)
        self.pushButton_labWrite.setStyleSheet("QPushButton{\n"
"background-color: #FFA500;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_labWrite.setObjectName("pushButton_labWrite")
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setGeometry(QtCore.QRect(720, 500, 301, 255))
        self.frame.setStyleSheet("background-image: url(:/image/resource/cielab-color.jpg);")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.textBrowser = QtWidgets.QTextBrowser(Form)
        self.textBrowser.setGeometry(QtCore.QRect(10, 645, 701, 111))
        self.textBrowser.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.textBrowser.setObjectName("textBrowser")
        self.widget_17 = QtWidgets.QWidget(Form)
        self.widget_17.setGeometry(QtCore.QRect(1190, 580, 141, 81))
        self.widget_17.setStyleSheet("QWidget {\n"
"background-color: rgb(255, 255, 255);\n"
"border-radius:6px;}")
        self.widget_17.setObjectName("widget_17")
        self.pushButton_camera_change = QtWidgets.QPushButton(self.widget_17)
        self.pushButton_camera_change.setGeometry(QtCore.QRect(17, 20, 110, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        font.setPointSize(14)
        self.pushButton_camera_change.setFont(font)
        self.pushButton_camera_change.setStyleSheet("QPushButton{\n"
"background-color: #A2A2A2;\n"
"color:rgb(255, 255, 255)\n"
"}\n"
"QPushButton{border-radius:6px;}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(126, 188, 89, 0);}")
        self.pushButton_camera_change.setObjectName("pushButton_camera_change")
        self.widget_18 = QtWidgets.QWidget(Form)
        self.widget_18.setGeometry(QtCore.QRect(1190, 500, 141, 71))
        self.widget_18.setStyleSheet("QWidget {\n"
"background-color: rgb(255, 255, 255);\n"
"border-radius:6px;}")
        self.widget_18.setObjectName("widget_18")
        self.radioButton_zn = QtWidgets.QRadioButton(self.widget_18)
        self.radioButton_zn.setGeometry(QtCore.QRect(30, 10, 71, 27))
        self.radioButton_zn.setObjectName("radioButton_zn")
        self.radioButton_en = QtWidgets.QRadioButton(self.widget_18)
        self.radioButton_en.setGeometry(QtCore.QRect(30, 40, 71, 27))
        self.radioButton_en.setObjectName("radioButton_en")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "LAB_Tool 1.0"))
        self.label_25.setText(_translate("Form", "L"))
        self.label_26.setText(_translate("Form", "A"))
        self.label_27.setText(_translate("Form", "B"))
        self.label1.setText(_translate("Form", "0"))
        self.label3.setText(_translate("Form", "0"))
        self.label5.setText(_translate("Form", "0"))
        self.label2.setText(_translate("Form", "255"))
        self.label4.setText(_translate("Form", "255"))
        self.label6.setText(_translate("Form", "255"))
        self.pushButton_reduce1.setText(_translate("Form", "-"))
        self.pushButton_plus1.setText(_translate("Form", "+"))
        self.pushButton_reduce3.setText(_translate("Form", "-"))
        self.pushButton_reduce5.setText(_translate("Form", "-"))
        self.pushButton_plus3.setText(_translate("Form", "+"))
        self.pushButton_plus5.setText(_translate("Form", "+"))
        self.pushButton_reduce2.setText(_translate("Form", "-"))
        self.pushButton_reduce4.setText(_translate("Form", "-"))
        self.pushButton_reduce6.setText(_translate("Form", "-"))
        self.pushButton_plus2.setText(_translate("Form", "+"))
        self.pushButton_plus4.setText(_translate("Form", "+"))
        self.pushButton_plus6.setText(_translate("Form", "+"))
        self.pushButton_exit.setText(_translate("Form", "关闭软件"))
        self.pushButton_addcolor.setText(_translate("Form", "新增颜色"))
        self.pushButton_deletecolor.setText(_translate("Form", "删除颜色"))
        self.label.setText(_translate("Form", "颜色列表"))
        self.pushButton_labWrite.setText(_translate("Form", "保存设置"))
        self.textBrowser.setHtml(_translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">LAB是由一个亮度通道和两个颜色通道组成的，每个颜色用L、A、B三个数字表示</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">- L*代表<span style=\" font-weight:600;\">亮度</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">- A*代表从<span style=\" font-weight:600;\">绿色</span>到<span style=\" font-weight:600;\">红色</span>的分量</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">- B*代表从<span style=\" font-weight:600;\">蓝色</span>到<span style=\" font-weight:600;\">黄色</span>的分量</p></body></html>"))
        self.pushButton_camera_change.setText(_translate("Form", "深度相机"))
        self.radioButton_zn.setText(_translate("Form", "中文"))
        self.radioButton_en.setText(_translate("Form", "English"))

import resource_rc

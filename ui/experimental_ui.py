#!/usr/bin/env python2

import sys
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *

class controlTable(QTableWidget):
	def __init__(self):
		super(QTableWidget, self).__init__()

	def itemClicked(self, ev):
			QTableView.selectRow(self,row)
class ParameterWindow(QWidget):

	def __init__(self):
		super(QWidget,self).__init__()
		self.initUI()

	def initUI(self):
		rospy.init_node('experiment')

		self.frame1 = QFrame()
		self.frame1.setStyleSheet("background-color: rgb(200, 255, 255)")

		self.horiz_layout1 = QHBoxLayout()
		self.horiz_layout2 = QHBoxLayout()
		self.horiz_layout3 = QHBoxLayout()
		self.horiz_layout4 = QHBoxLayout()
		self.horiz_layout5 = QHBoxLayout()
		self.horiz_layout6 = QHBoxLayout()

		self.vert_layout = QVBoxLayout()


		self.font = QFont()
		self.font.setBold(True)



		self.setWindowTitle("Experiment UI")
		self.setGeometry(500,200,1000,600)

		self.lbl_box = QLabel(self)
		self.lbl_box.setText('Participant ID:')
		self.lbl_box.setFont(self.font)
		self.horiz_layout1.addWidget(self.lbl_box)

		self.textbox = QLineEdit(self)
		self.textbox.resize(280,40)
		self.horiz_layout1.addWidget(self.textbox)
		
		self.smt_btn = QPushButton('Submit',self)
		self.smt_btn.clicked.connect(self.submit_data)
		self.smt_btn.setMaximumWidth(120)
		self.horiz_layout1.addWidget(self.smt_btn)


		self.lbl_tbl = QLabel(self)
		self.lbl_tbl.setText('Traverse List:')
		self.lbl_tbl.setFont(self.font)

		self.table = controlTable()
		self.table = QTableWidget(5,5,self)

		self.table.setHorizontalHeaderLabels(('Goal ID', 'Pos X', 'Pos Y','Theta','Fuel'))
		self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.table.setMaximumWidth(self.table.horizontalHeader().length()+30)
		self.table.setMaximumHeight(self.table.verticalHeader().length()+25)
		self.horiz_layout2.addWidget(self.table)

		self.go_btn = QPushButton('Go!',self)
		self.go_btn.clicked.connect(self.submit_data)
		self.go_btn.setMaximumWidth(120)
		self.horiz_layout2.addWidget(self.go_btn)

		self.buildLabels()

		self.cur_fuel_lbl = QLabel(self)
		self.cur_fuel = QLabel(self)
		self.cur_fuel_lbl.setText('Current Fuel:')
		self.cur_fuel_lbl.setFont(self.font)
		self.cur_fuel.setText('0')


		self.quit_btn = QPushButton('Quit',self)
		self.quit_btn.clicked.connect(self.closer)
		self.quit_btn.setMaximumWidth(120)

		self.buildWidgets()
		

	def submit_data(self):
		self.msg.id = int(self.textbox.text())
		self.hide()
		while not rospy.is_shutdown():
			self.pub.publish(self.msg)
			self.r.sleep()

	def closer(self):
		self.close()

	def buildLabels(self):
		self.cur_trav_lbl = QLabel(self)
		self.trav_goal = QLabel(self)
		self.trav_x = QLabel(self)
		self.trav_y = QLabel(self)
		self.trav_theta = QLabel(self)
		self.trav_x2 = QLabel(self)
		self.trav_y2 = QLabel(self)
		self.trav_theta2 = QLabel(self)
		self.trav_goal_val = QLabel(self)
		self.trav_x_val = QLabel(self)
		self.trav_y_val = QLabel(self)
		self.trav_theta_val = QLabel(self)

		self.trav_x2_val = QLabel(self)
		self.trav_y2_val = QLabel(self)
		self.trav_theta2_val = QLabel(self)


		self.cur_trav_lbl.setText('Current Traverse:')
		self.cur_trav_lbl.setFont(self.font)
		self.trav_goal.setText('Goal ID')
		self.trav_x.setText('Pos X')
		self.trav_y.setText('Pos Y')
		self.trav_theta.setText('Theta')


		self.trav_goal_val.setText('0')
		self.trav_x_val.setText('0')
		self.trav_y_val.setText('0')
		self.trav_theta_val.setText('0')

		self.trav_x2_val.setText('0')
		self.trav_y2_val.setText('0')
		self.trav_theta2_val.setText('0')


		self.trav_x2.setText('Pos X')
		self.trav_y2.setText('Pos Y')
		self.trav_theta2.setText('Theta')

		self.list = [self.trav_goal,self.trav_x,self.trav_y,self.trav_theta]


		self.cur_pos_lbl = QLabel(self)
		self.cur_pos_lbl.setText('Current Position:')
		self.cur_pos_lbl.setFont(self.font)

		self.list2 = [self.trav_x2,self.trav_y2,self.trav_theta2]
		self.list3 = [self.trav_goal_val,self.trav_x_val,self.trav_y_val,self.trav_theta_val]
		self.list4 = [self.trav_x2_val,self.trav_y2_val,self.trav_theta2_val]

		for i in range(0,len(self.list)):
			self.horiz_layout3.addWidget(self.list[i])
			self.horiz_layout5.addWidget(self.list3[i])

		for j in range(0,len(self.list2)):
			self.horiz_layout4.addWidget(self.list2[j])
			self.horiz_layout6.addWidget(self.list4[j])



	def buildWidgets(self):
		self.vert_layout.addLayout(self.horiz_layout1)
		self.vert_layout.addWidget(self.lbl_tbl)
		self.vert_layout.addLayout(self.horiz_layout2)

		self.vert_layout.addWidget(self.cur_trav_lbl)
		self.vert_layout.addLayout(self.horiz_layout3)
		self.vert_layout.addLayout(self.horiz_layout5)
		self.vert_layout.addWidget(self.cur_pos_lbl)
		self.vert_layout.addLayout(self.horiz_layout4)
		self.vert_layout.addLayout(self.horiz_layout6)
		self.vert_layout.addWidget(self.cur_fuel_lbl)
		self.vert_layout.addWidget(self.cur_fuel)

		self.vert_layout.addWidget(self.quit_btn)

		self.vert_layout.addSpacing(10)
		self.vert_layout.setContentsMargins(30,30,30,30)
		self.vert_layout.addStretch()
		self.setLayout(self.vert_layout)

		self.show()


if __name__ == '__main__':
	app = QApplication(sys.argv)
	coretools_app = ParameterWindow()
	sys.exit(app.exec_())


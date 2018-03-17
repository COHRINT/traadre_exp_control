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

		self.horiz_layout1 = QHBoxLayout()
		self.horiz_layout2 = QHBoxLayout()
		self.vert_layout = QVBoxLayout()
		self.setWindowTitle("Experiment UI")
		self.setGeometry(500,200,1000,600)

		self.lbl_box = QLabel(self)
		self.lbl_box.setText('Participant ID:')
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

		self.buildTables()

		self.quit_btn = QPushButton('Quit',self)
		self.quit_btn.clicked.connect(self.submit_data)
		self.quit_btn.setMaximumWidth(120)

		self.buildWidgets()
		

	def submit_data(self):
		self.msg.id = int(self.textbox.text())
		self.hide()
		while not rospy.is_shutdown():
			self.pub.publish(self.msg)
			self.r.sleep()

	def buildTables(self):
		self.cur_trav_lbl = QLabel(self)
		self.cur_trav_lbl.setText('Current Traverse:')
		self.trav_table = QTableWidget(1,4,self)
		self.trav_table.setHorizontalHeaderLabels(('Goal ID', 'Pos X', 'Pos Y','Theta'))
		self.trav_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.trav_table.setMaximumWidth(self.trav_table.horizontalHeader().length()+20)
		self.trav_table.setMaximumHeight(self.trav_table.verticalHeader().length()+25)



		self.cur_pos_lbl = QLabel(self)
		self.cur_pos_lbl.setText('Current Position:')
		self.cur_pos_table = QTableWidget(1,3,self)
		self.cur_pos_table.setHorizontalHeaderLabels(('Pos X', 'Pos Y','Theta'))
		self.cur_pos_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.cur_pos_table.setMaximumWidth(self.cur_pos_table.horizontalHeader().length()+20)
		self.cur_pos_table.setMaximumHeight(self.cur_pos_table.verticalHeader().length()+25)


		self.cur_fuel = QLabel(self)
		self.cur_fuel.setText('Current Fuel:')
		self.cur_fuel_table = QTableWidget(1,1,self)
		self.cur_fuel_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.cur_fuel_table.setMaximumWidth(self.cur_fuel_table.horizontalHeader().length()+20)
		self.cur_fuel_table.setMaximumHeight(self.cur_fuel_table.verticalHeader().length()+25)

	def buildWidgets(self):
		self.vert_layout.addLayout(self.horiz_layout1)
		self.vert_layout.addWidget(self.lbl_tbl)
		self.vert_layout.addLayout(self.horiz_layout2)

		self.vert_layout.addWidget(self.cur_trav_lbl)
		self.vert_layout.addWidget(self.trav_table)
		self.vert_layout.addWidget(self.cur_pos_lbl)
		self.vert_layout.addWidget(self.cur_pos_table)
		self.vert_layout.addWidget(self.cur_fuel)
		self.vert_layout.addWidget(self.cur_fuel_table)
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




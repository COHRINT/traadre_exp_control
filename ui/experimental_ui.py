#!/usr/bin/env python2

import sys
import rospy
from traadre_msgs.msg import *
from traadre_msgs.srv import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *

class controlTable(QTableWidget):
	def __init__(self):
		super(QTableWidget, self).__init__()


	def clickEvent():
		print('e')
class ParameterWindow(QWidget):

	def __init__(self):
		super(QWidget,self).__init__()
		self.initUI()

	def initUI(self):
		rospy.init_node('Experiment')


		self.frame1 = QFrame()
		self.frame1.setStyleSheet("background-color: rgb(200, 255, 255)")
		self.pub = rospy.Publisher('Experiment', part_id, queue_size=10, latch=True)
		self.msg = part_id()


		self.horiz_layout1 = QHBoxLayout()
		self.horiz_layout2 = QHBoxLayout()
		self.horiz_layout3 = QHBoxLayout()
		self.horiz_layout4 = QHBoxLayout()
		self.horiz_layout5 = QHBoxLayout()
		self.horiz_layout6 = QHBoxLayout()
		self.horiz_layout7 = QHBoxLayout()

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


		self.goal_titles, self.row = self.getGoals_client()

		self.buildTable()

		self.go_btn = QPushButton('Go!',self)
		self.go_btn.clicked.connect(self.choose_goal)
		self.go_btn.setMaximumWidth(120)
		self.horiz_layout2.addWidget(self.go_btn)

		self.buildLabels()

		self.cur_fuel_lbl = QLabel(self)
		self.cur_fuel = QLabel(self)
		self.cur_fuel_lbl.setText('Current Fuel:')
		self.cur_fuel_lbl.setFont(self.font)

		self.state_sub = rospy.Subscriber('state', RobotState, self.state_callback)
		self.cur_fuel.setText('0')


		self.quit_btn = QPushButton('Quit',self)
		self.quit_btn.clicked.connect(self.closer)
		self.quit_btn.setMaximumWidth(120)


		self.mdp_btn = QRadioButton("MDP",self)
		self.gc_btn = QRadioButton("GC",self)
		self.mdp_btn.clicked.connect(self.mdp_client)
		self.gc_btn.clicked.connect(self.gc_client)
		self.horiz_layout7.addWidget(self.mdp_btn)
		self.horiz_layout7.addWidget(self.gc_btn)
		self.buildWidgets()
		

	def submit_data(self):
		self.msg.id = int(self.textbox.text())
		self.pub.publish(self.msg)
		
	def state_callback(self, data):
		self._robotFuel = data.fuel
		self.worldX = data.pose.position.x
		self.worldY = data.pose.position.y

		self.trav_x2_val.setText(str(self.worldX))
		self.trav_y2_val.setText(str(self.worldY))

		self.cur_fuel.setText(str(self._robotFuel))

	def choose_goal(self):
		self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()) 
		self.trav_goal_val.setText(self.table.item(self.table.currentRow(),0).text())
		self.trav_x_val.setText(str(self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()).x))
		self.trav_y_val.setText(str(self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()).y))

	def closer(self):
		self.close()

	def mdp_client(self):
		try:
			mux = rospy.ServiceProxy('/topic_tools/MuxSelect', topic_tools/MuxSelect)
			response = mux(MDP)
			print(response)
			return response
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
   
	def gc_client(self):
		return
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


		self.trav_goal_val.setText('0')
		self.trav_x_val.setText('0')
		self.trav_y_val.setText('0')


		self.trav_x2_val.setText('0')
		self.trav_y2_val.setText('0')
		self.trav_theta2_val.setText('0')


		self.trav_x2.setText('Pos X')
		self.trav_y2.setText('Pos Y')
		self.trav_theta2.setText('Theta')

		self.list = [self.trav_goal,self.trav_x,self.trav_y]


		self.cur_pos_lbl = QLabel(self)
		self.cur_pos_lbl.setText('Current Position:')
		self.cur_pos_lbl.setFont(self.font)

		self.list2 = [self.trav_x2,self.trav_y2,self.trav_theta2]
		self.list3 = [self.trav_goal_val,self.trav_x_val,self.trav_y_val]
		self.list4 = [self.trav_x2_val,self.trav_y2_val,self.trav_theta2_val]

		for i in range(0,len(self.list)):
			self.horiz_layout3.addWidget(self.list[i])
			self.horiz_layout5.addWidget(self.list3[i])

		for j in range(0,len(self.list2)):
			self.horiz_layout4.addWidget(self.list2[j])
			self.horiz_layout6.addWidget(self.list4[j])


	def buildTable(self):
		self.table = controlTable()
		self.table = QTableWidget(len(self.goal_titles),5,self)

		self.table.setHorizontalHeaderLabels(('Goal ID', 'Pos X', 'Pos Y','Theta','Fuel'))
		self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.table.setMaximumWidth(self.table.horizontalHeader().length()+30)
		self.table.setMaximumHeight(self.table.verticalHeader().length()+25)

		for i in range(0,self.table.rowCount()):
			self.item = QTableWidgetItem(self.goal_titles[i])
			itemx = QTableWidgetItem( '%1.2f' % self.row[i].x)
			itemy = QTableWidgetItem( '%1.2f' % self.row[i].y)
			item_theta = QTableWidgetItem( '%1.2f' % self.row[i].theta)

			self.table.setItem(i, 0, self.item)
			self.table.setItem(i, 1, itemx)
			self.table.setItem(i, 2, itemy)
			self.table.setItem(i, 3, item_theta)

		self.table.cellDoubleClicked.connect(self.clickEvent)
		self.horiz_layout2.addWidget(self.table)
	def setCurrentGoal_client(self,id):
		try:
			goal = rospy.ServiceProxy('/policy_server/SetCurrentGoal', SetCurrentGoal)
			response = goal(id)
			return response.goal
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
   
	def getGoals_client(self):
		try:
			goal = rospy.ServiceProxy('/policy_server/GetGoalList', GetGoalList)
			response = goal()
			row = response.goals
			return response.ids, row
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
   
	def clickEvent(self):
		self.table.setCurrentItem(self.item)
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
		self.vert_layout.addLayout(self.horiz_layout7)

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



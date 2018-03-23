#!/usr/bin/env python2

import sys
import math
import rospy
from traadre_msgs.msg import *
from traadre_msgs.srv import *
from topic_tools.srv import *

import tf
from tf.transformations import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication, Qt
from PyQt5.QtGui import *



class ParameterWindow(QWidget):

	def __init__(self):
		super(QWidget,self).__init__()
		self.initUI()

	def initUI(self):
		rospy.init_node('Experiment')

		#self.pub = rospy.Publisher('Experiment', part_id, queue_size=10, latch=True)
		self.pub = rospy.Publisher('GoalComplete', GoalComplete, queue_size=10)
		self.callout_Pub = rospy.Publisher('Callout', Callout, queue_size=10)
		self.msg = GoalComplete()
		self.callout_msg = Callout()

		self.horiz_layout1 = QHBoxLayout()
		self.horiz_layout2 = QHBoxLayout()
		self.horiz_layout3 = QHBoxLayout()
		self.horiz_layout4 = QHBoxLayout()
		self.horiz_layout5 = QHBoxLayout()
		self.horiz_layout6 = QHBoxLayout()
		self.horiz_layout7 = QHBoxLayout()

		self.fuelGroup = QGroupBox()
		self.fuelLayout = QHBoxLayout()


		self.helperGroup = QGroupBox()
		self.helperLayout = QHBoxLayout()

		self.vert_layout = QVBoxLayout()
		self.isGoal = False
		self.missed = False
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
		self.lbl_tbl.setText('Goal List:')
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
		self.callout_btn = QPushButton('Callout Missed',self)
		self.callout_btn.clicked.connect(self.callout_missed)
		self.callout_btn.setMaximumWidth(120)
		self.fuelLayout.addWidget(self.cur_fuel_lbl)
		self.fuelLayout.addWidget(self.cur_fuel)
		self.fuelLayout.addWidget(self.callout_btn)
		self.fuelLayout.addStretch(0)


		self.state_sub = rospy.Subscriber('state', RobotState, self.state_callback)
		self.cur_fuel.setText('0')


		self.quit_btn = QPushButton('Quit',self)
		self.quit_btn.clicked.connect(self.closer)
		self.quit_btn.setMaximumWidth(120)


		self.mdp_btn = QRadioButton("MDP",self)
		self.gc_btn = QRadioButton("GC",self)
		self.mdp_btn.clicked.connect(self.mdp_client)
		self.gc_btn.clicked.connect(self.gc_client)
		self.helperLayout.addWidget(self.mdp_btn)
		self.helperLayout.addWidget(self.gc_btn)

		self.buildWidgets()

	def submit_data(self):
		self.msg.id = int(self.textbox.text())
		self.pub.publish(self.msg)

	def state_callback(self, data):
		self._robotFuel = data.fuel
		self.worldX = data.pose.position.x
		self.worldY = data.pose.position.y
				
		worldRoll, worldPitch, self.worldYaw = euler_from_quaternion([data.pose.orientation.x,
										  data.pose.orientation.y,
										  data.pose.orientation.z,
										  data.pose.orientation.w],'sxyz')

		self.trav_x2_val.setText('%1.2f'  %self.worldX)
		self.trav_y2_val.setText('%1.2f'  %self.worldY)
		self.trav_theta2_val.setText('%1.2f'  % self.worldYaw)
		self.cur_fuel.setText('%1.2f %%' % (self._robotFuel))

		if(self.isGoal == True):
			if math.sqrt(pow((self.goalx - self.worldX),2) + pow((self.goaly - self.worldY),2)) < 1:
					self.msg.goal_id = self.table.item(self.table.currentRow(),0).text()
					self.msg.fuel = self._robotFuel*100
					self.msg.header.stamp = rospy.Time.now()
					self.table.setCurrentCell(self.table.currentRow()+1,0) 
					self.pub.publish(self.msg)
					self.choose_goal()


	def choose_goal(self):
		self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()) 
		self.trav_goal_val.setText(self.table.item(self.table.currentRow(),0).text())

		self.goaly = self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()).y
		self.goalx = self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()).x

		self.trav_x_val.setText('%1.2f' % self.goalx)
		self.trav_y_val.setText('%1.2f' % self.goaly)		
		
		self.isGoal = True
	def closer(self):
		self.close()

	def callout_missed(self):
		self.missed = True
		self.callout_msg.fuel = (5* math.ceil(float(self._robotFuel)/5))
		self.callout_msg.made = False
		self.callout_msg.header.stamp = rospy.Time.now()
		self.callout_Pub.publish(self.callout_msg)

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


		self.cur_trav_lbl.setText('Current Goal:')
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

			self.item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			itemx.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			itemy.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			item_theta.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)


			self.table.setItem(i, 0, self.item)
			self.table.setItem(i, 1, itemx)
			self.table.setItem(i, 2, itemy)
			self.table.setItem(i, 3, item_theta)

		self.horiz_layout2.addWidget(self.table)
	def mdp_client(self):
		try:
			mux = rospy.ServiceProxy('/mux/select', MuxSelect)               
			req = MuxSelectRequest(topic='/policy/current_steer')
			resp = mux(req)
			return resp
				
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
   
	def gc_client(self):
		try:
			mux = rospy.ServiceProxy('/mux/select', MuxSelect)            
			req = MuxSelectRequest(topic='/ui/steer')
			resp = mux(req)
			return resp
				
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
   

	def setCurrentGoal_client(self,id):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/SetCurrentGoal', SetCurrentGoal)

			response = goal(id)
			return response.goal
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
   
	def getGoals_client(self):
		try:
			goal = rospy.ServiceProxy('/policy/policy_server/GetGoalList', GetGoalList)
			response = goal()
			row = response.goals
			return response.ids, row
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

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

		self.fuelGroup.setLayout(self.fuelLayout)
		self.fuelGroup.setStyleSheet("QGroupBox { background-color: rgb(255, 255,\
255); border:5px solid rgb(255, 170, 255); }")
		self.vert_layout.addWidget(self.fuelGroup)

		self.helperGroup.setLayout(self.helperLayout)
		self.helperGroup.setStyleSheet("QGroupBox { background-color: rgb(255, 255,\
255); border:1px solid rgb(255, 170, 255); }")
		self.vert_layout.addWidget(self.helperGroup)


		self.vert_layout.addWidget(self.quit_btn)

		self.vert_layout.addSpacing(10)
		self.vert_layout.setContentsMargins(30,30,30,30)
		self.vert_layout.addStretch()
		self.setLayout(self.vert_layout)

		self.show()

		
def main():
	app = QApplication(sys.argv)
	coretools_app = ParameterWindow()
	sys.exit(app.exec_())

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass



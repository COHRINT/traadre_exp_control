#!/usr/bin/env python2

import sys
import csv
import math
import rospy
import signal

from traadre_msgs.msg import *
from traadre_msgs.srv import *
from topic_tools.srv import *

import tf
from tf.transformations import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
        sys.exit(0)
        
signal.signal(signal.SIGINT, signal_handler)

class ParameterWindow(QWidget):
	part_id_desired = pyqtSignal()
	callouts_desired = pyqtSignal()
	location_update = pyqtSignal(int, int, int, int)
	goal_chosen = pyqtSignal(int,int)
	goal_reached = pyqtSignal()

	def __init__(self):
		super(QWidget,self).__init__()
		self.initUI()

	def initUI(self):
		rospy.init_node('Experiment')
 		

		self.pub = rospy.Publisher('goal_complete', GoalComplete, queue_size=10)
                self.teleportPub = rospy.Publisher('state_cmd', RobotState, queue_size=10)
                self.fakeSteerPub = rospy.Publisher('steer', Steering, queue_size=10)
                
		self.msg = GoalComplete()

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

		self.part_id_desired.connect(self.part_id_client)
		self.location_update.connect(self.location_client)
		self.goal_chosen.connect(self.goal_choose_client)
		self.goal_reached.connect(self.goal_reached_client)
		#self.part_id_desired.emit()

		self.lbl_tbl = QLabel(self)
		self.lbl_tbl.setText('Goal List:')
		self.lbl_tbl.setFont(self.font)


		self.goal_titles, self.row = self.getGoals_client()

                self.allGoals = zip(self.goal_titles, self.row) #Zip the tuples together so I get a list of tuples (instead of a tuple of lists)
                self.allGoals = sorted(self.allGoals, key=lambda param: param[0]) #Sort the combined list by the goal ID
                
		self.buildTable()

		self.go_btn = QPushButton('Begin',self)
		self.go_btn.clicked.connect(self.choose_goal)
		self.go_btn.setMaximumWidth(125)
                beginPanel = QVBoxLayout()
                beginPanel.addWidget(self.go_btn)
                self.lblTeleport = QLabel('Teleport requested')
                font = QFont()
                font.setBold(True)
                
                self.lblTeleport.setFont(font)
                self.lblTeleport.setStyleSheet('color: red;')
                self.lblTeleport.setVisible(False)
                beginPanel.addWidget(self.lblTeleport)
                
		self.horiz_layout2.addLayout(beginPanel)

		self.buildLabels()

		self.cur_fuel_lbl = QLabel(self)
		self.cur_fuel = QLabel(self)
		self.cur_fuel_lbl.setText('Current Fuel:')
		self.cur_fuel_lbl.setFont(self.font)

		self.fuelLayout.addWidget(self.cur_fuel_lbl)
		self.fuelLayout.addWidget(self.cur_fuel)
		self.fuelLayout.addStretch(0)

		self.callouts_desired.connect(self.callouts_client)
		#self.callouts_desired.emit()

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
                self.gc_btn.setChecked(True)
                self.gc_btn.clicked.emit() #set the initial state to be ground control

                '''
                Mods to support a traverse list:
                1. Read csv file containing initial pos and a pos / quat for each goal: Done
                2. Zip the pos/quats to the goals spit out by the policy server: Done
                3. Implement teleport publishing of a RobotState to /state_cmd : Done
                3a. When a goal is reached by proximity, just add gas : Done
                3b. When a goal is reached from teleport, set everything : Done
                4. Implement teleport requested UI
                5. 
                '''

                #Read the traverse lists
                
                csvFileName = sys.argv[1]
                print 'Using traverses in file:', csvFileName
                
                csvFile = open(csvFileName, 'rb')
                reader = csv.reader(csvFile, delimiter=',')
                dests = []
                for row in reader:
                        dests.append(row)

                #print 'Traverses:', dests
                self.initPos = dests[0]
                self.traverses = dests #take everything but the first one
	        self._robotFuel = 0.0
                self.lastStateMsg = None
                self.teleportNeeded = False

	def location_client(self,x,y,yaw,fuel):
		self.trav_x2_val.setText('%1.2f'  %x)
		self.trav_y2_val.setText('%1.2f'  %y)
		self.trav_theta2_val.setText('%1.2f'  %yaw)
		self.cur_fuel.setText('%1.2f %%' %fuel)

	def goal_choose_client(self, x, y):
		self.trav_x_val.setText('%1.2f' %x)
		self.trav_y_val.setText('%1.2f' %y)		
		
	def callouts_client(self):
		self.callout_Pub = rospy.Publisher('callout', Callout, queue_size=10)
		self.callout_msg = Callout()
		self.callout_btn = QPushButton('Callout Made',self)
		self.callout_btn.clicked.connect(self.callout_made)
		self.callout_btn.setMaximumWidth(120)
		self.fuelLayout.addWidget(self.callout_btn)

	def part_id_client(self):
    		self.pub = rospy.Publisher('Experiment', part_id, queue_size=10, latch=True)

    		self.part_id_layout = QHBoxLayout()
    		self.lbl_box = QLabel(self)
		self.lbl_box.setText('Participant ID:')
		self.lbl_box.setFont(self.font)
		self.part_id_layout.addWidget(self.lbl_box)

		self.textbox = QLineEdit(self)
		self.textbox.resize(280,40)
		self.part_id_layout.addWidget(self.textbox)
		
		self.smt_btn = QPushButton('Submit',self)
		
		self.smt_btn.clicked.connect(self.submit_data)
		self.smt_btn.setMaximumWidth(120)
		self.part_id_layout.addWidget(self.smt_btn)

		self.vert_layout.addLayout(self.part_id_layout)


	def submit_data(self):
		self.msg.id = int(self.textbox.text())
		self.pub.publish(self.msg)

	def state_callback(self, data):
                
                self.lastStateMsg = data
		self._robotFuel = data.fuel
		self.worldX = data.pose.position.x
		self.worldY = data.pose.position.y
		worldRoll, worldPitch, self.worldYaw = euler_from_quaternion([data.pose.orientation.x,
										  data.pose.orientation.y,
										  data.pose.orientation.z,
										  data.pose.orientation.w],'sxyz')


		self.location_update.emit(self.worldX, self.worldY, self.worldYaw, self._robotFuel)

		if(self.isGoal == True):
			if math.sqrt(pow((self.goalx - self.worldX),2) + pow((self.goaly - self.worldY),2)) < 20: #20 is the number of meters for goal transition
					self.msg.goal_id = self.table.item(self.table.currentRow(),0).text()
					self.msg.fuel = self._robotFuel
					self.msg.header.stamp = rospy.Time.now()
					self.pub.publish(self.msg)
					self.goal_reached.emit()
					self.choose_goal()
					self.teleportNeeded = False
                #May need a timeout here to prevent the system from cycling
                if data.needTeleport:
                        self.lblTeleport.setVisible(True)
                        self.teleportNeeded = True
                else:
                        self.lblTeleport.setVisible(False)

	def goal_reached_client(self):
		print("Goal Reached")
		if(self.table.currentRow()+1 < self.table.rowCount()):
			self.table.setCurrentCell(self.table.currentRow()+1,0) 
		else: 
			self.table.setCurrentCell(0,0) 
            


	def choose_goal(self):
                
                teleport = self.teleportNeeded
		self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()) 
		self.trav_goal_val.setText(self.table.item(self.table.currentRow(),0).text())

		self.goaly = self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()).y
		self.goalx = self.setCurrentGoal_client(self.table.item(self.table.currentRow(),0).text()).x

		self.goal_chosen.emit(self.goalx, self.goaly)
		self.isGoal = True

                #Send the robot to the designated initial position if needed
                #The initial position is the currentRow index into the self.traverses list
                msg = RobotState()
                msg.header.stamp = rospy.Time.now()

                #Parse the list we saved on startup
                print 'Selected goal index:', self.table.currentRow()

                initPos = self.traverses[self.table.currentRow()-1] #minus 1 works for all Version 2 pkls -> solves issue with starting on goal
                #starpack.pkl requires -1 removed
                print 'Sending to ', initPos, ' via teleport:', teleport
                #if not teleport:
                msg.pose.position.x = float(initPos[2])
                msg.pose.position.y = float(initPos[1])
                msg.pose.position.z = float(initPos[3])
                msg.pose.orientation.z = float(initPos[6])
                msg.pose.orientation.w = float(initPos[7])
                '''
                else:
                        #Don't teleport the user from where they were just reported
                        if not self.lastStateMsg is None: 
                                msg.pose = self.lastStateMsg.pose
                                msg.pose.position.z += 0.1 #make sure we're above the surface so we don't
                        #get ejected violently
                '''   
                        
                msg.fuel = float(initPos[8])
                msg.needTeleport = False
                self.teleportPub.publish(msg)
                self.lblTeleport.setVisible(False)

                steerMsg = Steering()
                steerMsg.header.stamp = rospy.Time.now()
                steerMsg.goal.x = self.goalx
                steerMsg.goal.y = self.goaly
                steerMsg.goal.theta = 0.0
                worldRoll, worldPitch, worldYaw = euler_from_quaternion([initPos[4],
									      initPos[5],
									      initPos[6],
                                                                              initPos[7]],'sxyz')
                
                steerMsg.steer = worldYaw
                self.fakeSteerPub.publish(steerMsg)
                                                                        
                
	def closer(self):
		self.close()

	def callout_made(self):
		self.missed = False
		self.callout_msg.fuel = float(self._robotFuel) #(5* math.ceil(float(self._robotFuel)/5))
		self.callout_msg.made = True
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

		self.list = [self.trav_goal,self.trav_x,self.trav_y]
		self.list2 = [self.trav_x2,self.trav_y2,self.trav_theta2]
		self.list3 = [self.trav_goal_val,self.trav_x_val,self.trav_y_val]
		self.list4 = [self.trav_x2_val,self.trav_y2_val,self.trav_theta2_val]

		self.cur_trav_lbl.setText('Current Goal:')
		self.cur_trav_lbl.setFont(self.font)
		self.trav_goal.setText('Goal ID')
		self.trav_x.setText('Pos X')
		self.trav_y.setText('Pos Y')

		self.trav_x2.setText('Pos X')
		self.trav_y2.setText('Pos Y')
		self.trav_theta2.setText('Theta')

		self.cur_pos_lbl = QLabel(self)
		self.cur_pos_lbl.setText('Current Position:')
		self.cur_pos_lbl.setFont(self.font)

		for i in range(0,len(self.list)):
			self.list3[i].setText('0')
			self.list4[i].setText('0')
			self.horiz_layout3.addWidget(self.list[i])
			self.horiz_layout5.addWidget(self.list3[i])
			self.horiz_layout4.addWidget(self.list2[i])
			self.horiz_layout6.addWidget(self.list4[i])




	def buildTable(self):
		self.table = QTableWidget(len(self.goal_titles),5,self)

		self.table.setHorizontalHeaderLabels(('Goal ID', 'Pos X', 'Pos Y','Theta','Fuel'))
		self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.table.setMaximumWidth(self.table.horizontalHeader().length()+30)
		self.table.setMaximumHeight(self.table.verticalHeader().length()+25)

		for i in range(0,self.table.rowCount()):
			self.item = QTableWidgetItem(self.allGoals[i][0])
			itemx = QTableWidgetItem( '%1.2f' % self.allGoals[i][1].x)
			itemy = QTableWidgetItem( '%1.2f' % self.allGoals[i][1].y)
			item_theta = QTableWidgetItem( '%1.2f' % self.allGoals[i][1].theta)
                        item_fuel = QTableWidgetItem(str(100))
			self.item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			itemx.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			itemy.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			item_theta.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
			item_fuel.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

			self.table.setItem(i, 0, self.item)
			self.table.setItem(i, 1, itemx)
			self.table.setItem(i, 2, itemy)
			self.table.setItem(i, 3, item_theta)
                        self.table.setItem(i, 4, item_fuel)

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
		self.vert_layout.addWidget(self.lbl_tbl)
		self.vert_layout.addLayout(self.horiz_layout2)

		self.vert_layout.addWidget(self.cur_trav_lbl)
		self.vert_layout.addLayout(self.horiz_layout3)
		self.vert_layout.addLayout(self.horiz_layout5)
		self.vert_layout.addWidget(self.cur_pos_lbl)
		self.vert_layout.addLayout(self.horiz_layout4)
		self.vert_layout.addLayout(self.horiz_layout6)

		self.fuelGroup.setLayout(self.fuelLayout)
		self.fuelGroup.setStyleSheet("QGroupBox {background-color: white; border: 2px inset grey;}")
		self.vert_layout.addWidget(self.fuelGroup)

		self.helperGroup.setLayout(self.helperLayout)
		self.helperGroup.setStyleSheet("QGroupBox {background-color: white; border: 2px inset grey;}")
		self.vert_layout.addWidget(self.helperGroup)

		self.vert_layout.setContentsMargins(30,30,30,30)
		self.vert_layout.addStretch()
		self.vert_layout.addWidget(self.quit_btn)

		self.setLayout(self.vert_layout)

		self.show()
                
class Application(QApplication):
    def event(self, e):
        return QApplication.event(self, e)
		
def main():
        if len(sys.argv) < 2:
                print 'Usage:', sys.argv[0], ' traverse.csv'
                print 'Please provide a traverse csv file'
                sys.exit(-1)
                
	app = Application(sys.argv)
	coretools_app = ParameterWindow()
        signal.signal(signal.SIGINT, lambda *a: app.quit())
        app.startTimer(200)

	sys.exit(app.exec_())

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass



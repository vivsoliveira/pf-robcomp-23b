#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan
import time
from std_msgs.msg import Float64
from aruco_3d import Aruco3d

""" 
	Para inicializar o mapa, em um terminal digite:
	roslaunch my_simulation reuniao.launch

	Para utilizar a garra, em um terminal digite:
	roslaunch mybot_description mybot_control2.launch

	Para executar, em um novo terminal digite:
	rosrun pf-robcomp-23b q2.py
"""

class Questao2:
	def __init__(self, ordem=[]):
		self.rate = rospy.Rate(250) # 250 Hz
		self.point = Point()
		self.lower_hsv = np.array([19,20,50],dtype=np.uint8) # Blue
		self.upper_hsv = np.array([151,255,255],dtype=np.uint8)
		self.kernel = np.ones((3,3),np.uint8)
		self.kp = 600
		self.start = Point(x = -1000)
		# self.sai_do_mesmo_creeeper = False
		self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
		self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
		# Subscribers
		self.bridge = CvBridge()
		self.laser_subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
		self.image_sub = rospy.Subscriber('/camera/image/compressed',CompressedImage,self.image_callback,queue_size=1,buff_size = 2**24)
		self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)
        # Publishers
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)
		self.cmd_vel_pub.publish(Twist())
		self.tempo_atual = 0
		self.tempo_percorido = 0
		self.tempo_inicial = 0
		self.contador = 3
		# Maquina de estados
		self.state = ""
		self.robot_state = "procura"
		self.robot_machine = {
			"procura": self.procura,
			"aproxima": self.aproxima,
			"derruba": self.derruba,
			"para": self.para,
			"ultimo_aruco": self.ultimo_aruco,
			"fim": self.fim,
		}
	
	def image_callback(self, msg: CompressedImage) -> None:
		"""
		Callback function for the image topic
		"""
		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
			hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		except CvBridgeError as e:
			print(e)
		
		self.pontos_aruco, self.areas_aruco = self.color_segmentation(hsv, self.lower_hsv, self.upper_hsv)
	def color_segmentation(self, hsv: np.ndarray, lower_hsv: np.ndarray, upper_hsv: np.ndarray,) -> Point:
		point = Point()
		area = 0
		height,width, _ = hsv.shape
		hsv_2 = hsv.copy()
		hsv_2[0:150] = 0
		mask_2 = cv2.inRange(hsv_2,lower_hsv, upper_hsv)
		mask = cv2.morphologyEx(mask_2,cv2.MORPH_OPEN, self.kernel)
		contours,_ = cv2.findContours(mask_2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		cv2.imshow("visao do robo",mask)
		cv2.waitKey(1)
		self.point.z = hsv.shape[1]/2
		if len(contours) > 0:
			cnt = max(contours, key = lambda x: cv2.contourArea(x))
			
			M = cv2.moments(cnt)
			point.x = int(M['m10']/M['m00'])
			point.y = int(M['m01']/M['m00'])
			point.z = hsv.shape[1]/2
			area = cv2.contourArea(cnt)
		else:
			point.x = -1
			point.y = -1
			point.z = -1
		return point, area

	def odom_callback(self, data: Odometry):
		self.odom = data
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.z = data.pose.pose.position.z
		
		orientation_list = [data.pose.pose.orientation.x,
							data.pose.pose.orientation.y,
							data.pose.pose.orientation.z,
							data.pose.pose.orientation.w]

		self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

	def laser_callback(self, msg: LaserScan) -> None:
		self.laser_msg = np.array(msg.ranges).round(decimals=2) # Converte para np.array e arredonda para 2 casas decimais
		self.laser_msg[self.laser_msg == 0] = np.inf

		self.frente = list(self.laser_msg)[0:5] + list(self.laser_msg)[-5:]
		self.tras = self.laser_msg[173:184]

	def get_error(self):
		self.err = self.point.z - self.point.x
		self.twist.angular.z = self.err / self.kp

	def procura(self):
		if self.state != "ultimo_aruco":
			if self.pontos_aruco != -1:
				self.twist = Twist()
				self.robot_state = "aproxima"
			else:
				self.twist.angular.z = 0.2
		else:
			self.robot_state = "ultimo_aruco"

	def aproxima(self):
		self.point.x = self.pontos_aruco.x			
		self.get_error()
		# print(self.point.x)
		# print(self.twist.angular.z)
		self.twist.linear.x = 0.2
		if np.min(self.frente) < 0.3:
			self.robot_state = "derruba"
		elif self.point.x == -1:
			self.twist.linear.x = 0.0
			self.twist.angular.z = -0.2
	
	def derruba(self):
		self.twist = Twist()
		if self.state != "ultimo_aruco":
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			#levanta
			time.sleep(1.5)
			self.ombro.publish(1.0) # numeros postivos 
			time.sleep(1.5)
			#abaixa
			self.ombro.publish(-1.5) #numeros negativos
			time.sleep(1.5)
			if self.areas_aruco >14000:
				#levanta
				time.sleep(1.5)
				self.ombro.publish(1.0) # numeros postivos 
				time.sleep(1.5)
				#abaixa
				self.ombro.publish(-1.5) #numeros negativos
				time.sleep(1.5)
			if self.areas_aruco < 17000:
				self.contador += 1
			if self.contador == 5:
				self.state = "ultimo_aruco"
			else:
				self.robot_state = "procura"
		else:
			self.robot_state = "procura"

	def ultimo_aruco(self):
		self.point.x = self.pontos_aruco.x			
		self.get_error()
		# print(self.point.x)
		# print(self.twist.angular.z)
		self.twist.linear.x = 0.2
		if np.min(self.frente) < 0.5:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			#levanta
			time.sleep(1.5)
			self.ombro.publish(1.0) # numeros postivos 
			time.sleep(1.5)
			#abaixa
			self.ombro.publish(-1.5) #numeros negativos
			time.sleep(1.5)
			if self.areas_aruco >15000:
				#levanta
				time.sleep(1.5)
				self.ombro.publish(1.0) # numeros postivos 
				time.sleep(1.5)
				#abaixa
				self.ombro.publish(-1.5) #numeros negativos
				time.sleep(1.5)
			else:
				self.robot_state = "para"
	def para(self):
		if self.state == "ultimo_aruco":
			self.twist = Twist()
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
			self.garra.publish(0)
			rospy.sleep(1)
			self.ombro.publish(1.5)
			rospy.sleep(0.3)
			self.garra.publish(-1.5)
		self.robot_state = "fim"
	def fim(self):
		self.twist = Twist()
		print("FIM!")

	def control(self) -> None:
		'''
		Esta função é chamada pelo menos em {self.rate} Hz.
		Esta função controla o robô.
		'''
		print(self.robot_state)
		self.twist = Twist()
		print(f'self.robot_state: {self.robot_state}')
		print(self.contador)
		self.robot_machine[self.robot_state]()

		self.cmd_vel_pub.publish(self.twist)

		self.rate.sleep()


def main():
	rospy.init_node('q2')
	control = Questao2(ordem=[])
	# control = Questao2(ordem=['ciano', 'amarelo', 'verde']) # Video desafio 1
	# control = Questao2(ordem=['amarelo', 'ciano']) # Video desafio 2
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()
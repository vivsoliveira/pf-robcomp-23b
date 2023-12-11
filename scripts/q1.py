from typing import Self
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
class Questao1:
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz
      	self.color_param = {
            "pista_vermelha": {
                "lower": np.array([0, 10, 223],dtype=np.uint8),
                "upper": np.array([4, 255, 255],dtype=np.uint8)
            },"pista_verde": {
                "lower": np.array([71, 244, 155],dtype=np.uint8),
                "upper": np.array([78, 255, 255],dtype=np.uint8)
            }, "pista_azul": {
                "lower": np.array([230//2, 50, 50],dtype=np.uint8),
                "upper": np.array([250//2, 255, 255],dtype=np.uint8)
            }}
      	self.kernel = np.ones((5,5),np.uint8)
        # Subscribers
		self.bridge = CvBridge()
		self.laser_subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
		self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback, queue_size=1, buff_size = 2**24)
		# Publishers
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
		self.cmd_vel_pub.publish(Twist())
		# Maquina de estados
		self.selected_mod = None 
		self.robot_state = "anda"
		self.kp = 250
		self.state = 1
		self.robot_machine = {
			"anda": self.anda,
			"para": self.para,
			"checar": self.checar,
			"aproxima_pista": self.aproxima_pista,
			'roda_dir': self.rotate_dir,
		}

	def laser_callback(self, msg: LaserScan) -> None:
		self.laser_msg = np.array(msg.ranges).round(decimals=2)
		self.laser_msg[self.laser_msg == 0] = np.inf


		self.laser_forward = np.min(list(self.laser_msg[0:5]) + list(self.laser_msg[354:359]))
		self.laser_backwards = np.min(list(self.laser_msg[175:185]))

	def image_callback(self, msg: CompressedImage) -> None:
		"""
		Callback function for the image topic
		"""
		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
			hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
			img = cv_image.copy()
		except CvBridgeError as e:
			print(e)

		h, w, d = img.shape
		self.centro_segue = (h, 25*h//40)
        self.pista_r, area_p = self.color_segmentation(hsv, self.color_param["pista_vermelha"]["lower"], self.color_param["pista_vermelha"]["upper"])
        self.pista_g, area_p = self.color_segmentation(hsv, self.color_param["pista_verde"]["lower"], self.color_param["pista_verde"]["upper"])
		self.pista_b, area_p = self.color_segmentation(hsv, self.color_param["pista_inicial"]["lower"], self.color_param["pista_inicial"]["upper"])
		if self.state == "2":
			if self.pista_r[0] != 0:
				self.selected_mod = "pista_vermelha"
				self.robot_state = "checar"
        
		
	def color_segmentation(self, hsv: np.ndarray, lower_hsv: np.ndarray, upper_hsv: np.ndarray,) -> Point:
		""" 
		Use HSV color space to segment the image and find the center of the object.

		Args:
			bgr (np.ndarray): image in BGR format
		
		Returns:
			Point: x, y and area of the object
		"""
		point = []
		mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

		contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		maior_contorno = None
		maior_contorno_area = 0
		

		for cnt in contornos:
			area = cv2.contourArea(cnt)
			if area > maior_contorno_area:
				maior_contorno = cnt
				maior_contorno_area = area

		M = cv2.moments(maior_contorno)

		if M["m00"] == 0:
			point_x = 0
			point_y = 0
			point = [point_x,point_y]

		else:
			point_x = int(M["m10"] / M["m00"])
			point_y = int(M["m01"] / M["m00"])
			point = [point_x,point_y]

		return point, maior_contorno_area
	
    def rotate_direita(self) -> None:
        """
        Rotate the robot
        """
        self.twist = Twist()
        self.twist.angular.z = -0.5

    def rotate_esquerda(self) -> None:
        """
        Rotate the robot
        """
        self.twist = Twist()
        self.twist.angular.z = 0.5

	def anda(self):
        if self.laser_forward > 1.2:
            self.twist.linear.x = 0.2
        else:
            self.robot_state = "para"
	
	def para(self):
		self.twist = Twist()

    def checar(self) -> None:
        if self.selected_mod == "pista_vermelha":
            self.robot_state = "anda_pista_verm"
        elif self.selected_mod == "pista_verde":
            self.robot_state = "anda_pista_verde"
        elif self.selected_mod == "pista_azul":
            self.robot_state = "anda_pista_azul"	
			
	def anda_pista_verm(self) -> None:
        self.center_on_coord()

        self.twist.linear.x = 0.2

        if self.state == 2:
            if self.pista[0] == 0:
                self.robot_state = "para"
        elif self.state == 5: 
            if self.centro[self.n1][0] == 0:
                self.robot_state = "para"

	def center_on_coord(self):
		self.twist = Twist()

		err = 0
		if self.state == 1:
			err = self.centro_segue[0] - self.pista_r[0]

		self.twist.angular.z = (float(err)/self.kp)


	def aproxima_pista(self) -> None:
		self.center_on_coord()

		self.twist.linear.x = 0.2

		if self.state == 1:
			if self.pista_r[0] == 0:
				self.robot_state = "roda_dir"
			else:
				self.robot_state = "aproxima_pista"

	def control(self) -> None:
		'''
		Esta função é chamada pelo menos em {self.rate} Hz.
		Esta função controla o robô.
		'''
		
		self.twist = Twist()
		print(f'self.robot_state: {self.robot_state}')
		self.robot_machine[self.robot_state]()

		self.cmd_vel_pub.publish(self.twist)

		self.rate.sleep()

def main():
    rospy.init_node('q1')
    control = Questao1()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        control.control()

if __name__=="__main__":
    main()
#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

""" 
	Para inicializar o mapa, em um terminal digite:
	roslaunch my_simulation pista_circulos.launch

	Para executar, em um novo terminal digite:
	rosrun pf-robcomp-23b q1_des.py
"""

class Questao1Desafio:
	def __init__(self):

		self.rate = rospy.Rate(250) # 250 Hz
		
		# Subscribers

        # Publishers
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)


		# Maquina de estados
		self.robot_state = ""
		self.robot_machine = {
			"": self.nada
		}
	
	def nada(self):
		pass
	
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
	rospy.init_node('q1_des')
	control = Questao1Desafio()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()
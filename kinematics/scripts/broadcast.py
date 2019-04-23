#!/usr/bin/env python
import rospy 
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as n
import ikpy
from ikpy import plot_utils
import matplotlib.pyplot as plt
import copy
def rotM(q): 
	quat = copy.deepcopy(q)
	quat = quat/n.linalg.norm(quat)
	x,y,z,w = quat
	rM = n.zeros((3,3))
	rM[0,:] = [1 - 2*(y**2+z**2), 2*(x*y - z*w), 2*(x*z + y*w)]
	rM[1,:] = [2*(x*y + z*w), 1 - 2*(x**2+z**2), 2*(y*z - x*w)]
	rM[2,:] = [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2+y**2)]
	return rM


def IK(target_frame): 
	ax = plot_utils.init_3d_figure()
	
	
	my_chain =ikpy.chain.Chain.from_urdf_file("../../urdf_tutorial/urdf/01-myfirst.urdf")
	print(my_chain)
	jnts = my_chain.inverse_kinematics(target_frame)
	my_chain.plot(jnts, ax, target=target_frame[:3,3])
	plt.show()
	return jnts

def talker(): 
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rospy.init_node('joint_state_publisher')

	# tgt_ori = (-n.pi/2,0,-n.pi/2)
	#tgt_ori = (-n.pi/2,0,-n.pi/2)
	# ori_x = -n.pi/2
	# ori_y = 0
	# ori_z = -n.pi/2

	ori_x = 0
	ori_y = n.pi
	ori_z = n.pi/2


	target_vector = [0.03453803, 0.20218451, 0.16631325]
	target_frame = n.eye(4)
	rM = rotM(tf.transformations.quaternion_from_euler(ori_x, ori_y, ori_z))
	target_frame[0:3,0:3] = rM
	# target_frame[0,0:3] = [1,-0.0001,-0.0001]
	# target_frame[1,0:3] = [0, 0, -1]
	# target_frame[2,0:3] = [0, 1, 0]
	target_frame[:3, 3] = target_vector
	# print(target_frame)
	# jnts = IK(target_frame)
	# print(jnts)

	rate = rospy.Rate(10) # 10hz
	hello_str = JointState()
	hello_str.header = Header()
	hello_str.name=['dummy_to_base','dummy_to_elbow_1','elbow_1_to_elbow_2','elbow_2_to_elbow_3','elbow_3_to_end_eff']
	hello_str.position = [0.0129405,
0.167161,
2.3739,
-0.9318,
0.0155165
]

#[jnts[1], jnts[2], jnts[3], jnts[4],  0.0]#[ 0.0  ,  0.01260592, -0.56007798,  2.7924331 , -3.64144066,  0.0]# #
	
	hello_str.velocity = []
	hello_str.effort = []
	br = tf.TransformBroadcaster()

	


	while not rospy.is_shutdown():
		hello_str.header.stamp = rospy.Time.now()
		pub.publish(hello_str)
		br.sendTransform((target_vector[0], target_vector[1], target_vector[2]),
                         tf.transformations.quaternion_from_euler(ori_x, ori_y, ori_z),
                         rospy.Time.now(),
                         "pose",
                         "base_link")
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
#!/usr/bin/env python
# generate pursuer trajctory  

import rospy 
import numpy as np 
from geometry_msgs.msg import Pose, PoseArray
import tf

def cart2pol(point, center):
	x = point[0] - center[0]
	y = point[1] - center[1]
	r = np.sqrt(x*x + y*y)
	theta = np.atan2(y, x)
	return r, theta

def pol2cart(radius, theta, center):
	x_ = radius*np.cos(theta)
	y_ = radius*np.sin(theta)
	x = center[0] + x_
	y = center[1] + y_
	return x, y

trajfile = "/home/yun/vis-pe/traj/env1-1.txt"
# TODO:replace with ros param 
max_ang_rate = 0.5 # rad per sec
max_speed = 0.3 # m/s 
rate = 0.1

with open(trajfile) as f:
	lines = f.readlines()

traj = []
# represent each pose along traj as 
# (x, y, heading) heading in radians 
for line in lines:
	line = line.rstrip().split()
	if len(line) < 12: # same point as last point 
		pos = (float(line[1]), float(line[2]))
		des_ang = float(line[4])/180*np.pi

		if len(traj) > 0:
			last_ang = traj[-1][-1] 
			last_ang += (des_ang - last_ang)/abs(des_ang - last_ang)*max_ang_rate*rate
			while abs(last_ang - des_ang) > max_ang_rate*rate:
				traj.append((pos[0], pos[1], last_ang))
				last_ang += (des_ang - last_ang)/abs(des_ang - last_ang)*max_ang_rate*rate
		traj.append((pos[0], pos[1], des_ang))

	else: # also include movement ontop of rotation 
		ori = int(line[6]) # get orientation 
		des_pos = (float(line[1]), float(line[2]))
		des_ang = float(line[4])/180*np.pi
		current_pos = (traj[-1][0], traj[-1][1])
		current_ang = traj[-1][2]
		# differentiate between straight and curve line 
		if ori == 0: # straight line 
			# first get the direction vector 
			vect = [des_pos[0] - traj[-1][0], des_pos[1] - traj[-1][1]]
			# find magnitude 
			mag = np.sqrt(vect[0]*vect[0] + vect[1]*vect[1])
			vect = [vect[0]/mag, vect[1]/mag]
			numpts = int(mag/(max_speed*rate)) + 1
			# find the evenly spaced coordinates (poses)
			x_vals = np.linspace(current_pos[0], des_pos[0], numpts)
			y_vals = np.linspace(current_pos[1], des_pos[1], numpts)
			ang_vals = np.linspace(current_ang, des_ang, numpts)
		else:
			cent = (float(line[8]), float(line[9])) # center 
			# first convert to radian about center
			des_r, des_theta = cart2pol(des_pos, cent)
			curr_r, curr_theta = cart2pol(current_pos, cent)
			inc_ang = (max_speed*rate)/des_r
			# find the theta values (polar) based on inc_ang
			if ori == 1:
				if des_theta < curr_theta:
					des_theta += 2*np.pi
				numpts = int((des_theta - curr_theta)/inc_ang) + 1
				thet_vals = np.linspace(curr_theta, des_theta, numpts)
			if ori == -1:
				if des_theta > curr_theta:
					des_theta -= 2*np.pi
				numpts = int((curr_theta - des_theta)/inc_ang) + 1
				thet_vals = np.linspace(des_theta, curr_theta, numpts)
				thet_vals = thet_vals[::-1]

			# convert the thet vals to cartesian points 
			x_vals, y_vals = pol2cart(des_r, thet_vals, cent)
			ang_vals = np.linspace(current_ang, des_ang, numpts)

		for i in range(numpts):
			traj.append((x_vals[i], y_vals[i], ang_vals[i]))
			# add to traj 

rospy.init_node("traj_loader")
pub_traj = rospy.Publisher("trajectory", PoseArray, queue_size=3)

r = rospy.Rate(1)
for stupidleiyao in range(10): # publish for ten cycles
	traj_message = PoseArray()
	traj_message.header.frame_id = "/map"
	traj_message.header.stamp = rospy.get_rostime()
	for i in range(len(traj)):
		pose = Pose()
		pose.position.x = traj[i][0]
		pose.position.y = traj[i][1]
		# first convert to quaternion
		quaternion = tf.transformations.quaternion_from_euler(0, 0, traj[i][2])
		pose.orientation.x = quaternion[0]
		pose.orientation.y = quaternion[1]
		pose.orientation.z = quaternion[2]
		pose.orientation.w = quaternion[3]
		traj_message.poses.append(pose)

	pub_traj.publish(traj_message) # publish message
	r.sleep()




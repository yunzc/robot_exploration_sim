#!/usr/bin/env python

# create the yaml and image files for ros maps 
# for this to work, vertices must be arranged in a countercockwise manner 

import numpy as np 
from PIL import Image
import yaml

def orientation(pt1, pt2, pt3):
	# compute the orientation of three points 
	# return counterclock (-1) collinear (0) clockwise (1)
	# the three points represented as tuples
	# referenced from https://www.geeksforgeeks.org/orientation-3-ordered-points/
	val = (pt2[1] - pt1[1])*(pt3[0] - pt2[0]) - (pt2[0] - pt1[0])*(pt3[1] - pt2[1])
	if val == 0:
		return 0
	if val < 0:
		return 1 # counterclockwise 
	if val > 0:
		return -1 # clockwise

def onsegment(seg, pt):
	# given a segment and pt check if pt on seg 
	if orientation(seg[0], seg[1], pt) != 0:
		return False # can't be on if not colinear 
	p = seg[0]
	q = seg[1]
	if pt[0] <= max(p[0], q[0]) and pt[0] >= min(p[0], q[0]) and \
				pt[1] <= max(p[1], q[1]) and pt[1] >= min(p[1], q[1]):
		return True
	return False

def dosegsintersect(seg1, seg2):
	# https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
	# segments represented as a tuple of two endpoints 
	p1 = seg1[0]
	q1 = seg1[1]
	p2 = seg2[0]
	q2 = seg2[1]
	o1 = orientation(p1, q1, p2)
	o2 = orientation(p1, q1, q2)
	o3 = orientation(p2, q2, p1)
	o4 = orientation(p2, q2, q1)
	# print("de", seg1, seg2, o1, o2, o3, o4)
	# general case 
	if o1 != o2 and o3 != o4:
		return 1
	# special cases (ignore, count these as non intersecting for our purposes )
	# 1. p1, q1, p2 colinear and p2 lies on p1q1
	if o1 == 0 and onsegment(seg1, p2):
		return 1
	# 2. p1, q1, q2 colinear and q2 lies on p1q1
	if o2 == 0 and onsegment(seg1, q2):
		return 1
	# 3. p2, q2, p1 colinear and p1 lies on p2q2
	if o3 == 0 and onsegment(seg2, p1):
		return 1
	# 4. p2, q2, q1 colinear and q1 lies on p2q2
	if o4 == 0 and onsegment(seg2, q1):
		return 1
	return 0

def pt_in_polygon(pt, vertices):
	# check if pt in polygon 
	# https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
	# pt represented as a tuple
	# vertices are the list of points that made up the polygon 
	intersections1 = 0
	intersections2 = 0
	for i in range(len(vertices)):
		i_next = i + 1 
		if i_next == len(vertices):
			i_next = 0 # wrap back 
		segment = (vertices[i], vertices[i_next])
		pt_seg1 = (pt, (pt[0] + 99999, pt[1])) # horizontal segment 
		pt_seg2 = (pt, (pt[0] - 99999, pt[1]))
		intersections1 += dosegsintersect(segment, pt_seg1)
		intersections2 += dosegsintersect(segment, pt_seg2)
	if intersections1 % 2 == 0 and intersections2 % 2 == 0:
		return 0
	else:
		return 1

if __name__ == "__main__":
	# datafiles = ["/home/yun/vis-pe/env/env1.txt", "/home/yun/vis-pe/env/env2.txt"]
	datafiles = ["/home/yun/vis-pe/env/testenv2.txt"]
	# datafile encodes the coordinates of the vertices of the polygon 
	# that describes the environement  
	# an environement with holes is described by many polygons 
	resolution = 0.01
	yaml_file = "../maps/env2.yaml"
	img_file = "../maps/env2.png"

	max_x = 0
	min_x = 0
	max_y = 0
	min_y = 0

	polygons = []
	for i in range(len(datafiles)):
		with open(datafiles[i]) as f:
			lines = f.readlines()

		vertices = []
		for line in lines:
			line = line.rstrip().split()
			if len(line) == 2:
				try:
					x = float(line[0])
					y = float(line[1])
					if x > max_x:
						max_x = x
					elif x < min_x:
						min_x = x
					if y > max_y:
						max_y = y
					elif y < min_y:
						min_y = y
					vertices.append((float(line[0]), float(line[1])))
				except:
					pass
		polygons.append(vertices)

	width = int((max_x - min_x)/resolution)
	height = int((max_y - min_y)/resolution)
	array = np.zeros((width, height))
	for i in range(width):
		for j in range(height):
			pt = (i*resolution, (height-j)*resolution)
			val = 0
			for k in range(len(polygons)):
				val += pt_in_polygon(pt, polygons[k])
			array[i,j] = val*255

	array = array.astype(np.uint8).transpose()
	img = Image.fromarray(array)
	img.save(img_file)

	img_file = img_file.split('/')[-1]
	mapinfo = dict(
		image = img_file,
		resolution = resolution,
		origin = [0.0, 0.0, 0.0],
		occupied_thresh = 0.65,
		free_thresh = 0.196,
		negate = 0)

	with open(yaml_file, 'w') as outfile:
		yaml.dump(mapinfo, outfile, default_flow_style=False)

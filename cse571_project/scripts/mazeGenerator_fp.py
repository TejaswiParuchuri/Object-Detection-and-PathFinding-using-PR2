from collections import defaultdict
import numpy as np
import argparse
import random
import pprint
import json
import copy

class Maze:

	def __init__(self, grid_dimension, myscale=0.5):
		self.grid_dimension = grid_dimension
		self.grid_start = 0
		self.myscale = myscale

	def __deepcopy__(self, memodict={}):
		new_maze = Maze(self.grid_dimension)
		return new_maze

	def copy_empty_world(self,root_path):
		f_in = open(root_path+'/worlds/empty_world.sdf', 'r')
		f_out = open(root_path+'/worlds/maze.sdf', 'w')
		for line in f_in:
			f_out.write(line)
		f_in.close()
		return f_out

	def add_walls_description(self,f_out):
		for i in range(1, 5):
			f_out.write('<model name=\'wall{}\'>\n'.format(i))
			f_out.write('<static>1</static>\n<link name=\'link\'>\n<pose frame=\'\'>0 0 0.42 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n')
			f_out.write('</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
			f_out.write('<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n</geometry>\n<material>\n<script>\n')
			f_out.write('<uri>model://grey_wall/materials/scripts</uri>\n<uri>model://grey_wall/materials/textures</uri>\n<name>vrc/grey_wall</name>\n</script>\n</material>\n</visual>\n<self_collide>0</self_collide>\n')
			f_out.write('<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>-0.779308 4.01849 0 0 -0 0</pose>\n</model>\n')

	def add_walls(self,f_out, length):
		scale = (length+2)/7.5
		wall_dimensions = [(-1, length/2, -1.55905, scale, 1), (length/2, length+1, 0, scale, 1), (length+1, length/2, -1.55905, scale, 1), (length/2, -1, 0, scale, 1)]
		for i in range(4):
			f_out.write('<model name=\'wall{}\'>\n'.format(i+1))
			f_out.write('<pose frame=\'\'>{} {} 0 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			f_out.write('<scale>{} {} 0.03</scale>\n'.format(wall_dimensions[i][3], wall_dimensions[i][4]))
			f_out.write('<link name=\'link\'>\n')
			f_out.write('<pose frame=\'\'>{} {} 0.42 -0 0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')

	def generate_blocked_edges(self, seed, root_path):
		object_dict = {}
		np.random.seed(seed)
		list_of_list_of_coords = []
		f_out = self.copy_empty_world(root_path)
		self.add_walls(f_out, self.grid_dimension*self.myscale)

		f_out.write('</state>')

		self.add_walls_description(f_out)
		#color list will decide color of the book. R, G, B, X(need to check)
		
		f_out.write('</world>\n</sdf>')
		f_out.close()

	 	return object_dict


if __name__ == "__main__":	
	grid_size = 6

	root_path = "/home/ketan/catkin_ws/src/search"

	books, mazeInfo = generate_blocked_edges(grid_size)

	# pprint.pprint(books)
	# print(mazeInfo)
from collections import defaultdict
import numpy as np
import argparse
import random
import pprint
import json
import copy
import math

class Maze:

	def __init__(self, grid_dimension, myscale=0.5):
		self.walls_list=[]
		self.boundary_list=[]
		self.coke_list=[]
		self.cup_list=[]
		self.bin_list=[]
		self.bin_loc=[]
		self.grid_dimension = grid_dimension
		self.grid_start = 0
		self.myscale = myscale
		self.blocked_edges = set()

	def __deepcopy__(self, memodict={}):
		new_maze = Maze(self.grid_dimension)
		new_maze.blocked_edges = copy.deepcopy(self.blocked_edges)
		return new_maze

	def copy_empty_world(self,root_path):
		f_in = open(root_path+'/worlds/empty_world.sdf', 'r')
		f_out = open(root_path+'/worlds/maze.sdf', 'w')
		for line in f_in:
			f_out.write(line)
		f_in.close()
		return f_out

	def add_walls_description(self,f_out):
		for i in range(4):
			f_out.write("<link name='Wall_{}'>\n<pose frame=''>{} {} {} {} {} {}</pose>\n<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n       <wrench>0 0 0 0 -0 0</wrench>\n</link>".format(i,self.boundary_list[i][0],self.boundary_list[i][1],self.boundary_list[i][2],self.boundary_list[i][3],self.boundary_list[i][4],self.boundary_list[i][5]))
		j=i+1
		for i in range(len(self.walls_list)):
			f_out.write(" <link name='Wall_{}'>\n<pose frame=''>{} {} {} {} {} {}</pose>\n<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n       <wrench>0 0 0 0 -0 0</wrench>\n</link>".format(i+j,self.walls_list[i][0],self.walls_list[i][1],self.walls_list[i][2],self.walls_list[i][3],self.walls_list[i][4],self.walls_list[i][5]))

		f_out.write("<model name='pr2_bin'>\n        <pose frame=''>{} {} {} {} {} {}</pose>\n        <scale>0.3 0.3 0.4</scale>\n        <link name='link'>\n          <pose frame=''>{} {} {} {} {} {}</pose>\n          <velocity>0 0 0 0 -0 0</velocity>\n          <acceleration>0 0 0 0 -0 0</acceleration>\n          <wrench>0 0 0 0 -0 0</wrench>\n        </link>\n      </model>".format(self.bin_list[0][0],self.bin_list[0][1],self.bin_list[0][2],self.bin_list[0][3],self.bin_list[0][4],self.bin_list[0][5],self.bin_list[0][0],self.bin_list[0][1],self.bin_list[0][2],self.bin_list[0][3],self.bin_list[0][4],self.bin_list[0][5]))
		f_out.write("</model>\n<light name='sun'>\n<pose frame=''>0 0 10 0 -0 0</pose>\n</light>\n</state>\n</world>\n</sdf>")
	
	def add_walls(self,f_out, length):
		for i in range(4):
			f_out.write("\n<link name='Wall_{}'>\n<collision name='Wall_{}_Collision'>\n<geometry>\n<box>\n<size>{} 0.15 0.5</size>\n</box>\n</geometry>\n<pose frame=''>0 0 0.25 0 -0 0</pose>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n<visual name='Wall_{}_Visual'>\n<pose frame=''>0 0 0.25 0 -0 0</pose>\n<geometry>\n<box>\n<size>{} 0.15 0.5</size>\n</box>\n</geometry>\n<material>\n<script>\n<uri>file://media/materials/scripts/gazebo.material</uri>\n<name>Gazebo/Wood</name>\n</script>\n<ambient>1 1 1 1</ambient>\n</material>\n</visual>\n<pose frame=''>{} {} {} {} {} {}</pose>\n<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n".format(i,i,length*2,i,length*2,self.boundary_list[i][0],self.boundary_list[i][1],self.boundary_list[i][2],self.boundary_list[i][3],self.boundary_list[i][4],self.boundary_list[i][5]))
			#f_out.write(string_write)
		j=i+1
		for i in range(len(self.walls_list)):
			f_out.write("\n<link name='Wall_{}'>\n<collision name='Wall_{}_Collision'>\n<geometry>\n<box>\n<size>{} 0.15 0.5</size>\n</box>\n</geometry>\n<pose frame=''>0 0 0.25 0 -0 0</pose>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n<visual name='Wall_{}_Visual'>\n<pose frame=''>0 0 0.25 0 -0 0</pose>\n<geometry>\n<box>\n<size>{} 0.15 0.5</size>\n</box>\n</geometry>\n<material>\n<script>\n<uri>file://media/materials/scripts/gazebo.material</uri>\n<name>Gazebo/Wood</name>\n</script>\n<ambient>1 1 1 1</ambient>\n</material>\n</visual>\n<pose frame=''>{} {} {} {} {} {}</pose>\n<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n".format(i+j,i+j,length*2*0.3,i+j,length*2*0.3,self.walls_list[i][0],self.walls_list[i][1],self.walls_list[i][2],self.walls_list[i][3],self.walls_list[i][4],self.walls_list[i][5]))
			#f_out.write(string_write)
		f_out.write("<static>1</static>\n</model>\n")
		#f_out.write(string_write)
		self.add_cans(f_out)
	
	def add_cans_description(self,f_out):
		for i in range(len(self.coke_list)):
			f_out.write("<model name='coke_can{}'>\n        <pose frame=''>{} {} {} {} {} {}</pose>\n        <scale>1 1 1</scale>\n        <link name='link'>\n          <pose frame=''>{} {} {} {} {} {}</pose>\n          <velocity>-7.4e-05 0.000547 0.002754 0.209596 0.022304 0.003201</velocity>\n          <acceleration>-0.096311 -0.13049 3.57572 -2.45192 0.965952 -2.20851</acceleration>\n          <wrench>-0.037561 -0.050891 1.39453 0 -0 0</wrench>\n        </link>\n      </model>".format(i,self.coke_list[i][0],self.coke_list[i][1],self.coke_list[i][2],self.coke_list[i][3],self.coke_list[i][4],self.coke_list[i][5],self.coke_list[i][0],self.coke_list[i][1],self.coke_list[i][2],self.coke_list[i][3],self.coke_list[i][4],self.coke_list[i][5]))
		

		f_out.write("<model name='ground_plane'>\n        <pose frame=''>0 0 0 0 -0 0</pose>\n        <scale>1 1 1</scale>\n        <link name='link'>\n          <pose frame=''>0 0 0 0 -0 0</pose>\n          <velocity>0 0 0 0 -0 0</velocity>\n          <acceleration>0 0 0 0 -0 0</acceleration>\n          <wrench>0 0 0 0 -0 0</wrench>\n        </link>\n      </model>\n")
		

		self.add_cups_description(f_out)
	#change in dimenssion of the book is handled in this function.




	def add_cans(self,f_out):
		for i in range(len(self.coke_list)):
			f_out.write("<model name='coke_can{}'>\n	<link name='link'>\n        <inertial>\n          <mass>0.60</mass>\n          <inertia>\n            <ixx>0.00055575</ixx>\n            <ixy>0</ixy>\n            <ixz>0</ixz>\n            <iyy>0.00055575</iyy>\n            <iyz>0</iyz>\n            <izz>0.0001755</izz>\n          </inertia>\n        </inertial>\n        <collision name='collision'>\n          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>\n          <geometry>\n            <mesh>\n              <uri>model://coke_can/meshes/coke_can.dae</uri>\n              <scale>1 1 1</scale>\n            </mesh>\n          </geometry>\n          <max_contacts>10</max_contacts>\n          <surface>\n            <contact>\n              <ode/>\n            </contact>\n            <bounce/>\n            <friction>\n              <torsional>\n                <ode/>\n              </torsional>\n              <ode/>\n            </friction>\n          </surface>\n        </collision>\n        <visual name='visual'>\n          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>\n          <geometry>\n            <mesh>\n              <uri>model://coke_can/meshes/coke_can.dae</uri>\n              <scale>2 2 1</scale>\n            </mesh>\n          </geometry>\n        </visual>\n        <self_collide>0</self_collide>\n        <kinematic>0</kinematic>\n        <gravity>1</gravity>\n      </link>\n      <pose frame=''>{} {} {} {} {} {}</pose>\n    </model>".format(i,self.coke_list[i][0],self.coke_list[i][1],self.coke_list[i][2],self.coke_list[i][3],self.coke_list[i][4],self.coke_list[i][5]))
		self.add_cups(f_out)
		f_out.write("<state world_name='default'>\n<sim_time>258 883000000</sim_time>\n<real_time>246 336670878</real_time>\n<wall_time>1573523046 957349641</wall_time>\n<iterations>123860</iterations>\n<model name='ground_plane'>\n<pose frame=''>0 0 0 0 -0 0</pose>\n<scale>1 1 1</scale>\n<link name='link'>\n<pose frame=''>0 0 0 0 -0 0</pose>\n<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n<model name='turtlebot3_plaza'>\n<pose frame=''>0.34992 -0.526986 0 0 -0 0</pose>\n    <scale>1 1 1</scale>\n")
	
	def add_cups_description(self,f_out):
		 for i in range(len(self.cup_list)):
			f_out.write("<model name='plastic_cup{}'>\n        <pose frame=''>{} {} {} {} {} {}</pose>\n        <scale>1 1 1</scale>\n        <link name='link'>\n          <pose frame=''>{} {} {} {} {} {}</pose>\n          <velocity>0 0 0 0 -0 0</velocity>\n          <acceleration>0 0 0 0 -0 0</acceleration>\n          <wrench>0 0 0 0 -0 0</wrench>\n        </link>\n      </model>".format(i,self.cup_list[i][0],self.cup_list[i][1],self.cup_list[i][2],self.cup_list[i][3],self.cup_list[i][4],self.cup_list[i][5],self.cup_list[i][0],self.cup_list[i][1],self.cup_list[i][2],self.cup_list[i][3],self.cup_list[i][4],self.cup_list[i][5]))

	def add_cups(self,f_out):
		for i in range(len(self.cup_list)):
			f_out.write("<model name='plastic_cup{}'>\n      <link name='link'>\n        <pose frame=''>0 0 0.065 0 -0 0</pose>\n        <inertial>\n          <mass>0.0599</mass>\n          <inertia>\n            <ixx>0</ixx>\n            <ixy>0</ixy>\n            <ixz>0</ixz>\n            <iyy>0</iyy>\n            <iyz>0</iyz>\n            <izz>0</izz>\n          </inertia>\n        </inertial>\n        <collision name='collision'>\n          <geometry>\n            <mesh>\n              <uri>model://plastic_cup/meshes/plastic_cup.dae</uri>\n              <scale>1 1 1</scale>\n            </mesh>\n          </geometry>\n          <surface>\n            <contact>\n              <poissons_ratio>0.35</poissons_ratio>\n              <elastic_modulus>3.10264e+09</elastic_modulus>\n              <ode>\n                <kp>100000</kp>\n                <kd>100</kd>\n                <max_vel>100</max_vel>\n                <min_depth>0.001</min_depth>\n              </ode>\n            </contact>\n            <friction>\n              <torsional>\n                <coefficient>1</coefficient>\n                <use_patch_radius>0</use_patch_radius>\n                <surface_radius>0.01</surface_radius>\n                <ode/>\n              </torsional>\n              <ode/>\n            </friction>\n            <bounce/>\n          </surface>\n          <max_contacts>10</max_contacts>\n        </collision>\n        <visual name='visual'>\n          <geometry>\n            <mesh>\n              <uri>model://plastic_cup/meshes/plastic_cup.dae</uri>\n              <scale>2 2 1</scale>\n            </mesh>\n          </geometry>\n          <material>\n            <script>\n              <uri>file://media/materials/scripts/gazebo.material</uri>\n              <name>Gazebo/SkyBlue</name>\n            </script>\n          </material>\n        </visual>\n        <self_collide>0</self_collide>\n        <kinematic>0</kinematic>\n        <gravity>1</gravity>\n      </link>\n      <pose frame=''>{} {} {} {} {} {}</pose>\n    </model>".format(i,self.cup_list[i][0],self.cup_list[i][1],self.cup_list[i][2],self.cup_list[i][3],self.cup_list[i][4],self.cup_list[i][5]))

		f_out.write("<model name='pr2_bin'>\n      <link name='link'>\n        <pose frame=''>0 0 0 0 -0 0</pose>\n        <self_collide>0</self_collide>\n        <kinematic>0</kinematic>\n        <gravity>1</gravity>\n        <inertial>\n          <mass>1</mass>\n          <pose frame=''>0 0 0 0 -0 0</pose>\n          <inertia>\n            <ixx>1</ixx>\n            <ixy>0</ixy>\n            <ixz>0</ixz>\n            <iyy>1</iyy>\n            <iyz>0</iyz>\n            <izz>1</izz>\n          </inertia>\n        </inertial>\n        <visual name='visual'>\n          <geometry>\n            <mesh>\n              <uri>model://bookcart/meshes/bookcart.dae</uri>\n              <scale>{} {} 1</scale>\n            </mesh>\n          </geometry>\n          <pose frame=''>0 0 0 0 -0 0</pose>\n          <cast_shadows>1</cast_shadows>\n          <transparency>0</transparency>\n          <material>\n            <shader type='vertex'>\n              <normal_map>__default__</normal_map>\n            </shader>\n          </material>\n        </visual>\n      </link>\n      <static>1</static>\n      <allow_auto_disable>1</allow_auto_disable>\n      <pose frame=''>{} {} {} {} {} {}</pose>\n    </model>".format(self.grid_dimension*self.myscale*0.1,self.grid_dimension*self.myscale*0.1,self.bin_list[0][0],self.bin_list[0][1],self.bin_list[0][2],self.bin_list[0][3],self.bin_list[0][4],self.bin_list[0][5]))
		return


	def can_dict_generator(self,cans,counter,location, coord1, cord2):
		cans["can_"+str(counter)]["loc"]= location
		cans["can_"+str(counter)]["load_loc"] = []
		cans["can_"+str(counter)]["load_loc"].append(coord1)
		cans["can_"+str(counter)]["load_loc"].append(cord2)
	def cup_dict_generator(self,cups, counter, location, coord1, cord2):
		cups["cup_"+str(counter)]["loc"]= location
		cups["cup_"+str(counter)]["load_loc"] = []
		cups["cup_"+str(counter)]["load_loc"].append(coord1)
		cups["cup_"+str(counter)]["load_loc"].append(cord2)

	def bin_dict_generator(self,bins, access_loc_list, location):
		bins["bin"]["loc"]= location
		bins["bin"]["load_loc"] = access_loc_list

	def generate_blocked_edges(self, list_of_number_of_cans, list_of_number_of_cups,seed, root_path):
		object_dict = {}
		cans = {}
		cups={}
		bins={}
		can_loadloc=[]
		cup_loadloc=[]
		np.random.seed(seed)
		f_out = self.copy_empty_world(root_path)
		dimension=self.grid_dimension*self.myscale
		self.boundary_list.append((-dimension,0,0,0,-0,1.5708))
		self.boundary_list.append((0,dimension-0.08,0,0,-0,0))
		self.boundary_list.append((dimension,0,0,0,-0,1.5708))
		self.boundary_list.append((0,-dimension+0.08,0,0,-0,3.14159))
		limit=int(math.ceil(dimension*2))
		for i in range(limit):
			self.blocked_edges.add((-dimension,0+i*self.myscale,-dimension,0.5+i*self.myscale))
			self.blocked_edges.add((-dimension,0-i*self.myscale,-dimension,-0.5-i*self.myscale))
			self.blocked_edges.add((-dimension+0.5,0.5+i*self.myscale,-dimension,0.5+i*self.myscale))
			self.blocked_edges.add((-dimension+0.5,-0.5-i*self.myscale,-dimension,-0.5-i*self.myscale))
			self.blocked_edges.add((0+i*self.myscale,dimension,0.5+i*self.myscale,dimension))
			self.blocked_edges.add((0-i*self.myscale,dimension,-0.5-i*self.myscale,dimension))
			self.blocked_edges.add((0.5+i*self.myscale,dimension-0.5,0.5+i*self.myscale,dimension))
			self.blocked_edges.add((-0.5-i*self.myscale,dimension-0.5,-0.5-i*self.myscale,dimension))
			self.blocked_edges.add((dimension,0+i*self.myscale,dimension,0.5+i*self.myscale))
			self.blocked_edges.add((dimension,0-i*self.myscale,dimension,-0.5-i*self.myscale))
			self.blocked_edges.add((dimension-0.5,0.5+i*self.myscale,dimension,0.5+i*self.myscale))
			self.blocked_edges.add((dimension-0.5,-0.5-i*self.myscale,dimension,-0.5-i*self.myscale))
			self.blocked_edges.add((0+i*self.myscale,-dimension,0.5+i*self.myscale,-dimension))
			self.blocked_edges.add((0-i*self.myscale,-dimension,-0.5-i*self.myscale,-dimension))
			self.blocked_edges.add((0.5+i*self.myscale,-dimension+0.5,0.5+i*self.myscale,-dimension))
			self.blocked_edges.add((-0.5-i*self.myscale,-dimension+0.5,-0.5-i*self.myscale,-dimension))
		self.walls_list.append((-dimension+dimension*0.3,-dimension+dimension*0.3,0,0,-0,0))
		self.walls_list.append((-0.12992,-dimension+dimension*0.3,0,0,0,-1.5708))
		self.walls_list.append((dimension-dimension*0.48,-dimension+dimension*0.6,0,0,-0,1.5708))
		self.walls_list.append((dimension-dimension*0.90,dimension-dimension*0.85,0,0,0,-1.5708))
		self.walls_list.append((dimension-dimension*0.3 ,dimension-dimension*0.80,0,0,-0,0))
		self.walls_list.append((dimension-dimension*0.45,dimension-dimension*0.30,0,0,0,-1.57080))
		self.walls_list.append((-dimension+dimension*0.6,dimension-dimension*0.30,0,0,-0,0))
		self.walls_list.append((-dimension+dimension*0.3,-dimension+dimension*0.9,0,0,0,-1.5708))
		self.bin_list.append((dimension-self.grid_dimension*self.myscale*0.23,-dimension+0.3,0,0,0,0))
		limit=int(math.ceil(dimension*2*0.3))
		x_binedge=dimension-self.grid_dimension*self.myscale*0.23-(dimension-self.grid_dimension*self.myscale*0.23)%0.5
		y_binedge=-dimension+0.3+self.grid_dimension*self.myscale*0.2
		y_binedge_temp=y_binedge
		x_binedge_temp=x_binedge
		while(y_binedge>-dimension):
			self.bin_loc.append((x_binedge,y_binedge))
			y_binedge-=self.myscale
		x_binedge+=self.myscale
		while(x_binedge<dimension):
			self.bin_loc.append((x_binedge,y_binedge_temp))
			x_binedge+=self.myscale
		#print self.bin_loc
		bins["bin"]={}
		self.bin_dict_generator(bins,self.bin_loc,(dimension-self.grid_dimension*self.myscale*0.23,-dimension+0.3))
		for i in self.bin_loc:
			x=i[0]
			y=i[1]
			while(1):
				if(x==x_binedge_temp and y==y_binedge_temp):
					break
				elif(i[0]==y_binedge_temp):
					if(y>-dimension):
						self.blocked_edges.add((x,y,x,y+self.myscale))
						y+=self.myscale
					else:
						break
				else:
					if(x<dimension):
						self.blocked_edges.add((x,y,x+self.myscale,y))
						x+=self.myscale
					else:
						break
			
		for i in range(0,limit):
			self.blocked_edges.add((-dimension+dimension*0.3-((-dimension+dimension*0.3)%0.5)+(i+1)*self.myscale,-dimension+dimension*0.3-((-dimension+dimension*0.3)%0.5),-dimension+dimension*0.3-((-dimension+dimension*0.3)%0.5)+(i+1)*self.myscale,-dimension+dimension*0.3+0.5-((-dimension+dimension*0.3)%0.5)))
			self.blocked_edges.add((-dimension+dimension*0.3+0.5-((-dimension+dimension*0.3)%0.5)-(i+1)*self.myscale,-dimension+dimension*0.3-((-dimension+dimension*0.3)%0.5),-dimension+dimension*0.3+0.5-((-dimension+dimension*0.3)%0.5)-(i+1)*self.myscale,-dimension+dimension*0.3+0.5-((-dimension+dimension*0.3)%0.5)))
			self.blocked_edges.add((0.0,-dimension+dimension*0.3-((-dimension+dimension*0.3)%0.5)+(i+1)*self.myscale,-0.5,-dimension+dimension*0.3-((-dimension+dimension*0.3)%0.5)+(i+1)*self.myscale))
			self.blocked_edges.add((0.0,-dimension+dimension*0.3+0.5-((-dimension+dimension*0.3)%0.5)-(i+1)*self.myscale,-0.5,-dimension+dimension*0.3+0.5-((-dimension+dimension*0.3)%0.5)-(i+1)*self.myscale))
			self.blocked_edges.add((dimension-dimension*0.48+0.5-((dimension-dimension*0.48)%0.5),-dimension+dimension*0.6-((-dimension+dimension*0.6)%0.5)+(i+1)*self.myscale,dimension-dimension*0.48-((dimension-dimension*0.48)%0.5),-dimension+dimension*0.6-((-dimension+dimension*0.6)%0.5)+(i+1)*self.myscale))
			self.blocked_edges.add((dimension-dimension*0.48+0.5-((dimension-dimension*0.48)%0.5),-dimension+dimension*0.6+0.5-((-dimension+dimension*0.6)%0.5)-(i+1)*self.myscale,dimension-dimension*0.48-((dimension-dimension*0.48)%0.5),-dimension+dimension*0.6+0.5-((-dimension+dimension*0.6)%0.5)-(i+1)*self.myscale))
			self.blocked_edges.add((dimension-dimension*0.90+0.5-((dimension-dimension*0.90)%0.5),dimension-dimension*0.85-((dimension-dimension*0.85)%0.5)+(i+1)*self.myscale,dimension-dimension*0.90-((dimension-dimension*0.90)%0.5),dimension-dimension*0.85-((dimension-dimension*0.85)%0.5)+(i+1)*self.myscale))
			self.blocked_edges.add((dimension-dimension*0.90+0.5-((dimension-dimension*0.90)%0.5),dimension-dimension*0.85+0.5-((dimension-dimension*0.85)%0.5)-(i+1)*self.myscale,dimension-dimension*0.90-((dimension-dimension*0.90)%0.5),dimension-dimension*0.85+0.5-((dimension-dimension*0.85)%0.5)-(i+1)*self.myscale))
			self.blocked_edges.add((dimension-dimension*0.3-((dimension-dimension*0.3)%0.5)+(i+1)*self.myscale,dimension-dimension*0.80-((dimension-dimension*0.80)%0.5),dimension-dimension*0.3-((dimension-dimension*0.3)%0.5)+(i+1)*self.myscale,dimension-dimension*0.80+0.5-((dimension-dimension*0.80)%0.5)))
			self.blocked_edges.add((dimension-dimension*0.3+0.5-((dimension-dimension*0.3)%0.5)-(i+1)*self.myscale,dimension-dimension*0.80-((dimension-dimension*0.80)%0.5),dimension-dimension*0.3+0.5-((dimension-dimension*0.3)%0.5)-(i+1)*self.myscale,dimension-dimension*0.80+0.5-((dimension-dimension*0.80)%0.5)))	
			self.blocked_edges.add((dimension-dimension*0.45+0.5-((dimension-dimension*0.45)%0.5),dimension-dimension*0.30-((dimension-dimension*0.30)%0.5)+(i+1)*self.myscale,dimension-dimension*0.45-((dimension-dimension*0.45)%0.5),dimension-dimension*0.30-((dimension-dimension*0.30)%0.5)+(i+1)*self.myscale))
			self.blocked_edges.add((dimension-dimension*0.45+0.5-((dimension-dimension*0.45)%0.5),dimension-dimension*0.30+0.5-((dimension-dimension*0.30)%0.5)-(i+1)*self.myscale,dimension-dimension*0.45-((dimension-dimension*0.45)%0.5),dimension-dimension*0.30+0.5-((dimension-dimension*0.30)%0.5)-(i+1)*self.myscale))
			self.blocked_edges.add((-dimension+dimension*0.6-((-dimension+dimension*0.6)%0.5)+(i+1)*self.myscale,dimension-dimension*0.30-((dimension-dimension*0.30)%0.5),-dimension+dimension*0.6-((-dimension+dimension*0.6)%0.5)+(i+1)*self.myscale,dimension-dimension*0.30+0.5-((dimension-dimension*0.30)%0.5)))
			self.blocked_edges.add((-dimension+dimension*0.6+0.5-((-dimension+dimension*0.6)%0.5)-(i+1)*self.myscale,dimension-dimension*0.30-((dimension-dimension*0.30)%0.5),-dimension+dimension*0.6+0.5-((-dimension+dimension*0.6)%0.5)-(i+1)*self.myscale,dimension-dimension*0.30+0.5-((dimension-dimension*0.30)%0.5)))
			self.blocked_edges.add((-dimension+dimension*0.3+0.5-((-dimension+dimension*0.3)%0.5),-dimension+dimension*0.9-((-dimension+dimension*0.9)%0.5)+(i+1)*self.myscale,-dimension+dimension*0.3-((-dimension+dimension*0.3)%0.5),-dimension+dimension*0.9-((-dimension+dimension*0.9)%0.5)+(i+1)*self.myscale))
			self.blocked_edges.add((-dimension+dimension*0.3+0.5-((-dimension+dimension*0.3)%0.5),-dimension+dimension*0.9+0.5-((-dimension+dimension*0.9)%0.5)-(i+1)*self.myscale,-dimension+dimension*0.3-((-dimension+dimension*0.3)%0.5),-dimension+dimension*0.9+0.5-((-dimension+dimension*0.9)%0.5)-(i+1)*self.myscale))
		count=0
		while count<list_of_number_of_cans:
			x = self.myscale*np.random.randint(0, (self.grid_dimension+1))
			y = self.myscale*np.random.randint(0, (self.grid_dimension+1))
			x_sign=random.choice(['-','+'])
			y_sign=random.choice(['+','-'])
			offset_y=-self.myscale/4
			offset_x=-self.myscale/4
			if(x_sign=='-'):
				x=x*-1
				offset_x=self.myscale/4
			if(y_sign=='-'):
				y=y*-1
				offset_y=self.myscale/4
			if(count%2==0):
				if(x<0):
					x1=x+self.myscale
				else:
					x1=x-self.myscale
				y1=y
				
				if (((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0) not in self.coke_list) and ((x,y,x1,y1) not in self.blocked_edges) and ((x1,y1,x,y) not in self.blocked_edges) and (x>=1.0 or x<=-1.0) and (y>=1.0 or y<=-1.0) and ((x,y) not in can_loadloc) and ((x1,y1) not in can_loadloc) and ((x,y) not in self.bin_loc) and ((x1,y1) not in self.bin_loc):
					self.coke_list.append(((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0))
					cans["can_"+str(count)]={}
					self.can_dict_generator(cans,count,((x+x1+offset_x)/2,(y+y1+offset_y)/2),(x,y),(x1,y1))
					count=count+1
					can_loadloc.append((x,y))
					can_loadloc.append((x1,y1))
					self.blocked_edges.add((x,y,x1,y1))
			else:
				if(y<0):
					y1=y+self.myscale
					
				else:
					y1=y-self.myscale
				x1=x
				if (((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0) not in self.coke_list) and ((x,y,x1,y1) not in self.blocked_edges) and ((x1,y1,x,y) not in self.blocked_edges) and (x>=1.0 or x<=-1.0) and (y>=1.0 or y<=-1.0) and ((x,y) not in self.bin_loc) and ((x1,y1) not in self.bin_loc):
					self.coke_list.append(((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0))
					cans["can_"+str(count)]={}
					self.can_dict_generator(cans,count,((x+x1+offset_x)/2,(y+y1+offset_y)/2),(x,y),(x1,y1))
					count=count+1
					can_loadloc.append((x,y))
					can_loadloc.append((x1,y1))
					self.blocked_edges.add((x,y,x1,y1))
		count=0
		while count<list_of_number_of_cups:
			x = self.myscale*np.random.randint(0, (self.grid_dimension+1))
			y = self.myscale*np.random.randint(0, (self.grid_dimension+1))
			x_sign=random.choice(['-','+'])
			y_sign=random.choice(['+','-'])
			offset_y=-self.myscale/4
			offset_x=-self.myscale/4
			if(x_sign=='-'):
				x=x*-1
				offset_x=self.myscale/4
			if(y_sign=='-'):
				y=y*-1
				offset_y=self.myscale/4
			if(count%2==0):
				if(x<0):
					x1=x+self.myscale
				else:
					x1=x-self.myscale
				y1=y
				'''if ((x,y,x1,y1)  in self.blocked_edges) or ((x1,y1,x,y)  in self.blocked_edges):
					print ((x,y,x1,y1),'is a blocked edg')'''
				if (((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0) not in self.coke_list) and (((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0) not in self.cup_list) and ((x,y,x1,y1) not in self.blocked_edges) and ((x1,y1,x,y) not in self.blocked_edges) and (x>=1.0 or x<=-1.0) and (y>=1.0 or y<=-1.0) and ((x,y) not in can_loadloc) and ((x1,y1) not in can_loadloc) and ((x,y) not in cup_loadloc) and ((x1,y1) not in cup_loadloc) and ((x,y) not in self.bin_loc) and ((x1,y1) not in self.bin_loc):
					self.cup_list.append(((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0))
					cups["cup_"+str(count)]={}
					self.cup_dict_generator(cups,count,((x+x1+offset_x)/2,(y+y1+offset_y)/2),(x,y),(x1,y1))
					count=count+1
					cup_loadloc.append((x,y))
					cup_loadloc.append((x1,y1))
					self.blocked_edges.add((x,y,x1,y1))
			else:
				if(y<0):
					y1=y+self.myscale
				else:
					y1=y-self.myscale
				x1=x
				'''if ((x,y,x1,y1)  in self.blocked_edges) or ((x1,y1,x,y)  in self.blocked_edges):
					print ((x,y,x1,y1),'is a blocked edge')'''
				if (((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0) not in self.coke_list) and (((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0) not in self.cup_list) and ((x,y,x1,y1) not in self.blocked_edges) and ((x1,y1,x,y) not in self.blocked_edges) and (x>=1.0 or x<=-1.0) and (y>=1.0 or y<=-1.0) and ((x,y) not in can_loadloc) and ((x1,y1) not in can_loadloc) and ((x,y) not in cup_loadloc) and ((x1,y1) not in cup_loadloc) and ((x,y) not in self.bin_loc) and ((x1,y1) not in self.bin_loc):
					self.cup_list.append(((x+x1+offset_x)/2,(y+y1+offset_y)/2,0,0,-0,0))
					cups["cup_"+str(count)]={}
					self.cup_dict_generator(cups,count,((x+x1+offset_x)/2,(y+y1+offset_y)/2),(x,y),(x1,y1))
					count=count+1
					cup_loadloc.append((x,y))
					cup_loadloc.append((x1,y1))
					self.blocked_edges.add((x,y,x1,y1))
		self.add_walls(f_out, self.grid_dimension*self.myscale)		
		self.add_cans_description(f_out)
		self.add_walls_description(f_out)
		object_dict["cans"] = cans
		object_dict["cups"] = cups
		object_dict["bins"] = bins
	 	with open(root_path + '/objects.json', 'w') as fp:
	 		json.dump(object_dict, fp)

	 	return object_dict
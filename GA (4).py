import numpy
import copy
import time
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.transforms import Bbox
from matplotlib.path import Path as Line
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from shapely.geometry import LineString, Point, GeometryCollection, MultiPoint, MultiLineString
class My_Point:
	def __init__ (self, x, y):
		self.x = x
		self.y = y
		self.distance = 0
	def get_xy(self):
		return (self.x, self.y)

	@staticmethod
	def get_distance(a, b):
		return numpy.sqrt(numpy.square(a.x-b.x)+numpy.square(a.y-b.y))

class Path:
	def __init__ (self, start=None, target=None, length=0,path=None):
		if(start is None):
			self.path=path
			self.start=path[0]
			self.target=path[len(path)-1]
			self.length=0
		else:
			self.start = start
			self.target = target
			self.length = length
			self.path = []
			self.path.append(self.start)
			for i in range(length):
				self.path.append(My_Point(numpy.random.uniform()*100, numpy.random.uniform()*100))
			self.path.append(self.target)
	# def __init__(self,path):
	# 	self.path=path
	# 	self.start=path[0]
	# 	self.target=path[len(path)-1]
	# 	self.length=0
	# def get_line_string(self):
	# 	return LineString([p.getXy() for p in self.path])
	# first = { path_1 , path_2 ,path_3}
	#second = {path_2 , path_3 , path_4}
	def get_Multiline_string(self):
		multi = ()
		for first, second in zip(self.path[:-1], self.path[1:]):
			multi = multi+ ( (first.get_xy() , second.get_xy()), )
		
		return MultiLineString(multi)

	def get_total_distance(self):
		dist = 0
		for first, second in zip(self.path[:-1], self.path[1:]):
			dist += My_Point.get_distance(first, second)
			second.distance = dist		#assign the distance from the start point to the current reched point (point_2)
		return dist
	#get collisions with obstacles
	def get_collision(self, obstacles):
		count = 0
		for first, second in zip(self.path[:-1], self.path[1:]):
			vertices = [[first.x, first.y], [second.x, second.y]]
			path = Line(vertices)
			for obstacle in obstacles:
				bbox = obstacle.bbox
				if path.intersects_bbox(bbox):
					count += 1
		return count
	#draws the robots path
	def draw(self, ax, idx,tasksnum):
		if idx == 0:
			color = 'r'
		elif idx == 1:
			color = 'b'
		else:
			color = 'k'

		for i, (first, second) in enumerate(zip(self.path[:-1], self.path[1:])):
			ax.plot([first.x, second.x], [first.y, second.y], color, label='Agent %d' %(idx) if i==0 else None)
			# enumerate returns index with  a tuple of 2 points each time ...... (0,start,Point_1) , (1,Point_1,Point_2)
			#label : create a legend , so it is called once in the dirst tuple of enumeration only.
		for c in self.path:
			ax.plot(c.x, c.y, 'ro')
		g=tasksnum+1
		while(g<len(self.path)):
			ax.plot(self.path[g].x,self.path[g].y,'go')
			g+=tasksnum+1
		ax.plot(self.start.x, self.start.y, 'bo')
		ax.plot(self.target.x, self.target.y, 'go')

	def get_cost(self, obstacles):
		return self.get_collision(obstacles)*500 + self.get_total_distance()

	def modify(self):
		r1 = numpy.random.randint(1, len(self.path)-1)

		tempx = self.path[r1].x + (numpy.random.uniform())*10
		while ((tempx<0) or (tempx>100)):
			tempx = self.path[r1].x + (numpy.random.uniform())*10
		#while tempx>100:
		#	tempx = self.path[r1].x + (numpy.random.uniform())*10 	# 0.5 to limit the modification on the coordinate for range (-5,5)

		self.path[r1].x = tempx
		
		tempy = self.path[r1].y + (numpy.random.uniform())*10
		while ((tempy<0) or (tempy>100)):
			tempy = self.path[r1].y + (numpy.random.uniform())*10
		#while tempy>100:
		#	tempy = self.path[r1].y + (numpy.random.uniform())*10
		self.path[r1].y = tempy

	def modify_updated(self):
		for i in range (1,len(self.path)-1):
			tempx = self.path[i].x + (numpy.random.uniform())*20
			while ((tempx<0) or (tempx>100)):
				tempx = self.path[i].x + (numpy.random.uniform())*20
			self.path[i].x = tempx

			tempy = self.path[i].y + (numpy.random.uniform())*20
			while ((tempy<0) or (tempy>100)):
				tempy = self.path[i].y + (numpy.random.uniform())*20
			self.path[i].y = tempy


	def display_path_object(self):
		for i in range(len(self.path)):
			print(self.path[i].get_xy())

class GA_MaPath:
	def __init__(self,Robot_list,length,populationsize,obstacles,PE,PC,PM):
		self.obstacles=obstacles
		self.PE=PE
		self.PC=PC
		self.PM=PM
		self.Robot_list=Robot_list
		self.length=length
		self.populationsize=populationsize
		self.population=[]
		#self.child_1=MAPath(copy.deepcopy(Robot_list),self.length)
		#self.child_2=MAPath(copy.deepcopy(Robot_list),self.length)
		print("I will enter the loop ")
		for i in range(populationsize):
			print("iteration ,",i)
			new_path = MAPath(copy.deepcopy(Robot_list),length)
			new_path_2 =copy.deepcopy(new_path)
			#new_path.display_path_list()
			self.population.append(new_path_2)
		self.Elite_List=[]
		self.crossover_list=[]
		self.medium_list=[]
		self.mutation_list=[]
		# self.assign_path_list_of_GA_Mapath()
		#print("size of robot 0 pathslist =",len(self.Robot_list[0].paths_list))
		#print("size of robot 1 pathslist =",len(self.Robot_list[1].paths_list))
	def display_robot_tasks_GA(self):
		for i in range(len(self.population)):
			print("for MAPath %d"%(i))
			self.population[i].display_robot_tasks()
	def get_index_of_medium_ind_to_crossover(self):
		cost_ind_0 = self.Elite_List[0].cost_mapath
		for i in range(0,len(self.medium_list)):
			if((self.medium_list[i].cost_mapath - cost_ind_0)>10):
				return i
	def assign_path_list_of_GA_Mapath(self):
			self.Robot_list=copy.deepcopy(self.population[0].RobotList)
	def get_index_of_elite_list_ind_to_crossover(self):
		cost_ind_0 = self.Elite_List[0].cost_mapath
		for i in range(0,len(self.Elite_List)):
			if((self.Elite_List[i].cost_mapath - cost_ind_0)>10):
				return i
		return 0
	def assign_cost_to_population(self):
		print("size of population on assigning the cost is ", len(self.population))
		for i in range(len(self.population)):
			#temp = copy.deepcopy(self.population[i])
			#temp.get_cost_MA_Path_Planning(self.obstacles)
			#self.population[i]=copy.deepcopy(temp)
			self.population[i].get_cost_MA_Path_Planning(self.obstacles)
			#print("length of path list of this MApath = ",len(self.population[i].path_list))
			#print("no of points inside the MApath robot 0 :",len(self.population[i].path_list[0].path))
			#print("no of points inside the MApath robot 1 :",len(self.population[i].path_list[1].path))
			#print("length of Robot_list of this MApath = ",len(self.population[i].RobotList))
			#print("total path list For MApath no :", i)
			#self.population[i].display_path_list()
		print("finished assigning cost ")

	def sort_the_population (self):
		self.population=sorted(self.population,key=lambda x: x.cost_mapath)

	def display_sorted_costs(self):
		print("sorted costs :")
		for i in range(len(self.population)):
			#self.population[i].display_path_list()
			print("for MApath no %d the new cost equals %d"%(i,self.population[i].cost_mapath))

	def get_E_M_M_lists(self):
		#print("-------------before appending the lists :")
		#print("size of elite list = ",len(self.Elite_List))
		#print("size of medium list = ",len(self.medium_list))
		#print("size of mutation list = ",len(self.mutation_list))
		num_of_elites=(int)(self.populationsize*self.PE)
		self.Elite_List=[]
		self.medium_list=[]
		self.mutation_list=[]
		num_of_children=(int)(self.populationsize*self.PC)
		num_of_crossovers=num_of_children
		if(num_of_children % 2 !=0):
			num_of_crossovers+=1
		num_of_crossovers=(int)(num_of_crossovers/2)
		for i in range(self.populationsize):
			if(i<(num_of_elites)):
				self.Elite_List.append(copy.deepcopy(self.population[i]))
			elif(i<(num_of_children+num_of_elites)):
				self.medium_list.append(copy.deepcopy(self.population[i]))
			else:
				self.mutation_list.append(copy.deepcopy(self.population[i]))
		#print("-------------after appending the lists :")
		#print("size of elite list = ",len(self.Elite_List))
		#print("size of medium list = ",len(self.medium_list))
		#print("size of mutation list = ",len(self.mutation_list))

	def assign_total_path_to_population(self):
		for i in range(self.populationsize):
			temp = copy.deepcopy(self.population[i])
			temp.assign_TotalPath()
			self.population[i]=copy.deepcopy(temp)
		#self.child_1.assign_TotalPath()
		#self.child_2.assign_TotalPath()
		#print("finished assign_robotPath_to_population for iteration 0 ")

	def crossover(self):
		print("start crossover ////////////////////////////////")
		self.crossover_list=[] # list of childs of type {MAPaths} 
		num_of_children=(int)(self.populationsize*self.PC)
		num_of_crossovers=num_of_children
		if(num_of_children % 2 !=0):
			num_of_crossovers+=1
		num_of_crossovers=(int)(num_of_crossovers/2)
		 # we need to modify the robotlist[i].paths_list ={path1 , path2 ,........,path n}
		alpha = 0.3
		#token_for_parent_2_elite = 0
		#token_for_addidtion = 1
		#token_for_parent_1_elite = 1
		for i in range(num_of_crossovers):
			token_for_parent_1_elite = numpy.random.randint(0,2)
			token_for_parent_2_elite = numpy.random.randint(0,2)
			#r_parent_1=numpy.random.randint(0,len(self.Elite_List))  # Parent MAPath from elite list. ={path_rob1 , path_rob2}
			#self.child_1.assign_TotalPath()
			#self.child_2.assign_TotalPath()
			#print("self.child 1 path before crossover is :")
			#self.child_1.display_path_list()
			#print("self.child 2 path before crossover is :")
			#self.child_2.display_path_list()
			if((token_for_parent_1_elite == 1) and (token_for_parent_2_elite == 1)):
				print("crossover between 2 elites individuals")
				parent_1=copy.deepcopy(self.Elite_List[0])
				#r_parent_2=numpy.random.randint(0,len(self.Elite_List)) # Parent MAPath from medium list. ={path_rob1 , path_rob2}
				r_parent_2=1
				#while(r_parent_2 == 0):
				#	r_parent_2=numpy.random.randint(0,len(self.Elite_List))
				print("the elite individual to crossover with the elite is at index ",r_parent_2)
				parent_2=copy.deepcopy(self.Elite_List[r_parent_2])
				token_for_addidtion =   numpy.random.randint(0,2)    		# may be 0 or 1 randomly
				#token_for_addidtion =  0
				print("token_for addition randomly = ",token_for_addidtion)
			elif(((token_for_parent_1_elite == 1) and (token_for_parent_2_elite == 0)) or ((token_for_parent_1_elite == 1) and (token_for_parent_2_elite == 0))):
				#r_parent_1=numpy.random.randint(0,len(self.medium_list))
				print("crossover between elite and medium individuals")
				parent_1 = self.Elite_List[0]
				r_parent_2=numpy.random.randint(0,len(self.medium_list))
				print("the medium individual to crossover with the elite is at index ",r_parent_2)
				parent_2=copy.deepcopy(self.medium_list[r_parent_2])
				token_for_addidtion= 0
				print("token_for addition randomly = ",token_for_addidtion)
				

			else:
				print("crossover between 2 medium list individuals")
				r_parent_1=numpy.random.randint(0,len(self.medium_list))
				parent_1=copy.deepcopy(self.medium_list[r_parent_1])
				r_parent_2=numpy.random.randint(0,len(self.medium_list))
				parent_2=copy.deepcopy(self.medium_list[r_parent_2])
				print("the crossover is done between MAPath %d and MApath %d"%(r_parent_1,r_parent_2))
				token_for_addidtion = 0
			#print("parent 1 path :")
			#parent_1.display_path_list()
			#print("parent 2 path :")
			#parent_2.display_path_list()
			child_1_temp=copy.deepcopy(self.child_1)
			child_2_temp=copy.deepcopy(self.child_2)
			#child_1_temp.assign_TotalPath()
			#child_2_temp.assign_TotalPath()
			print("child_1_temp path before crossover is :")
			child_1_temp.display_path_list()
			print("child_2_temp path before crossover is :")
			child_2_temp.display_path_list()
			if(token_for_addidtion == 1):
				print("now addition crossover !")
				for j in range(len(self.Robot_list)): # loop over the 2 robots
					for k in range(len(parent_1.RobotList[j].paths_list)):
						print("ana henaaaaaaaaaaaaaaaaaaaaa ahoooooooooooooooooo")
						#print("size of self.robotlist.pathslist=",len(self.Robot_list[j].paths_list))
						#print("size of robot path list =",len(parent_1.RobotList[j].paths_list))
						path_p1 = copy.deepcopy(parent_1.RobotList[j].paths_list[k])
						path_p2 = copy.deepcopy(parent_2.RobotList[j].paths_list[k])
						for l in range (1,len(parent_1.RobotList[j].paths_list[k].path)-1):
							#print("for path no ",k)
							#print ("for point no ",l)
							#print("parent_1_x =", path_p1.path[l].x)	
							#print("parent_1_y =", path_p1.path[l].y)
							#print("parent_2_x =", path_p2.path[l].x)	
							#print("parent_2_y =", path_p2.path[l].y)	
							child_1_temp.RobotList[j].paths_list[k].path[l].x = alpha * (path_p1.path[l].x ) + (1-alpha)*(path_p2.path[l].x )
							child_2_temp.RobotList[j].paths_list[k].path[l].x = alpha * (path_p2.path[l].x ) + (1-alpha)*(path_p1.path[l].x )
							child_1_temp.RobotList[j].paths_list[k].path[l].y = alpha * (path_p1.path[l].y ) + (1-alpha)*(path_p2.path[l].y )
							child_2_temp.RobotList[j].paths_list[k].path[l].y = alpha * (path_p2.path[l].y ) + (1-alpha)*(path_p1.path[l].y )
							#print("child_1_x =", child_1_temp.RobotList[j].paths_list[k].path[l].x )
							#print("child_1_y =", child_1_temp.RobotList[j].paths_list[k].path[l].y)
				token_for_addidtion = 0
			elif (token_for_addidtion == 0):
				print("now Subtraction crossover !")
				for j in range(len(self.Robot_list)): # loop over the 2 robots
					for k in range(len(parent_1.RobotList[j].paths_list)):
						print("ana henaaaaaaaaaaaaaaaaaaaaa ahoooooooooooooooooo")
						#print("size of self.robotlist.pathslist=",len(self.Robot_list[j].paths_list))
						#print("size of robot path list =",len(parent_1.RobotList[j].paths_list))
						path_p1 = copy.deepcopy(parent_1.RobotList[j].paths_list[k])
						path_p2 = copy.deepcopy(parent_2.RobotList[j].paths_list[k])
						for l in range (1,len(parent_1.RobotList[j].paths_list[k].path)-1):
							#print("for path no ",k)
							#print ("for point no ",l)
							#print("parent_1_x =", path_p1.path[l].x)	
							#print("parent_1_y =", path_p1.path[l].y)
							#print("parent_2_x =", path_p2.path[l].x)	
							#print("parent_2_y =", path_p2.path[l].y)	
							child_1_temp.RobotList[j].paths_list[k].path[l].x = abs(alpha * (path_p1.path[l].x ) - (1-alpha)*(path_p2.path[l].x ))
							child_2_temp.RobotList[j].paths_list[k].path[l].x = abs(alpha * (path_p2.path[l].x ) - (1-alpha)*(path_p1.path[l].x ))
							child_1_temp.RobotList[j].paths_list[k].path[l].y = abs(alpha * (path_p1.path[l].y ) - (1-alpha)*(path_p2.path[l].y ))
							child_2_temp.RobotList[j].paths_list[k].path[l].y = abs(alpha * (path_p2.path[l].y ) - (1-alpha)*(path_p1.path[l].y ))
							#print("child_1_x =", child_1_temp.RobotList[j].paths_list[k].path[l].x )
							#print("child_1_y =", child_1_temp.RobotList[j].paths_list[k].path[l].y)
				token_for_addidtion = 1	
			#print("child_1_x in robot 0 path 0 point 1=", child_1_temp.RobotList[0].paths_list[0].path[1].x )
			#print("child_1_y in robot 0 path 0 point 1=", child_1_temp.RobotList[0].paths_list[0].path[1].y)
			#child_1_temp.assign_TotalPath()
			#child_2_temp.assign_TotalPath()
			#print("child_1_x in robot 0 path 0 point 1 =", child_1_temp.RobotList[0].paths_list[0].path[1].x )
			#print("child_1_y in robot 0 path 0 point 1 =", child_1_temp.RobotList[0].paths_list[0].path[1].y)
			#print("child 1 path :")
			#child_1_temp.display_path_list()
			#print("child 2 path :")
			#child_2_temp.display_path_list()
			#self.child_1=copy.deepcopy(child_1_temp)
			#self.child_2=copy.deepcopy(child_2_temp)
			print("child_1_temp path after crossover is :")
			child_1_temp.display_path_list()
			print("child_2_temp path after crossover is :")
			child_2_temp.display_path_list()
			print('-----------------')
			
			#print([self.child_1.RobotList[0].paths_list[0].path[count].x, self.child_1.RobotList[0].paths_list[0].path[count].y for count in range(len(self.child_1.RobotList[0].paths_list[0].path))])
			self.child_1.display_path_list()
			self.child_1=copy.deepcopy(child_1_temp)
			self.child_2=copy.deepcopy(child_2_temp)
			self.crossover_list.append(child_1_temp)
			self.crossover_list.append(child_2_temp)
			print("now size of crossover list = ",len(self.crossover_list))
			
		if(len(self.crossover_list)>num_of_children):
			self.crossover_list.pop()

	def crossover_updated(self):
		print(" I am inside crossover updated !!!")
		self.crossover_list=[]
		num_of_children=(int)(self.populationsize*self.PC)
		num_of_crossovers=num_of_children
		if(num_of_children % 2 !=0):
			num_of_crossovers+=1
		num_of_crossovers=(int)(num_of_crossovers/2)
		alpha = 0.7
		token_for_random_crossover = 0
		#ratio=0.7
		for i in range(num_of_crossovers):
			token_for_addidtion = numpy.random.randint(0,2)
			child_1=MAPath(copy.deepcopy(self.Robot_list),self.length)
			child_2=MAPath(copy.deepcopy(self.Robot_list),self.length)
			child_1.assign_TotalPath()
			child_2.assign_TotalPath()
			print("Before Crossover , the child 1_path is:")
			child_1.display_path_list()
			print("Before Crossover , the child 2_path is:")
			child_2.display_path_list()
			print("sisaaaaaaaay crossover")
			#r_parent_1=numpy.random.randint(0,len(self.Elite_List))
			parent_1=copy.deepcopy(self.Elite_List[0])
			if(token_for_random_crossover == 1): # the best elite with one from the medium lists
				#r_parent_2=numpy.random.randint(0,len(self.medium_list))
				r_parent_2 = self.get_index_of_medium_ind_to_crossover()
				parent_2=copy.deepcopy(self.medium_list[r_parent_2])
				token_for_random_crossover = 0
			if(token_for_random_crossover == 0 ): # the best elite with one elite
				r_parent_2=numpy.random.randint(1,len(self.Elite_List))
				parent_2=copy.deepcopy(self.Elite_List[r_parent_2])
				token_for_random_crossover = 1
			#if(token_for_random_crossover == 2): # any 2 individuals from the population except the elites
			#	r_parent_1 = numpy.random.randint(len(self.Elite_List),len(self.population)-len(self.Elite_List))
			#	r_parent_2 = numpy.random.randint(len(self.Elite_List),len(self.population)-len(self.Elite_List))
			#	while(r_parent_2 == r_parent_1):
			#		r_parent_2 = numpy.random.randint(len(self.Elite_List),len(self.population)-len(self.Elite_List))
			#	parent_1=copy.deepcopy(self.population[r_parent_1])
			#	parent_2=copy.deepcopy(self.population[r_parent_2])
			#	token_for_random_crossover = 1
			if(token_for_addidtion == 1):
				for j in range(len(self.Robot_list)): # loop over the 2 robots
						for k in range(len(parent_1.RobotList[j].paths_list)):
							print("ana henaaaaaaaaaaaaaaaaaaaaa ahoooooooooooooooooo")
							#print("size of self.robotlist.pathslist=",len(self.Robot_list[j].paths_list))
							#print("size of robot path list =",len(parent_1.RobotList[j].paths_list))
							path_p1 = copy.deepcopy(parent_1.RobotList[j].paths_list[k])
							path_p2 = copy.deepcopy(parent_2.RobotList[j].paths_list[k])
							for l in range (1,len(parent_1.RobotList[j].paths_list[k].path)-1):
								#print("for path no ",k)
								#print ("for point no ",l)
								#print("parent_1_x =", path_p1.path[l].x)	
								#print("parent_1_y =", path_p1.path[l].y)
								#print("parent_2_x =", path_p2.path[l].x)	
								#print("parent_2_y =", path_p2.path[l].y)	
								child_1.RobotList[j].paths_list[k].path[l].x = alpha * (path_p1.path[l].x ) + (1-alpha)*(path_p2.path[l].x )
								child_2.RobotList[j].paths_list[k].path[l].x = alpha * (path_p2.path[l].x ) + (1-alpha)*(path_p1.path[l].x )
								child_1.RobotList[j].paths_list[k].path[l].y = alpha * (path_p1.path[l].y ) + (1-alpha)*(path_p2.path[l].y )
								child_2.RobotList[j].paths_list[k].path[l].y = alpha * (path_p2.path[l].y ) + (1-alpha)*(path_p1.path[l].y )
				
			else :
				for j in range(len(self.Robot_list)): # loop over the 2 robots
						for k in range(len(parent_1.RobotList[j].paths_list)):
							print("ana henaaaaaaaaaaaaaaaaaaaaa ahoooooooooooooooooo")
							#print("size of self.robotlist.pathslist=",len(self.Robot_list[j].paths_list))
							#print("size of robot path list =",len(parent_1.RobotList[j].paths_list))
							path_p1 = copy.deepcopy(parent_1.RobotList[j].paths_list[k])
							path_p2 = copy.deepcopy(parent_2.RobotList[j].paths_list[k])
							for l in range (1,len(parent_1.RobotList[j].paths_list[k].path)-1):
								#print("for path no ",k)
								#print ("for point no ",l)
								#print("parent_1_x =", path_p1.path[l].x)	
								#print("parent_1_y =", path_p1.path[l].y)
								#print("parent_2_x =", path_p2.path[l].x)	
								#print("parent_2_y =", path_p2.path[l].y)	
								child_1.RobotList[j].paths_list[k].path[l].x = abs(alpha * (path_p1.path[l].x ) - (1-alpha)*(path_p2.path[l].x ))
								child_2.RobotList[j].paths_list[k].path[l].x = abs(alpha * (path_p2.path[l].x ) - (1-alpha)*(path_p1.path[l].x ))
								child_1.RobotList[j].paths_list[k].path[l].y = abs(alpha * (path_p1.path[l].y ) - (1-alpha)*(path_p2.path[l].y ))
								child_2.RobotList[j].paths_list[k].path[l].y = abs(alpha * (path_p2.path[l].y ) - (1-alpha)*(path_p1.path[l].y ))
								#print("child 1_x =", child_1.RobotList[j].paths_list[k].path[l].x)	
								#print("child_1_y =", child_1.RobotList[j].paths_list[k].path[l].y)
								#print("child_2_x =", child_2.RobotList[j].paths_list[k].path[l].x)	
								#print("child_2_y =", child_2.RobotList[j].paths_list[k].path[l].x)
			child_1.assign_TotalPath()
			child_2.assign_TotalPath()
			#print("after Crossover and before appending , the child 1_path is:")
			#child_1.display_path_list()
			#print("after Crossover and before appending , the child 2_path is:")
			#child_2.display_path_list()
			self.crossover_list.append(copy.deepcopy(child_1))
			self.crossover_list.append(copy.deepcopy(child_2))
			#print("now the size of crossover list =",len(self.crossover_list))
			#print("the added child_1 to the crossover list is")
			#self.crossover_list[len(self.crossover_list)-2].display_path_list()
			#print("the added child_2 to the crossover list is")
			#self.crossover_list[len(self.crossover_list)-1].display_path_list()
		if(len(self.crossover_list)>num_of_children):
			self.crossover_list.pop()
	def mutation(self):
		print("Iam in mutation ******************/////////////////////*********")
		new_mutation_list=[]
		for i in range(len(self.mutation_list)):
			#print(" in loop of i")
			temp_MApath =copy.deepcopy(self.mutation_list[i])
			#print("Individual paths before before mutation :")
			#self.mutation_list[i].display_path_list()
			temp_MApath.modify()
			new_mutation_list.append(copy.deepcopy(temp_MApath))
			#self.mutation_list[i]=copy.deepcopy(temp_MApath)
			self.mutation_list[i]=copy.deepcopy(temp_MApath)
			self.mutation_list[i].assign_TotalPath()
			#print("Individual Paths After mutation :")
			#self.mutation_list[i].display_path_list()
			#for j in range(len(self.Robot_list)): # loop over the 2 robots
			#	print(" in loop of j")
			#	print("size of robotlist[j].pathslist = ",len(self.Robot_list[j].paths_list))
			#	for k in range(len(self.population[0].RobotList[j].paths_list)):
			#		print("in loop k")
			#		temp=copy.deepcopy(self.mutation_list[i].RobotList[j].paths_list[k])
			#		temp.modify()
			#		self.mutation_list[i].RobotList[j].paths_list[k]=copy.deepcopy(temp)
			#self.mutation_list[i].assign_TotalPath()	# re assign the modified paths to each MAPath object
		self.mutation_list=copy.deepcopy(new_mutation_list)
		print(" I have finished the mutation correctly !!")

	def modify(self):
		self.crossover_updated()
		self.mutation()
		self.population=[]
		self.population.extend(copy.deepcopy(self.Elite_List))
		self.population.extend(copy.deepcopy(self.crossover_list))
		self.population.extend(copy.deepcopy(self.mutation_list))	

	def get_best_MA_GA(self):
		return copy.deepcopy(self.Elite_List[0])
	def display_mutation_list(self):
		print(" The mutation list individuals are :")
		for i in range(len(self.mutation_list)):
			self.mutation_list[i].display_path_list()
	def display_crossover_list(self):
		print(" The crossover list individuals are :")
		for i in range(len(self.crossover_list)):
			self.crossover_list[i].display_path_list()
# MAPath.pathlist = {path_rob1 , path_rob2}
#Mapth.robotlist[i].robot_path = {path 1 , path 2 , path 3}
#mapath.robotlist[i].robot_path = {path 1 , path 2 ,path 3}
#crossover : 
	


class MAPath:
	def __init__(self, Robot_list,length):
		self.length = length	#number of points between start and target.
		self.RobotList=Robot_list
		self.cost_mapath=0
		self.path_list=[] #array of paths of all robots. {path1,path2} 
		print(" I am in MAPath ")
		print(" I finished MAPath ")
		print("before  size of robot 0 pathslist =",len(self.RobotList[0].paths_list))
		print("before  size of robot 1 pathslist =",len(self.RobotList[1].paths_list))
		self.assign_robotPath()
		print("size of robot 0 pathslist =",len(self.RobotList[0].paths_list))
		print("size of robot 1 pathslist =",len(self.RobotList[1].paths_list))
		
	def display_robot_tasks(self):
		for i in range(len(self.RobotList)):
			print("for robot %d : "%(i))
			for j in range(len(self.RobotList[i].tasks)):
				print(self.RobotList[i].tasks[j].point.get_xy()) 

	def assign_robotPath(self): #assign paths between tasks for every robot
		for i in range(len(self.RobotList)):
			temp=None
			self.RobotList[i].paths_list=[]
			for j in range(len(self.RobotList[i].tasks)):
				if(j==0):
					temp=Path(self.RobotList[i].point,self.RobotList[i].tasks[j].point,self.length)
				else:
					temp=Path(self.RobotList[i].tasks[j-1].point,self.RobotList[i].tasks[j].point,self.length)
				self.RobotList[i].paths_list.append(temp)
		#print("finished assign robot path")	
	def assign_TotalPath(self):
		for i in range(len(self.RobotList)):
			temp=[]
			for j in range(len(self.RobotList[i].paths_list)):
				temp.extend(self.RobotList[i].paths_list[j].path[:-1])
			temp.append(self.RobotList[i].paths_list[len(self.RobotList[i].paths_list)-1].path[len(self.RobotList[i].paths_list[len(self.RobotList[i].paths_list)-1].path)-1]) 
			self.RobotList[i].TotalPath=Path(None,None,None,copy.deepcopy(temp))
		# Update the self.path_list
		self.path_list = []
		for i in range(len(self.RobotList)):
			path = copy.deepcopy(self.RobotList[i].TotalPath)
			self.path_list.append(path)
		#print("finished assign total path")	

	def get_path_collision(self):
		count = 0
		for i in range(len(self.path_list)-1):
			line1 = self.path_list[i].get_Multiline_string()
			for j in range(i+1, len(self.path_list)):
				line2 = self.path_list[j].get_Multiline_string()

				intersections = line1.intersection(line2)
			
				if intersections is None:
					continue
				elif isinstance(intersections, GeometryCollection):

					for intersection in intersections:
						if isinstance(intersection, LineString):
							count += 1
						elif isinstance(intersection, Point):
							for idx, line in enumerate(line1):
								if line.distance(intersection) < 1e-3 :
									d1 = self.path_list[i].path[idx].distance + My_Point.get_distance(self.path_list[i].path[idx], My_Point(intersection.x, intersection.y))

							for idx, line in enumerate(line2):
								if line.distance(intersection) < 1e-3 :
									d2 = self.path_list[j].path[idx].distance + My_Point.get_distance(self.path_list[j].path[idx], My_Point(intersection.x, intersection.y))
							
							if numpy.abs(d1-d2) < 2:
								count += 1
				elif isinstance(intersections, MultiLineString):
					count += len(intersections)

				elif isinstance(intersections, MultiPoint):
					
					for intersection in intersections:
						
						for idx, line in enumerate(line1):
							
							if line.distance(intersection) < 1e-3  :
								d1 = self.path_list[i].path[idx].distance + My_Point.get_distance(self.path_list[i].path[idx], My_Point(intersection.x, intersection.y))
							
						for idx, line in enumerate(line2):
							if line.distance(intersection) < 1e-3 :
								d2 = self.path_list[j].path[idx].distance + My_Point.get_distance(self.path_list[j].path[idx], My_Point(intersection.x, intersection.y))
						
						if numpy.abs(d1-d2) < 2:
							count += 1

				elif isinstance(intersections, LineString):
					count += 1

				elif isinstance(intersections, Point):
						
					for idx, line in enumerate(line1):
						if line.distance(intersections) < 1e-3 :
							d1 = self.path_list[i].path[idx].distance + My_Point.get_distance(self.path_list[i].path[idx], My_Point(intersections.x, intersections.y))
							
					for idx, line in enumerate(line2):
						if line.distance(intersections) < 1e-3 :
							d2 = self.path_list[j].path[idx].distance + My_Point.get_distance(self.path_list[j].path[idx], My_Point(intersections.x, intersections.y))
							
					if numpy.abs(d1-d2) < 2:
						count += 1
		return count

	def get_final_collision(self):
		for i in range(len(self.path_list)-1):
			line1 = self.path_list[i].get_Multiline_string()
			for j in range(i+1, len(self.path_list)):
				line2 = self.path_list[j].get_Multiline_string()

				intersections = line1.intersection(line2)

				if intersections is None:
					continue
				elif isinstance(intersections, MultiPoint):
					for intersection in intersections:
						for idx, line in enumerate(line1):
							if line.distance(intersection) < 1e-3 :
								d1 = self.path_list[i].path[idx].distance + My_Point.get_distance(self.path_list[i].path[idx], My_Point(intersection.x, intersection.y))

						for idx, line in enumerate(line2):
							if line.distance(intersection) < 1e-3 :
								d2 = self.path_list[j].path[idx].distance + My_Point.get_distance(self.path_list[j].path[idx], My_Point(intersection.x, intersection.y))
						print("Agent %d and %d reaches point x = %7.4f and y = %7.4f at times %7.4f and %7.4f respectively"%(i,j, intersection.x, intersection.y, d1, d2))
			
				elif isinstance(intersections, Point):	
					for idx, line in enumerate(line1):
						if line.distance(intersections) < 1e-3 :
							d1 = self.path_list[i].path[idx].distance + My_Point.get_distance(self.path_list[i].path[idx], My_Point(intersections.x, intersections.y))

					for idx, line in enumerate(line2):
						if line.distance(intersections) < 1e-3 :
							d2 = self.path_list[j].path[idx].distance + My_Point.get_distance(self.path_list[j].path[idx], My_Point(intersections.x, intersections.y))
					
					print("Agent %d and %d reaches point x = %7.4f and y = %7.4f at times %7.4f and %7.4f respectively"%(i,j, intersections.x, intersections.y, d1, d2))

	def draw(self, ax):
		for idx, path in enumerate(self.path_list):
			path.draw(ax,idx,self.length)


	# def get_cost(self, obstacles):
	# 	cost = 0
	# 	for path in self.path_list:
	# 		cost += path.get_cost(obstacles)
	# 	cost += self.get_path_collision()*300
	# 	return cost

	def modify(self):
		for i in range(len(self.RobotList)):
			for j in range(len(self.RobotList[i].paths_list)):
				temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
				temp_path.modify_updated()
				self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)

	# def get_cost_MA_Path_Planning(self):
	# 	cost = 0
	# 	for i in range(len(self.path_list)):	
	# 		cost += self.path_list[i].get_total_distance()
	# 	cost+= self.get_path_collision()
	# 	return cost
	def get_cost_MA_Path_Planning(self,obstacles):
		cost = 0
		for i in range(len(self.path_list)):
			temp_path =copy.deepcopy(self.path_list[i])	
			cost += temp_path.get_cost(obstacles)
		cost+= self.get_path_collision()*200
		self.cost_mapath = cost
		return cost

	def display_path_list(self):
		k=0
		for robot_path in self.path_list:
			print("robot %d  path  :" % (k+1))
			for point in robot_path.path:
				print(point.get_xy())
			k+=1
			
				
		
class Task:
	def __init__(self,point,time):
		self.point=point
		self.time=time
		self.distance=0
		
class Robot:	
	def __init__(self,point):
		self.point=point
		self.time=0
		self.tasks=[]
		self.paths_list=[]
		self.TotalPath=None
		self.distance_travelled=0
	def display_robot_path(self):
		for i in range(len(self.paths_list)):
			for j in range(len(self.paths_list[i].path)):
				print(self.paths_list[i].path[j].get_xy())
	def display_robot_tasks_points(self):
		for i in range(len(self.tasks)):
			print(self.tasks[i].point.get_xy())
class GA_Task_allocation:
	def __init__(self,task_list,Robot_list,max_num_tasks_per_robot,populationsize,PE,PC,PM):
		self.task_list=task_list
		self.Robot_list=Robot_list
		self.max_num_tasks_per_robot=max_num_tasks_per_robot
		self.populationsize=populationsize
		self.population=[]
		self.PE=PE
		self.PC=PC
		self.PM=PM
		self.Elite_List=[]
		self.crossover_list=[]
		self.medium_list=[]
		self.mutation_list=[]
		
		for i in range(self.populationsize):
			#temp2=copy.deepcopy(TaskAllocation(self.task_list,self.Robot_list,self.max_num_tasks_per_robot))
			self.population.append(TaskAllocation(self.task_list,self.Robot_list,self.max_num_tasks_per_robot))
		#self.get_SortedPopulation()
	def sort_the_population(self):
		self.population=sorted(self.population,key=lambda x: x.cost_TA_list)
	def get_E_M_M_lists(self):
		num_of_elites=(int)(self.populationsize*self.PE)
		self.Elite_List=[]
		self.medium_list=[]
		self.mutation_list=[]
		num_of_children=(int)(self.populationsize*self.PC)
		num_of_crossovers=num_of_children
		if(num_of_children % 2 !=0):
			num_of_crossovers+=1
		num_of_crossovers=(int)(num_of_crossovers/2)
		#print("number of elites = ", num_of_elites)
		#print("number of children =", num_of_children)
		for i in range(self.populationsize):
			if(i<(num_of_elites)):# i<10
				self.Elite_List.append(self.population[i])
				
			elif(i<(num_of_children + num_of_elites)):#i<40
				self.medium_list.append(self.population[i])
				
			else:
				if(self.population[i].check_feasibilty() == True):
					self.mutation_list.append(self.population[i])
				else:
					print("inside emm all_list" , self.population[i].Allocation_list)
					
		print("size of elite list = ",len(self.Elite_List))
		print("size of medium list = ",len(self.medium_list))
		print("size of mutation list = ",len(self.mutation_list))
		# print(len(self.Elite_List))
		# print(len(self.population))
		# print(len(self.medium_list))
		# print("num of children",num_of_children)
		# print(len(self.mutation_list))
		
	def crossover(self): ## Random elite with Random chromosome
		self.crossover_list=[]
		num_of_children=(int)(self.populationsize*self.PC)
		num_of_crossovers=num_of_children
		if(num_of_children % 2 !=0):
			num_of_crossovers+=1
		num_of_crossovers=(int)(num_of_crossovers/2)
		for i in range(num_of_crossovers):
			child_1=TaskAllocation(self.task_list,self.Robot_list,self.max_num_tasks_per_robot)
			child_2=TaskAllocation(self.task_list,self.Robot_list,self.max_num_tasks_per_robot)
			while(1):
				print("sisaaaaaaaay crossover")
				r_parent_1=numpy.random.randint(0,len(self.Elite_List))
				parent_1=self.Elite_List[r_parent_1]
				r_parent_2=numpy.random.randint(0,len(self.medium_list))
				parent_2=self.medium_list[r_parent_2]
				r=numpy.random.randint(1,len(self.population[i].Allocation_list)-1)
				
				for j in range(len(self.population[i].Allocation_list)):
					if(j<r):
						child_1.Allocation_list[j]=parent_1.Allocation_list[j]
						child_2.Allocation_list[j]=parent_2.Allocation_list[j]
					else:
						child_1.Allocation_list[j]=parent_2.Allocation_list[j]
						child_2.Allocation_list[j]=parent_1.Allocation_list[j]
				#child_1.assign_tasks()
				#child_2.assign_tasks()

				#replace upper lines by those :
				child_1_temp=copy.deepcopy(child_1)
				child_2_temp=copy.deepcopy(child_2)
				child_1_temp.assign_tasks()
				child_2_temp.assign_tasks()
				child_1_temp.calculatetime_robot()
				child_2_temp.calculatetime_robot()
				child_1=copy.deepcopy(child_1_temp)
				child_2=copy.deepcopy(child_2_temp)

				check_child_1 =child_1.check_feasibilty()
				check_child_2 =child_2.check_feasibilty()
				if((check_child_1 == True) and (check_child_2 == True)):
					break
			self.crossover_list.append(child_1)
			self.crossover_list.append(child_2)
			print("children crossovered lists :")
			print("child 1 : ",child_1.Allocation_list)
			print("child 2 : ",child_2.Allocation_list)

		if(len(self.crossover_list)>num_of_children):
			self.crossover_list.pop()
		#print("size of crossoverlist = ",len(self.crossover_list))
	def mutation(self):
		print("size of mutation list before starting all mutations is :",len(self.mutation_list))
		for i in range(len(self.mutation_list)):
			print("for mutation list index :", i)
			print("allocation list before mutation =",self.mutation_list[i].Allocation_list)
			#task_all_temp=copy.deepcopy(self.mutation_list[i])
			#while(1):
			print("sisaaaaaaay Mutation")
			print("size of allocation list before anything =",len(self.mutation_list[i].Allocation_list))
			r_1=numpy.random.randint(0,len(self.mutation_list[i].Allocation_list))
			print("r_1=",r_1)
			r_2=numpy.random.randint(0,len(self.mutation_list[i].Allocation_list))
			while(r_1 == r_2):
				r_2=numpy.random.randint(0,len(self.mutation_list[i].Allocation_list))
			print("r_2=",r_2)
			print("number of robot 0 tasks before mutation =",len(self.mutation_list[i].Robot_list[0].tasks))
			print("number of robot 1 tasks before mutation = :",len(self.mutation_list[i].Robot_list[1].tasks))
			print("first index before switch = ",self.mutation_list[i].Allocation_list[r_1])
			print("first index before switch = ",self.mutation_list[i].Allocation_list[r_2])
			temp=self.mutation_list[i].Allocation_list[r_1]
			self.mutation_list[i].Allocation_list[r_1]=self.mutation_list[i].Allocation_list[r_2]
			self.mutation_list[i].Allocation_list[r_2]=temp
			print("first index after switch = ",self.mutation_list[i].Allocation_list[r_1])
			print("first index after switch = ",self.mutation_list[i].Allocation_list[r_2])
			print(" allocation list after switching the values =",self.mutation_list[i].Allocation_list)
			#self.mutation_list[i].assign_tasks()
			#self.mutation_list[i].calculatetime_robot()
				#if(self.mutation_list[i].check_feasibilty() == True):
				#	print("TRUE feasibility *********")
				#	print("number of robot 0 tasks after mutation :",len(self.mutation_list[i].Robot_list[0].tasks))
				#	print("number of robot 1 tasks after mutation :",len(self.mutation_list[i].Robot_list[1].tasks))
				#	break
				#else:
				#	print("FALSEEE !!!!")
				#	print("size of allocation list =",len(self.mutation_list[i].Allocation_list))
				#	print("number of total tasks = ",len(self.mutation_list[i].task_list))
				#	print("number of robot 0 tasks in case of fail:",len(self.mutation_list[i].Robot_list[0].tasks))
				#	print("number of robot 1 tasks in case of fail:",len(self.mutation_list[i].Robot_list[1].tasks))
				#	self.mutation_list[i] = copy.deepcopy(task_all_temp)
				#	self.mutation_list[i].assign_tasks()
				#	self.mutation_list[i].calculatetime_robot()
			print("this mutation list is mutated successfully !")
		print("size of mutation list after finishing all mutations is :",len(self.mutation_list))
			
	def modify(self):
		self.crossover()
		self.mutation()
		print("size of mutation before extend  is ",len(self.mutation_list))
		self.population=[]
		self.population.extend(copy.deepcopy(self.Elite_List))
		self.population.extend(copy.deepcopy(self.crossover_list))
		self.population.extend(copy.deepcopy(self.mutation_list))
		print("size of mutation after modify  is ",len(self.mutation_list))
		

	def get_best(self):
		return self.Elite_List[0]

	def assign_tasks_to_robots(self):
		for i in range(self.populationsize):
			#print(i)
			temp=copy.deepcopy(self.population[i])
			temp.assign_tasks()
			self.population[i]=copy.deepcopy(temp)
			
	def assign_costs_to_TA_lists(self):
		for i in range(self.populationsize):
			self.population[i].get_cost_taskAllocation()
	def display_all_allocation_lists(self):
		for i in range(self.populationsize):
			print("task allocation no %d "%(i),self.population[i].Allocation_list)
			print("its cost = ",self.population[i].cost_TA_list)
			print("for robot 0 : ",len(self.population[i].Robot_list[0].tasks))
			print("for robot 1 : ",len(self.population[i].Robot_list[1].tasks))
			print("robot 0 time = ",self.population[i].Robot_list[0].time)
			print("robot 1 time = ",self.population[i].Robot_list[1].time)

	def assign_robots_timings(self):
		for i in range(self.populationsize):
			self.population[i].calculatetime_robot()
		
class TaskAllocation:
	def __init__(self,task_list,Robot_list,max_num_tasks_per_robot):
		self.cost_TA_list=0
		self.task_list=task_list
		self.Robot_list=Robot_list
		self.Allocation_list=[0]*len(task_list) # solution representation for task allocation
		self.maxn=max_num_tasks_per_robot
		while(1):
			self.Allocation_list=[0]*len(task_list) 
			for i in range(len(task_list)):
				self.Allocation_list[i]+=numpy.random.randint(0, len(self.Robot_list))
			self.assign_tasks()
			#self.calculatetime_robot()
			if(self.check_feasibilty() == True):
				break

	def modify(self):
		r1= numpy.random.randint(0, len(self.Allocation_list))
		temp= self.Allocation_list[r1]+ numpy.random.randint(-(len(self.Robot_list)-1),len(self.Robot_list))
		while temp< 0 or temp>len(self.Robot_list)-1:
			temp= self.Allocation_list[r1]+ numpy.random.randint(-(len(self.Robot_list)-1),len(self.Robot_list))
		self.Allocation_list[r1]=temp
	def assign_tasks(self):
		for i in range(len(self.Robot_list)):
			self.Robot_list[i].tasks=[]
		for i in range(len(self.Allocation_list)):
			self.Robot_list[self.Allocation_list[i]].tasks.append(self.task_list[i])
			
			#print("task %d is assigned to robot %d successfully"%(i,self.Allocation_list[i]))
	def get_cost_taskAllocation(self):
		#self.assign_tasks()
		max=0
		SI=0
		tempdistance=0
		for robot in self.Robot_list:
			temp=0
			
			for task in range(len(robot.tasks)):
				if(task==0):
					temp+= My_Point.get_distance(robot.point,robot.tasks[task].point)
					tempdistance+=temp
					temp+= robot.tasks[task].time
				else:
					temp+=My_Point.get_distance(robot.tasks[task].point,robot.tasks[task-1].point)
					tempdistance+=temp
					temp+= robot.tasks[task].time
			robot.time=temp    
			if(temp>max):
				max=temp
		for robot in self.Robot_list:
			SI+=numpy.square(robot.time-max)
		SI=SI/len(self.Robot_list)
		SI=numpy.sqrt(SI)
		SI= SI*0.25 + 0.75*tempdistance
		self.cost_TA_list=SI
		return SI
	def check_feasibilty(self): # to check the constraints of max no of tasks assigned to each robot
		for robot in self.Robot_list :
			if (len(robot.tasks) > self.maxn):
				return False
		return True
	def calculatetime_robot(self):
		for robot in self.Robot_list:
			temp=0
			for task in range(len(robot.tasks)):
				if(task==0):
					temp+= My_Point.get_distance(robot.point,robot.tasks[task].point)
					temp+= robot.tasks[task].time
				else:
					temp+=My_Point.get_distance(robot.tasks[task].point,robot.tasks[task-1].point)
					temp+= robot.tasks[task].time
			robot.time=temp    
class Obstacle:
	def __init__(self,left, bottom, width, height ):
		self.rect = plt.Rectangle((left, bottom), width, height, facecolor="black", alpha=0.5)
		self.bbox = Bbox.from_bounds(left, bottom, width, height)

	def draw(self,ax):
		ax.add_patch(self.rect)



if __name__ == '__main__':

	#####################	TASK ALLOCATION GA ALGORITHM 	#####################

	print ("************************************************** TASK ALLOCATION RESULTS ************************************************** ")

	task_list= [Task(My_Point(40,50),30),Task(My_Point(60,80),29),Task(My_Point(80,30),58),Task(My_Point(60,60),36),Task(My_Point(90,10),39),
	Task(My_Point(99,70),25),Task(My_Point(80,80),40),Task(My_Point(50,80),20),Task(My_Point(30,80),55)]
	
	Robot_list=[Robot(My_Point(10,10)),Robot(My_Point(10,90))]
	
	max_num_tasks_per_robot = 5
	
	GA_task_allocation=GA_Task_allocation(task_list,Robot_list,max_num_tasks_per_robot,100,0.1,0.6,0.3)#initialize population with percentages elitism
	print(len(GA_task_allocation.population))
	GA_task_allocation.assign_tasks_to_robots()	
	GA_task_allocation.assign_costs_to_TA_lists()
	GA_task_allocation.assign_robots_timings()
	GA_task_allocation.display_all_allocation_lists()																								#crossover and mutation
	GA_task_allocation.sort_the_population()
	GA_task_allocation.get_E_M_M_lists()																						
	GA_task_allocation.display_all_allocation_lists()
	start_time = time.time()
	i_max=120
	i=0
	##plotting 
	temp_valeus_yaxis=[]
	iterations_xaxis=[range(0,i_max+1)]
	best_cost=GA_task_allocation.Elite_List[0].cost_TA_list
	best_taskAllocation=copy.deepcopy(GA_task_allocation.Elite_List[0])
	#print(len(GA_task_allocation.population))
	#print(len(GA_task_allocation.Elite_List))
	best_costlist=[]
	best_costlist.append(best_cost)
	while(i<i_max):
		print("**********************************************************For iteration :",i)
		#print("size of population before manipulation=",len(GA_task_allocation.population))
		#print("size of mutation list =",len(GA_task_allocation.mutation_list))
		#print("size of elite list =",len(GA_task_allocation.Elite_List))
		#print("size of crossover list =",len(GA_task_allocation.crossover_list))
		#print("size of medium list =",len(GA_task_allocation.medium_list))
		GA_task_allocation.modify()
		GA_task_allocation.assign_tasks_to_robots()
		GA_task_allocation.assign_costs_to_TA_lists()
		GA_task_allocation.assign_robots_timings()
		GA_task_allocation.sort_the_population()
		#print("******size of mutation list before calling EMM FUNCTION******* =",len(GA_task_allocation.mutation_list))
		GA_task_allocation.get_E_M_M_lists()
		#print("******size of mutation list AFTER CALLING EMM FUNCTION******** =",len(GA_task_allocation.mutation_list))
		#print("size of population after manipulation =",len(GA_task_allocation.population))

		currentcost_best=GA_task_allocation.Elite_List[0].cost_TA_list
		if(currentcost_best<best_cost):
			best_taskAllocation=copy.deepcopy(GA_task_allocation.Elite_List[0])
			print("best_task_all = ",best_taskAllocation.Allocation_list)
			best_cost=currentcost_best
		print("current best TA_GA" ,best_cost)
		i=i+1
		best_costlist.append(best_cost)
	print("final best GA_TA= ",best_cost)
	print(best_taskAllocation.Allocation_list)
	print(best_taskAllocation.Robot_list[0].time)
	print(best_taskAllocation.Robot_list[1].time)
	best_taskAllocation.assign_tasks()
	print("number of tasks of robot 0 = ", len(best_taskAllocation.Robot_list[0].tasks))
	print("number of tasks of robot 1 = ", len(best_taskAllocation.Robot_list[1].tasks))
	#best_taskAllocation.assign_tasks()
	#print(best_taskAllocation.Allocation_list)
	#best_taskAllocation.calculatetime_robot()
	print("best cost other = ",best_taskAllocation.cost_TA_list)
	fig2 = plt.figure()
	plt.ylabel("Best Cost TA")
	plt.xlabel("Iterations")
	plt.plot(best_costlist)
	plt.show()


	#####################	PATH PLANNING GA ALGORITHM ####################
	

	fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
	

	
	ax1.legend(loc='upper left')
	ax1.set_xlabel('X(m)')
	ax1.set_ylabel('Y(m)')
	ax1.set_title('Initial Path')
	
	obstacle_list = []
	obstacle_list2 = []
	left1, bottom1, width1, height1 = (20, 10, 20, 20)
	rect1 = Obstacle(left1, bottom1, width1, height1)
	obstacle_list.append(rect1)
	rect11 = Obstacle(left1, bottom1, width1, height1)
	obstacle_list2.append(rect11)
	
	for obstacle in obstacle_list:
		obstacle.draw(ax1)

	print("BEFORE WE GOOOOO !!!")

	GA_Path_Planning = GA_MaPath(copy.deepcopy(best_taskAllocation.Robot_list),2,3000,obstacle_list,0.2,0.7,0.1)
	GA_Path_Planning.assign_total_path_to_population()	
	GA_Path_Planning.assign_cost_to_population()																								
	GA_Path_Planning.sort_the_population()
	GA_Path_Planning.display_sorted_costs()
	GA_Path_Planning.get_E_M_M_lists()
	GA_Path_Planning.display_mutation_list()
	GA_Path_Planning.display_crossover_list()	
	GA_Path_Planning.get_best_MA_GA().draw(ax1)		
	print("HERE WE GOOOO !!!")
	i_max=10
	i=0
	best_cost=GA_Path_Planning.Elite_List[0].cost_mapath
	best_MAPath =copy.deepcopy(GA_Path_Planning.Elite_List[0])
	while(i<i_max):
		print(" ***********************FOR ITERAtion   %d  ****************************** "%(i))
		
		#print(" A random MApath beofre Modification")
		#GA_Path_Planning.population[15].display_path_list()
		GA_Path_Planning.modify()
		print("modification done successfully !")
		GA_Path_Planning.assign_total_path_to_population()
		print("assign robot path done successfully !")
		#print(" A random MApath after Modification")
		#GA_Path_Planning.population[15].display_path_list()
		GA_Path_Planning.assign_cost_to_population()
		print("assign costs done successfully !")
		GA_Path_Planning.sort_the_population()
		print("sorting done successfullly !")
		GA_Path_Planning.display_sorted_costs()
		#print("individual no  is :")
		#GA_Path_Planning.population[180].display_path_list()
		#print("its cost is =",GA_Path_Planning.population[180].cost_mapath)
		GA_Path_Planning.get_E_M_M_lists()
		print("getting EMM lists done succesfully !")
		GA_Path_Planning.display_mutation_list()
		GA_Path_Planning.display_crossover_list()
		currentcost_best=GA_Path_Planning.Elite_List[0].cost_mapath
		if(currentcost_best<best_cost):
			best_MAPath=copy.deepcopy(GA_Path_Planning.Elite_List[0])
			best_cost=currentcost_best
		print("current best PP_GA " ,best_cost)
		i=i+1
	print("YA SISAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAY")
	print("best path is :")
	best_MAPath.display_path_list()
	print("final best GA_PP= ",best_cost)












	best_MAPath.draw(ax2)
	for obstacle in obstacle_list2:
		obstacle.draw(ax2)
	ax2.legend(loc='upper left')

	ax2.set_xlabel('X(m)')
	ax2.set_ylabel('Y(m)')
	ax2.set_title('Optimized Path')
	
	# fig2 = plt.figure()
	# plt.ylabel("Best Cost")
	# plt.xlabel("Iterations")
	# plt.plot(cost_list)
	

	plt.show()


	#####################	     Testing area		   ####################

	#Path_planning = MAPath(taskallocation.Robot_list,2)
	#print("robot 0 tasks :")
	#Path_planning.RobotList[0].display_robot_tasks_points()
	#print("MApath list :")
	#Path_planning.display_path_list()
	#print('robot 0 path :')
	#Path_planning.RobotList[0].display_robot_path()
	#print("robot total path :")
	#Path_planning.RobotList[0].TotalPath.display_path_object()

	#path_test =Path(My_Point(5,0),My_Point(2,4),3)
	#print("path test points :")
	#path_test.display_path_object()
	#print("cost of test path :")
	#print(path_test.get_total_distance())


	#####################	PATH PLANNING SA ALGORITHM ####################
	# print ("************************************************** PATH PLANNING RESULTS ************************************************** ")
	# fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
	
	# Path_planning = MAPath(taskallocation.Robot_list,2)
	# Path_planning.draw(ax1)
	# ax1.legend(loc='upper left')
	# ax1.set_xlabel('X(m)')
	# ax1.set_ylabel('Y(m)')
	# ax1.set_title('Initial Path')
	
	# obstacle_list = []
	# obstacle_list2 = []
	# left1, bottom1, width1, height1 = (20, 10, 20, 20)
	# rect1 = Obstacle(left1, bottom1, width1, height1)
	# obstacle_list.append(rect1)
	# rect11 = Obstacle(left1, bottom1, width1, height1)
	# obstacle_list2.append(rect11)
	
	# for obstacle in obstacle_list:
	# 	obstacle.draw(ax1)
	
	# T_initial_pp=100000
	# T_final_pp=1
	# i_max_pp=800
	# current_cost_planning=Path_planning.get_cost_MA_Path_Planning(obstacle_list)
	# best_path=copy.deepcopy(Path_planning.path_list)
	# best_cost_pp = current_cost_planning
	# best_cost_iteration_number = 0
	# best_cost_subiteration_number=0
	# T_current_pp=T_initial_pp
	# Beta_pp= (T_initial_pp - T_final_pp)/i_max_pp
	# i=0
	# currentRobotList=copy.deepcopy(Path_planning.RobotList)
	# print("Initial_cost = ",current_cost_planning)
	# print("Initial temp =",T_initial_pp)
	# print("Initial no of collisions =",Path_planning.get_path_collision())
	# best_pathplanning=copy.deepcopy(Path_planning)
	# cost_list =[]
	# temprature_list=[]
	# k=1
	
	# while(i<i_max_pp):
	# 	print("*********************************************************************************************************************")
	# 	print(" ***** For iteration %d ****" % (i+1))
	# 	print("current temperature = ",T_current_pp)
	# 	for j in range(0,10):
	# 		print("--- for sub-iteration number %d ---" %(j+1))
	# 		temp=copy.deepcopy(currentRobotList)
	# 		Path_planning.modify()
	# 		print("length of self.pathlist BEFORE assign total path =",len(Path_planning.path_list))
	# 		Path_planning.assign_TotalPath()
	# 		#print("path_planning path list modified is :")
	# 		#Path_planning.display_path_list()
	# 		print("length of self.pathlist AFTER assign total path =",len(Path_planning.path_list))
	# 		new_cost_path_planning = Path_planning.get_cost_MA_Path_Planning(obstacle_list)
			
	# 		print("no of collisions = ",Path_planning.get_path_collision())
	# 		print("new_cost_calculated = ", new_cost_path_planning)
	# 		print("current cost_before_comparing = = ", current_cost_planning)
	# 		delta_energy_pp = new_cost_path_planning - current_cost_planning
	# 		print(" deltaenergy_PP =", delta_energy_pp)
	# 		if(delta_energy_pp<0):
	# 			print(" WE DIRECTLY ACCEPTED THE NEW SOLUTION ")
	# 			print("length of self.pathlist before assign_total_path =",len(Path_planning.path_list))
	# 			currentRobotList=copy.deepcopy(Path_planning.RobotList)
	# 			Path_planning.assign_TotalPath()
	# 			current_cost_planning=new_cost_path_planning
	# 			print("current_cost =",current_cost_planning)
	# 		else:
	# 			r = numpy.random.uniform()
				
	# 			prob_pp = numpy.exp(-1*delta_energy_pp/T_current_pp)
	# 			print(" r = ",r)
	# 			print( " prob_pp =", prob_pp)
	# 			if prob_pp > r:
	# 				print(" WE ACCEPTED THE SOLUTION BY PROBABILITY !!")
	# 				currentRobotList=copy.deepcopy(Path_planning.RobotList)
	# 				print("length of self.pathlist before assign_total_path =",len(Path_planning.path_list))
	# 				Path_planning.assign_TotalPath()
	# 				current_cost_planning=new_cost_path_planning
	# 				print("current_cost =",current_cost_planning)
	# 			else:
	# 				print(" WE REJECTED THE SOLUTION !!!! ")
	# 				print("current_cost remains =",current_cost_planning)
	# 				Path_planning.RobotList=copy.deepcopy(temp)
	# 				Path_planning.assign_TotalPath()
			
	# 		if(best_cost_pp > current_cost_planning):
	# 			best_cost_pp=current_cost_planning
	# 			best_path=copy.deepcopy(Path_planning.path_list)
	# 			best_pathplanning=copy.deepcopy(Path_planning)
	# 			best_cost_iteration_number = i+1
	# 			best_cost_subiteration_number = j+1
	# 			print(" BEST COST UPDATED SUCCESSFULLY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
	# 			print("Now, best cost = ",best_cost_pp)
	# 		cost_list.append(best_cost_pp)
			
	# 		print("So,best_cost_pp until now = ",best_cost_pp)
	# 	temprature_list.append(T_current_pp)
	# 	# # if(i>200):
	# 	# 	T_current_pp=T_initial_pp*pow(0.8,k)
	# 	# 	k+=1
	# 	# else:
	# 	# 	T_current_pp=T_initial_pp - Beta_pp*(i+1)
	# 	T_current_pp=T_initial_pp*pow(0.95,i)
	# 	i=i+1
	# Path_planning.path_list = copy.deepcopy(best_path)
	# print("*********************************************************************************************************************")
	# print("Final best cost_pp =",best_cost_pp)
	# print("best_cost_pp is obtained in iteration no: %d , in sub-iteration %d " % (best_cost_iteration_number ,best_cost_subiteration_number))
	# print("Final_best_path :")
	# endtime=start_time-time.time()
	# print("code finished in ",endtime)
	# Path_planning.display_path_list()
	# Path_planning.get_final_collision()

	# best_pathplanning.draw(ax2)
	# for obstacle in obstacle_list2:
	# 	obstacle.draw(ax2)
	# ax2.legend(loc='upper left')

	# ax2.set_xlabel('X(m)')
	# ax2.set_ylabel('Y(m)')
	# ax2.set_title('Optimized Path')
	
	# fig2 = plt.figure()
	# plt.ylabel("Best Cost")
	# plt.xlabel("Iterations")
	# plt.plot(cost_list)
	

	
	# fig3=plt.figure()
	# plt.ylabel("temprature")
	# plt.xlabel("iterations")
	# plt.plot(temprature_list)
	# plt.show()

	








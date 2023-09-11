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
				self.path.append(My_Point(numpy.random.randint(0, 100), numpy.random.randint(0, 100)))
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
			ax.plot([first.x, second.x], [first.y, second.y], color, label='Robot %d' %(idx) if i==0 else None)
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
	def draw_tasks(self, ax, idx,tasksnum):
		if idx == 0:
			color = 'ro'
		elif idx == 1:
			color = 'go'
		else:
			color = 'k'

		# for i, (first, second) in enumerate(zip(self.path[:-1], self.path[1:])):
		# 	ax.plot([first.x, second.x], [first.y, second.y], color, label='Robot %d' %(idx) if i==0 else None)
		# 	# enumerate returns index with  a tuple of 2 points each time ...... (0,start,Point_1) , (1,Point_1,Point_2)
		# 	#label : create a legend , so it is called once in the dirst tuple of enumeration only.
		# for c in self.path:
		# 	ax.plot(c.x, c.y, 'ro')
		g=tasksnum+1
		while(g<len(self.path)):
			ax.plot(self.path[g].x,self.path[g].y,color)
			g+=tasksnum+1
		ax.plot(self.start.x, self.start.y, color)
		# ax.plot(self.target.x, self.target.y, 'go')
	def get_cost(self, obstacles):
		return self.get_collision(obstacles)*500 + self.get_total_distance()

	def modify(self):
		r1 = numpy.random.randint(1, len(self.path)-1)

		tempx = self.path[r1].x + (numpy.random.uniform()-0.5)*10
		while tempx<0:
			tempx = self.path[r1].x + (numpy.random.uniform()-0.5)*10
		while tempx>100:
			tempx = self.path[r1].x + (numpy.random.uniform()-0.5)*10 	# 0.5 to limit the modification on the coordinate for range (-5,5)

		self.path[r1].x = tempx
		
		tempy = self.path[r1].y + (numpy.random.uniform()-0.5)*10
		while tempy<0:
			tempy = self.path[r1].y + (numpy.random.uniform()-0.5)*10
		while tempy>100:
			tempy = self.path[r1].y + (numpy.random.uniform()-0.5)*10
		self.path[r1].y = tempy

	def display_path_object(self):
		for i in range(len(self.path)):
			print(self.path[i].get_xy())
	


class MAPath:
	def __init__(self, Robot_list,length):
		self.length = length	#number of points between start and target.
		self.RobotList=Robot_list
		self.path_list=[] #array of paths of all robots.
		self.assign_robotPath()
		self.assign_TotalPath()
				
	def assign_robotPath(self): #assign paths between tasks for every robot
		for i in range(len(self.RobotList)):
			temp=None
			for j in range(len(self.RobotList[i].tasks)):
				if(j==0):
					temp=Path(self.RobotList[i].point,self.RobotList[i].tasks[j].point,self.length)
				else:
					temp=Path(self.RobotList[i].tasks[j-1].point,self.RobotList[i].tasks[j].point,self.length)
				self.RobotList[i].paths_list.append(temp)	
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
			path = self.RobotList[i].TotalPath
			self.path_list.append(path)
	
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
				# elif isinstance(intersections, MultiLineString):
				# 	count += len(intersections)

				# elif isinstance(intersections, MultiPoint):
					
				# 	for intersection in intersections:
						
				# 		for idx, line in enumerate(line1):
							
				# 			if line.distance(intersection) < 1e-3  :
				# 				d1 = self.path_list[i].path[idx].distance + My_Point.get_distance(self.path_list[i].path[idx], My_Point(intersection.x, intersection.y))
							
				# 		for idx, line in enumerate(line2):
				# 			if line.distance(intersection) < 1e-3 :
				# 				d2 = self.path_list[j].path[idx].distance + My_Point.get_distance(self.path_list[j].path[idx], My_Point(intersection.x, intersection.y))
						
				# 		if numpy.abs(d1-d2) < 2:
				# 			count += 1

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
						print("Robot %d and %d reaches point x = %7.4f and y = %7.4f at times %7.4f and %7.4f respectively"%(i,j, intersection.x, intersection.y, d1, d2))
			
				elif isinstance(intersections, Point):	
					for idx, line in enumerate(line1):
						if line.distance(intersections) < 1e-3 :
							d1 = self.path_list[i].path[idx].distance + My_Point.get_distance(self.path_list[i].path[idx], My_Point(intersections.x, intersections.y))

					for idx, line in enumerate(line2):
						if line.distance(intersections) < 1e-3 :
							d2 = self.path_list[j].path[idx].distance + My_Point.get_distance(self.path_list[j].path[idx], My_Point(intersections.x, intersections.y))
					
					print("Robot %d and %d reaches point x = %7.4f and y = %7.4f at times %7.4f and %7.4f respectively"%(i,j, intersections.x, intersections.y, d1, d2))

	def draw(self, ax):
		for idx, path in enumerate(self.path_list):
			path.draw(ax,idx,self.length)
	def draw_tasks(self, ax):
		for idx, path in enumerate(self.path_list):
			path.draw_tasks(ax,idx,self.length)

	def get_cost(self, obstacles):
		cost = 0
		for path in self.path_list:
			cost += path.get_cost(obstacles)
		cost += self.get_path_collision()*300
		return cost

	def modify(self):
		for i in range(len(self.RobotList)):
			for j in range(len(self.RobotList[i].paths_list)):
				self.RobotList[i].paths_list[j].modify()

	# def get_cost_MA_Path_Planning(self):
	# 	cost = 0
	# 	for i in range(len(self.path_list)):	
	# 		cost += self.path_list[i].get_total_distance()
	# 	cost+= self.get_path_collision()
	# 	return cost
	def get_cost_MA_Path_Planning(self,obstacles):
		cost = 0
		for i in range(len(self.path_list)):	
			cost += self.path_list[i].get_cost(obstacles)
		cost+= self.get_path_collision()*200
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

class TaskAllocation:
	def __init__(self,task_list,Robot_list,max_num_tasks_per_robot):
		self.task_list=task_list
		self.Robot_list=Robot_list
		self.Allocation_list=[0]*len(task_list) # solution representation for task allocation
		self.maxn=max_num_tasks_per_robot
		while(1):
			self.Allocation_list=[0]*len(task_list) 
			for i in range(len(task_list)):
				self.Allocation_list[i]+=numpy.random.randint(0, len(self.Robot_list))
			self.assign_tasks()
			if(self.check_feasibilty() == True):
				break

	def modify(self):
		r1= numpy.random.randint(0, len(self.Allocation_list))
		temp= self.Allocation_list[r1]+ numpy.random.randint(-(len(self.Robot_list)-1),len(self.Robot_list))
		while temp< 0 or temp>len(self.Robot_list)-1:
			temp= self.Allocation_list[r1]+ numpy.random.randint(-(len(self.Robot_list)-1),len(self.Robot_list))
		self.Allocation_list[r1]=temp
	def assign_tasks(self):
		for i in range(0,len(self.Robot_list)):
			self.Robot_list[i].tasks=[]
		for i in range(0,len(self.Allocation_list)):
			self.Robot_list[self.Allocation_list[i]].tasks.append(self.task_list[i])
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
					tempdistance+=My_Point.get_distance(robot.point,robot.tasks[task].point)
					temp+= robot.tasks[task].time
				else:
					temp+=My_Point.get_distance(robot.tasks[task].point,robot.tasks[task-1].point)
					tempdistance+=My_Point.get_distance(robot.tasks[task].point,robot.tasks[task-1].point)
					temp+= robot.tasks[task].time
			robot.time=temp    
			if(temp>max):
				max=temp
		for robot in self.Robot_list:
			SI+=numpy.square(robot.time-max)
		SI=SI/len(self.Robot_list)
		SI=numpy.sqrt(SI)
		print("Si inside cost=",SI )
		print("temp distance inside cost ", tempdistance)
		SI= SI*0.2*40 + 0.8*tempdistance
		#SI=tempdistance
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
	#####################	TASK ALLOCATION SA ALGORITHM 	#####################
	print ("************************************************** TASK ALLOCATION RESULTS ************************************************** ")
	#taskallocation=TaskAllocation(task_list= [Task(My_Point(50,30),50),Task(My_Point(50,60),55),Task(My_Point(40,90),55),
	#Task(My_Point(80,55),55),Task(My_Point(80,90),55)],
	#Robot_list=[Robot(My_Point(10,10)),Robot(My_Point(10,20))],max_num_tasks_per_robot=10)
	task_list= [Task(My_Point(15,80),120),Task(My_Point(40,40),400),Task(My_Point(50,70),100),Task(My_Point(80,90),120),Task(My_Point(90,10),50),
	Task(My_Point(75,60),70),Task(My_Point(75,30),50),Task(My_Point(90,30),50),Task(My_Point(45,20),70)]
	max_num_tasks_per_robot = 5
	Robot_list=[Robot(My_Point(10,45)),Robot(My_Point(15,45))]
	taskallocation=TaskAllocation(task_list,Robot_list,max_num_tasks_per_robot)

	cost0= taskallocation.get_cost_taskAllocation()
	print(cost0)
	print(taskallocation.Robot_list[0].time)
	print(taskallocation.Robot_list[1].time)
	current_cost=cost0
	current_taskalloction=copy.deepcopy(taskallocation.Allocation_list)
	T_initial=500
	T_final=1
	T_current=500
	best_cost = cost0
	best_taskallocation=copy.deepcopy(taskallocation.Allocation_list)
	start_time = time.time()
	i_max=1000
	Beta=-(T_final-T_initial)/i_max
	i=0
	##plotting 
	temp_valeus_yaxis=[]
	iterations_xaxis=[range(0,i_max+1)]
	bestcost_list_TA=[]
	bestcost_list_TA.append(best_cost)
	temprature_list_TA=[T_current]
	while(i<i_max):
		for n in range(0,50):
			temp_allocationlist=copy.deepcopy(current_taskalloction)
			while(1):
				taskallocation.modify()
				taskallocation.assign_tasks()
				if(taskallocation.check_feasibilty()==True):
					break
				else:
					taskallocation.Allocation_list=copy.deepcopy(temp_allocationlist)
			cost_new=taskallocation.get_cost_taskAllocation()
			delta_energy=cost_new - current_cost
			print("delta energy_TA = ", delta_energy)
			if(delta_energy<0):
				current_cost=cost_new
				current_taskalloction=copy.deepcopy(taskallocation.Allocation_list)
				print("we accept directly the solution ")
			else:
				r = numpy.random.uniform()
				prob_task_allocation =numpy.exp((current_cost-cost_new)/T_current)
				print("for iteration :" , i)
				print("r =",r)
				print("prob_TA = ",prob_task_allocation)
				if r < prob_task_allocation:
					current_cost=cost_new
					current_taskalloction=copy.deepcopy(taskallocation.Allocation_list)
				else:
					taskallocation.Allocation_list=copy.deepcopy(temp_allocationlist)
			if(current_cost<best_cost):
				best_cost=current_cost
				best_taskallocation=copy.deepcopy(current_taskalloction)
			
			print("For iteration ",i)
			print("current_cost_TA =",current_cost)
			print("time robot 1",taskallocation.Robot_list[0].time ," Time robot 2 ",taskallocation.Robot_list[1].time)	
		#T_current=T_initial*pow(0.99,i)		
		T_current=T_initial-Beta*(i+1)
		bestcost_list_TA.append(best_cost)
		temprature_list_TA.append(T_current)
		i=i+1
	taskallocation.Allocation_list=copy.deepcopy(best_taskallocation)
	taskallocation.assign_tasks()
	taskallocation.calculatetime_robot()
	print(best_cost)
	print(best_taskallocation)
	print(taskallocation.Robot_list[0].time)
	print(taskallocation.Robot_list[1].time)
	print("number of tasks of robot 0 = ", len(taskallocation.Robot_list[0].tasks))
	print("number of tasks of robot 1 = ", len(taskallocation.Robot_list[1].tasks))
	
		
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
	print ("************************************************** PATH PLANNING RESULTS ************************************************** ")
	fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
	fig6,ax3=plt.subplots()
	
	Path_planning = MAPath(taskallocation.Robot_list,2)
	Path_planning.draw(ax1)
	Path_planning.draw_tasks(ax3)
	
	ax1.legend(loc='upper left')
	ax1.set_xlabel('X(m)')
	ax1.set_ylabel('Y(m)')
	ax1.set_title('Initial Path')
	ax3.legend(loc='upper left')
	ax3.set_xlabel('X(m)')
	ax3.set_ylabel('Y(m)')
	ax3.set_title('Task Allocation')
	#plt.show()
	obstacle_list = []
	obstacle_list2 = []
	left1, bottom1, width1, height1 = (20, 10, 20, 20)
	rect1 = Obstacle(left1, bottom1, width1, height1)
	left2, bottom2, width2, height2 = (30, 60, 15, 15)
	rect2 = Obstacle(left2, bottom2, width2, height2)
	#left3, bottom3, width3, height3 = (65, 40, 20, 15)
	#rect3 = Obstacle(left3, bottom3, width3, height3)
	obstacle_list.append(rect1)
	obstacle_list.append(rect2)
	#obstacle_list.append(rect3)
	rect11 = Obstacle(left1, bottom1, width1, height1)
	#left2, bottom2, width2, height2 = (30, 60, 15, 15)
	rect22=Obstacle(left2, bottom2, width2, height2)
	#left3, bottom3, width3, height3 = (65, 40, 20, 15)
	#rect33 = Obstacle(left3, bottom3, width3, height3)
	obstacle_list2.append(rect11)
	obstacle_list2.append(rect22)
	
	for obstacle in obstacle_list:
		obstacle.draw(ax1)
	
	T_initial_pp=100000
	T_final_pp=1
	i_max_pp=3000
	current_cost_planning=Path_planning.get_cost_MA_Path_Planning(obstacle_list)
	best_path=copy.deepcopy(Path_planning.path_list)
	best_cost_pp = current_cost_planning
	best_cost_iteration_number = 0
	best_cost_subiteration_number=0
	T_current_pp=T_initial_pp
	Beta_pp= (T_initial_pp - T_final_pp)/i_max_pp
	i=0
	currentRobotList=copy.deepcopy(Path_planning.RobotList)
	print("Initial_cost = ",current_cost_planning)
	print("Initial temp =",T_initial_pp)
	print("Initial no of collisions =",Path_planning.get_path_collision())
	best_pathplanning=copy.deepcopy(Path_planning)
	cost_list =[]
	temprature_list=[]
	k=1
	while(i<i_max_pp):
		print("*********************************************************************************************************************")
		print(" ***** For iteration %d ****" % (i+1))
		print("current temperature = ",T_current_pp)
		for j in range(0,10):
			print("--- for sub-iteration number %d ---" %(j+1))
			temp=copy.deepcopy(currentRobotList)
			Path_planning.modify()
			print("length of self.pathlist BEFORE assign total path =",len(Path_planning.path_list))
			Path_planning.assign_TotalPath()
			#print("path_planning path list modified is :")
			#Path_planning.display_path_list()
			print("length of self.pathlist AFTER assign total path =",len(Path_planning.path_list))
			new_cost_path_planning = Path_planning.get_cost_MA_Path_Planning(obstacle_list)
	
			print("no of collisions = ",Path_planning.get_path_collision())
			print("new_cost_calculated = ", new_cost_path_planning)
			print("current cost_before_comparing = = ", current_cost_planning)
			delta_energy_pp = new_cost_path_planning - current_cost_planning
			print(" deltaenergy_PP =", delta_energy_pp)
			if(delta_energy_pp<0):
				print(" WE DIRECTLY ACCEPTED THE NEW SOLUTION ")
				print("length of self.pathlist before assign_total_path =",len(Path_planning.path_list))
				currentRobotList=copy.deepcopy(Path_planning.RobotList)
				Path_planning.assign_TotalPath()
				current_cost_planning=new_cost_path_planning
				print("current_cost =",current_cost_planning)
			else:
				r = numpy.random.uniform()
		
				prob_pp = numpy.exp(-1*delta_energy_pp/T_current_pp)
				print(" r = ",r)
				print( " prob_pp =", prob_pp)
				if prob_pp > r:
					print(" WE ACCEPTED THE SOLUTION BY PROBABILITY !!")
					currentRobotList=copy.deepcopy(Path_planning.RobotList)
					print("length of self.pathlist before assign_total_path =",len(Path_planning.path_list))
					Path_planning.assign_TotalPath()
					current_cost_planning=new_cost_path_planning
					print("current_cost =",current_cost_planning)
				else:
					print(" WE REJECTED THE SOLUTION !!!! ")
					print("current_cost remains =",current_cost_planning)
					Path_planning.RobotList=copy.deepcopy(temp)
					Path_planning.assign_TotalPath()
	
			if(best_cost_pp > current_cost_planning):
				best_cost_pp=current_cost_planning
				best_path=copy.deepcopy(Path_planning.path_list)
				best_pathplanning=copy.deepcopy(Path_planning)
				best_cost_iteration_number = i+1
				best_cost_subiteration_number = j+1
				print(" BEST COST UPDATED SUCCESSFULLY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
				print("Now, best cost = ",best_cost_pp)
			cost_list.append(best_cost_pp)
	
			print("So,best_cost_pp until now = ",best_cost_pp)
		temprature_list.append(T_current_pp)
		# # if(i>200):
		# 	T_current_pp=T_initial_pp*pow(0.8,k)
		# 	k+=1
		# else:
		# 	T_current_pp=T_initial_pp - Beta_pp*(i+1)
		T_current_pp=T_initial_pp*pow(0.95,i)
		i=i+1
	Path_planning.path_list = copy.deepcopy(best_path)
	print("*********************************************************************************************************************")
	print("Final best cost_pp =",best_cost_pp)
	print("best_cost_pp is obtained in iteration no: %d , in sub-iteration %d " % (best_cost_iteration_number ,best_cost_subiteration_number))
	print("Final_best_path :")
	endtime=start_time-time.time()
	print("code finished in ",endtime)
	Path_planning.display_path_list()
	Path_planning.get_final_collision()
	best_pathplanning.draw(ax2)
	for obstacle in obstacle_list2:
		obstacle.draw(ax2)
	ax2.legend(loc='upper left')

	ax2.set_xlabel('X(m)')
	ax2.set_ylabel('Y(m)')
	ax2.set_title('Optimized Path')
	
	fig2 = plt.figure()
	plt.ylabel("Best Cost")
	plt.xlabel("Iterations")
	plt.plot(cost_list)
	plt.show()

	fig3=plt.figure()
	plt.ylabel("temperature")
	plt.xlabel("iterations")
	plt.plot(temprature_list)
	plt.show()


	#best_pathplanning.draw(ax2)
	#for obstacle in obstacle_list2:
	#	obstacle.draw(ax2)
	#ax2.legend(loc='upper left')
	#ax2.set_xlabel('X(m)')
	#ax2.set_ylabel('Y(m)')
	#ax2.set_title('Optimized Path')
		#fig2 = plt.figure()
	#plt.ylabel("Best Cost Path Planning")
	#plt.xlabel("Iterations")
	#plt.plot(cost_list)
	#fig4 = plt.figure()
	#plt.ylabel("Best Cost TA")
	#plt.xlabel("Iterations")
	#plt.plot(bestcost_list_TA)
	#fig3=plt.figure()
	#plt.ylabel("temprature")
	#plt.xlabel("iterations")
	#plt.plot(temprature_list)
	#plt.show()
	#fig5=plt.figure()
	#plt.ylabel("temprature")
	#plt.xlabel("iterations")
	#plt.plot(temprature_list_TA)
	#plt.show()









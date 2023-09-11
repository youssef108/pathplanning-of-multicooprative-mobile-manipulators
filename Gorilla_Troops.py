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
import random
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
	def modify_UnkownLocation(self):
		r=[]
		for i in range(self.length*2): # to cover random change in x and y coordinates of each point in the path
			r.append(numpy.random.uniform())
		i=0
		j=1	
		while(i<2*self.length):
			self.path[j].x=(100-0)*r[i]+0
			self.path[j].y=(100-0)*r[i+1]+0
			j+=1
			i+=2
	def modify_moveToGorilla(self,maxiterations,iteration,Xrand):
		r4=[]
		r2=[]
		for i in range(self.length*2): # to cover random change in x and y coordinates of each point in the path
			r4.append(numpy.random.uniform())
			r2.append(numpy.random.uniform())
		F=[]
		for i in range(self.length*2):
			F.append(numpy.cos(r4[i]*2)+1)
		C=[]
		for i in range(self.length*2):
			C.append(F[i]*(1-(iteration/maxiterations)))
		l=[]
		for i in range(self.length*2):
			l.append(2*(random.random())-1)
		L=[]
		for i in range(self.length*2):
			L.append(C[i]*l[i])
		Z=[]
		for i in range(self.length*2):
			Z.append(2*C[i]*random.random()-C[i])
		H=[]
		i=0
		j=0
		while(i<self.length*2):
			H.append(Z[i]*self.path[j].x)
			H.append(Z[i+1]*self.path[j].y)
			j+=1
			i+=2
		i=0
		j=1

		while(i<self.length*2):

			self.path[j].x=((r2[i]-C[i])*Xrand.path[j].x) + L[i]*H[i]
			while((self.path[j].x < 0) or (self.path[j].x > 100)):
				if(self.path[j].x < 0):
					self.path[j].x = self.path[j].x + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
				else:
					self.path[j].x = self.path[j].x - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
			self.path[j].y=((r2[i+1]-C[i+1])*Xrand.path[j].y) + L[i+1]*H[i+1]
			while((self.path[j].y < 0) or (self.path[j].y > 100)):
				if(self.path[j].y < 0):
					self.path[j].y = self.path[j].y + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
				else:
					self.path[j].y = self.path[j].y - (numpy.random.uniform())*5
			i+=2
			j+=1
	def modify_KnownPosition(self,maxiterations,iteration,GXrand):
		r4=[]
		r3=[]
		for i in range(self.length*2): # to cover random change in x and y coordinates of each point in the path
			r4.append(numpy.random.uniform())
			r3.append(numpy.random.uniform())
		F=[]
		for i in range(self.length*2):
			F.append(numpy.cos(r4[i]*2)+1)
		C=[]
		for i in range(self.length*2):
			C.append(F[i]*(1-(iteration/maxiterations)))
		l=[]
		for i in range(self.length*2):
			l.append(2*(random.random())-1)
		L=[]
		for i in range(self.length*2):
			L.append(C[i]*l[i])
		i=0
		j=1
		while(i<self.length*2):

			self.path[j].x=self.path[j].x-L[i]*(L[i]*(self.path[j].x-GXrand.path[j].x))+r3[i]*(self.path[j].x-GXrand.path[j].x)
			while((self.path[j].x < 0) or (self.path[j].x > 100)):
				if(self.path[j].x < 0):
					self.path[j].x = self.path[j].x + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
				else:
					self.path[j].x = self.path[j].x - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
			self.path[j].y=self.path[j].y-L[i+1]*(L[i+1]*(self.path[j].y-GXrand.path[j].y))+r3[i+1]*(self.path[j].y-GXrand.path[j].y)
			while((self.path[j].y < 0) or (self.path[j].y > 100)):
				if(self.path[j].y < 0):
					self.path[j].y = self.path[j].y + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
				else:
					self.path[j].y = self.path[j].y - (numpy.random.uniform())*5
			i+=2
			j+=1
	def modify_CompititionForFemales(self,beta,N1,N2,XsilverBack):
		A=[]
		Q=[]
		for i in range(self.length*2): # to cover random change in x and y coordinates of each point in the path
			Q.append(2*numpy.random.uniform()-1)
			if(numpy.random.uniform()>=0.5):
				A.append(beta*N1)
			else:
				A.append(beta*N2)
		i=0
		j=1
		while(i<self.length*2):

			self.path[j].x=XsilverBack.path[j].x - (XsilverBack.path[j].x*Q[i] - self.path[j].x*Q[i])*A[i]
			while((self.path[j].x < 0) or (self.path[j].x > 100)):
				if(self.path[j].x < 0):
					self.path[j].x = self.path[j].x + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
				else:
					self.path[j].x = self.path[j].x - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
			self.path[j].y=XsilverBack.path[j].y - (XsilverBack.path[j].y*Q[i+1] - self.path[j].y*Q[i+1])*A[i+1]
			while((self.path[j].y < 0) or (self.path[j].y > 100)):
				if(self.path[j].y < 0):
					self.path[j].y = self.path[j].y + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
				else:
					self.path[j].y = self.path[j].y - (numpy.random.uniform())*5
			i+=2
			j+=1
	def modify_FollowSilverback(self,maxiterations,iteration,M,Xsilverback):
		r4=[]
		for i in range(self.length*2): # to cover random change in x and y coordinates of each point in the path
			r4.append(numpy.random.uniform())
		F=[]
		for i in range(self.length*2):
			F.append(numpy.cos(r4[i]*2)+1)
		C=[]
		for i in range(self.length*2):
			C.append(F[i]*(1-(iteration/maxiterations)))
		l=[]
		for i in range(self.length*2):
			l.append(2*(random.random())-1)
		L=[]
		for i in range(self.length*2):
			L.append(C[i]*l[i])	
		g=[]
		for i in range(self.length*2):
			g.append(pow(2,L[i]))
		
		for i in range(self.length*2):
			M[i]=pow(abs(pow(M[i],g[i])),(1/g[i]))
		
		i=0
		j=1
		while(i<self.length*2):

			self.path[j].x=L[i]*M[i]*(self.path[j].x-Xsilverback.path[j].x)+self.path[j].x
			while((self.path[j].x < 0) or (self.path[j].x > 100)):
				if(self.path[j].x < 0):
					self.path[j].x = self.path[j].x + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
				else:
					self.path[j].x = self.path[j].x - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
			self.path[j].y=L[i+1]*M[i+1]*(self.path[j].y-Xsilverback.path[j].y)+self.path[j].y
			while((self.path[j].y < 0) or (self.path[j].y > 100)):
				if(self.path[j].y < 0):
					self.path[j].y = self.path[j].y + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
				else:
					self.path[j].y = self.path[j].y - (numpy.random.uniform())*5
			i+=2
			j+=1
		


			

	def modify_EncirclingPrey(self,a,X_best):
		print("Encircling started")
		j=1
		r=[]
		for i in range(self.length*2): # to cover random change in x and y coordinates of each point in the path
			r.append(numpy.random.uniform())
		A=[]
		for i in range(self.length*2):
			A.append(2*a*r[i]-a)
		# A_temp=numpy.array(copy.deepcopy(A))
		# A_mag=numpy.linalg.norm(copy.deepcopy(A_temp))
		A_mag=0
		for i in range(len(A)):
			A_mag+=pow(A[i],2)
		A_mag=numpy.sqrt(A_mag)
		while(A_mag>=1):
			r=[]
			for i in range(self.length*2):
				r.append(numpy.random.uniform())
			A=[]
			for i in range(self.length*2):
				A.append(2*a*r[i]-a)
			A_mag=0
			for i in range(len(A)):
				A_mag+=pow(A[i],2)
			A_mag=numpy.sqrt(A_mag)
		
		C=[]
		for i in range(self.length*2):
			C.append(2*r[i])

		# now modify x and y coordinates of the points 
		D=[]
		i=0
		while(i<2*(self.length)):
			D.append(abs(C[i]*(X_best.path[j].x)-self.path[j].x))
			D.append(abs(C[i+1]*(X_best.path[j].y)-self.path[j].y))
			i=i+2
			j=j+1

		i=0
		j=1
		while(i<2*(self.length)): # loop on size of vector D and A
			self.path[j].x = ( X_best.path[j].x ) -A[i]*D[i]

			while((self.path[j].x < 0) or (self.path[j].x > 100)):
				if(self.path[j].x < 0):
					self.path[j].x = self.path[j].x + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
				else:
					self.path[j].x = self.path[j].x - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)		
			self.path[j].y = ( X_best.path[j].y ) -A[i+1]*D[i+1]

			while((self.path[j].y < 0) or (self.path[j].y > 100)):
				if(self.path[j].y < 0):
					self.path[j].y = self.path[j].y + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
				else:
					self.path[j].y = self.path[j].y - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
			i=i+2
			j=j+1
		print("Encircling finished")

	def modify_SearchforPrey(self,a,X_rand):
		print("search_for_prey started")
		j=1
		r=[]
		for i in range(self.length*2): # to cover random change in x and y coordinates of each point in the path
			r.append(numpy.random.uniform())
		A=[]
		for i in range(self.length*2):
			A.append(2*a*r[i]-a)
		# A_temp=numpy.array(copy.deepcopy(A))
		# A_mag=numpy.linalg.norm(copy.deepcopy(A_temp))
		A_mag=0
		for i in range(len(A)):
			A_mag+=pow(A[i],2)
		A_mag=numpy.sqrt(A_mag)
		while(A_mag<1):
			r=[]
			for i in range(self.length*2):
				r.append(numpy.random.uniform())
			A=[]
			for i in range(self.length*2):
				A.append(2*a*r[i]-a)
			A_mag=0
			for i in range(len(A)):
				A_mag+=pow(A[i],2)
			A_mag=numpy.sqrt(A_mag)
		
		C=[]
		for i in range(self.length*2):
			C.append(2*r[i])

		# now modify x and y coordinates of the points 
		D=[]
		i=0
		while(i<2*(self.length)):
			D.append(abs(C[i]*(X_rand.path[j].x)-self.path[j].x))
			D.append(abs(C[i+1]*(X_rand.path[j].y)-self.path[j].y))
			i=i+2
			j=j+1

		j=1
		i=0
		while(i<2*(self.length)): # loop on size of vector D and A
			self.path[j].x = (X_rand.path[j].x) -A[i]*D[i]

			while((self.path[j].x < 0) or (self.path[j].x > 100)):
				if(self.path[j].x < 0):
					self.path[j].x = self.path[j].x + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
				else:
					self.path[j].x = self.path[j].x - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
			self.path[j].y = ( X_rand.path[j].y ) -A[i+1]*D[i+1]

			while((self.path[j].y < 0) or (self.path[j].y > 100)):
				if(self.path[j].y < 0):
					self.path[j].y = self.path[j].y + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
				else:
					self.path[j].y = self.path[j].y - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
			i=i+2
			j=j+1
		print("search_for_prey finished")

	def modify_spiral(self,a,X_best,b,l):
		print("spiral started")
		j=1
		D=[]
		i=0
		while(i<2*(self.length)):
			D.append(abs(X_best.path[j].x - self.path[j].x))
			D.append(abs(X_best.path[j].y - self.path[j].y))
			i=i+2
			j=j+1
		i=0
		j=1
		while(i<2*(self.length)):
			self.path[j].x  = D[i]*(numpy.exp(b*l))*(numpy.cos(2*numpy.pi*l)) + X_best.path[j].x

			while((self.path[j].x < 0) or (self.path[j].x > 100)):
				if(self.path[j].x < 0):
					self.path[j].x = self.path[j].x + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
				else:
					self.path[j].x = self.path[j].x - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].x)
			self.path[j].y  = D[i+1]*(numpy.exp(b*l))*(numpy.cos(2*numpy.pi*l)) + X_best.path[j].y

			while((self.path[j].y < 0) or (self.path[j].y > 100)):
				if(self.path[j].y < 0):
					self.path[j].y = self.path[j].y + (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
				else:
					self.path[j].y = self.path[j].y - (numpy.random.uniform())*5
					#print("In Feasibility", self.path[j].y)
			i=i+2
			j=j+1
		print("spiral finished") 	



	def display_path_object(self):
		for i in range(len(self.path)):
			print(self.path[i].get_xy())

class Gorilla_MAPath:
	def __init__(self,Robot_list,length,populationsize,obstacles,maxIterations,N1,N2):
		self.obstacles=obstacles
		self.Robot_list=Robot_list
		self.length=length
		self.populationsize=populationsize
		self.population=[]
		self.candidate_population=[]
		self.beta=3
		self.p=0.03
		# self.W=0.8
		self.maxiterations=maxIterations
		self.iteration=0
		self.N1=N1
		self.N2=N2
		
		print("I will enter the loop ")
		for i in range(populationsize):
			print("iteration ,",i)
			new_path = MAPath(copy.deepcopy(Robot_list),length)
			new_path_2 =copy.deepcopy(new_path)
			#new_path.display_path_list()
			self.population.append(new_path_2)
		self.candidate_population=copy.deepcopy(self.population)
		# self.assign_path_list_of_GA_Mapath()
	def modify_UnkownLocation(self,i):
		self.candidate_population[i].modify_UnkownLocation()
	def modify_moveToGorilla(self,i):
		r_rand=numpy.random.randint(0,len(self.population))
		while(r_rand==i):
			r_rand=numpy.random.randint(0,len(self.population))
		X_rand=copy.deepcopy(self.population[r_rand])
		self.candidate_population[i].modify_moveToGorilla(self.maxiterations,self.iteration,copy.deepcopy(X_rand))
	def modify_KnownPosition(self,i):
		r_rand=numpy.random.randint(0,len(self.population))
		while(r_rand==i):
			r_rand=numpy.random.randint(0,len(self.population))
		GX_rand=copy.deepcopy(self.candidate_population[r_rand])
		self.candidate_population[i].modify_KnownPosition(self.maxiterations,self.iteration,copy.deepcopy(GX_rand))
	def modify_CompititionForFemales(self,i):
		XsilverBack=copy.deepcopy(self.population[0])
		self.candidate_population[i].modify_CompititionForFemales(self.beta,self.N1,self.N2,copy.deepcopy(XsilverBack))
	def modify_FollowSilverback(self,i):    #to be continued 
		XsilverBack=copy.deepcopy(self.population[0])
		for z in range(len(self.Robot_list)):
			for j in range(len(self.Robot_list[z].paths_list)):
				M=[]
				for k in range(1,len(self.Robot_list[z].paths_list[j].path)-1):
					sum_x=0
					sum_y=0
					for p in range(self.populationsize):
						sum_x+=self.candidate_population[p].RobotList[z].paths_list[j].path[k].x
						sum_y+=self.candidate_population[p].RobotList[z].paths_list[j].path[k].y
					Average_x=sum_x/self.populationsize
					Average_y=sum_y/self.populationsize
					M.append(Average_x)
					M.append(Average_y)
				self.candidate_population[i].RobotList[z].paths_list[j].modify_FollowSilverback(self.maxiterations,self.iteration,copy.deepcopy(M),XsilverBack.RobotList[z].paths_list[j])
	def Compare(self):
		for i in range(self.populationsize):
			if(self.candidate_population[i].cost_mapath<self.population[i].cost_mapath):
				self.population[i]=copy.deepcopy(self.candidate_population[i])
						
	# def modify_SearchforPrey(self,i):
	# 	r_rand=numpy.random.randint(0,len(self.population))
	# 	while(r_rand==i):
	# 		r_rand=numpy.random.randint(0,len(self.population))
	# 	X_rand=copy.deepcopy(self.old_population[r_rand])
	# 	self.population[i].modify_SearchforPrey(self.a,X_rand)

	def sort_the_population (self):
		self.population=sorted(self.population,key=lambda x: x.cost_mapath)

	# def modify_EncirclingPrey(self,i):
	# 	self.sort_the_population()
	# 	self.population[i].modify_EncirclingPrey(self.a,copy.deepcopy(self.old_population[0]))

	# def modify_spiral(self,i):
	# 	self.sort_the_population()
	# 	self.population[i].modify_spiral(self.a,copy.deepcopy(self.old_population[0]),self.b)	
	def assign_total_path_to_candidate_population(self):
		for i in range(self.populationsize):
			#temp_ind = copy.deepcopy(self.population[i])
			#temp_ind.assign_TotalPath()
			#self.population[i]=copy.deepcopy(temp_ind)
			for k in range(len(self.candidate_population[i].RobotList)):
				temp=[]
				for j in range(len(self.candidate_population[i].RobotList[k].paths_list)):
					temp.extend(self.candidate_population[i].RobotList[k].paths_list[j].path[:-1])
				temp.append(self.candidate_population[i].RobotList[k].paths_list[len(self.candidate_population[i].RobotList[k].paths_list)-1].path[len(self.candidate_population[i].RobotList[k].paths_list[len(self.candidate_population[i].RobotList[k].paths_list)-1].path)-1]) 
				self.candidate_population[i].RobotList[k].TotalPath=Path(None,None,None,copy.deepcopy(temp))
		# Update the self.path_list
			self.candidate_population[i].path_list = []
			for l in range(len(self.candidate_population[i].RobotList)):
				path = self.candidate_population[i].RobotList[l].TotalPath
				self.candidate_population[i].path_list.append(path)
	def assign_total_path_to_population(self):
		for i in range(self.populationsize):
			#temp_ind = copy.deepcopy(self.population[i])
			#temp_ind.assign_TotalPath()
			#self.population[i]=copy.deepcopy(temp_ind)
			for k in range(len(self.population[i].RobotList)):
				temp=[]
				for j in range(len(self.population[i].RobotList[k].paths_list)):
					temp.extend(self.population[i].RobotList[k].paths_list[j].path[:-1])
				temp.append(self.population[i].RobotList[k].paths_list[len(self.population[i].RobotList[k].paths_list)-1].path[len(self.population[i].RobotList[k].paths_list[len(self.population[i].RobotList[k].paths_list)-1].path)-1]) 
				self.population[i].RobotList[k].TotalPath=Path(None,None,None,copy.deepcopy(temp))
		# Update the self.path_list
			self.population[i].path_list = []
			for l in range(len(self.population[i].RobotList)):
				path = self.population[i].RobotList[l].TotalPath
				self.population[i].path_list.append(path)
	def assign_cost_to_population(self):
		print("size of population on assigning the cost is ", len(self.population))
		for i in range(len(self.population)):
			#temp = copy.deepcopy(self.population[i])
			#temp.get_cost_MA_Path_Planning(self.obstacles)
			#self.population[i]=copy.deepcopy(temp)
			cost = self.population[i].get_cost_MA_Path_Planning(self.obstacles)
			print("cost assigned to MApath no :", i)
			print("cost returned = ",cost)
			print("cost of corresponding  ind =",self.population[i].cost_mapath)
	def assign_cost_to_candidate_population(self):
		print("size of population on assigning the cost is ", len(self.candidate_population))
		for i in range(len(self.candidate_population)):
			#temp = copy.deepcopy(self.population[i])
			#temp.get_cost_MA_Path_Planning(self.obstacles)
			#self.population[i]=copy.deepcopy(temp)
			cost = self.candidate_population[i].get_cost_MA_Path_Planning(self.obstacles)
			print("cost assigned to MApath no :", i)
			print("cost returned = ",cost)
			print("cost of corresponding  ind =",self.candidate_population[i].cost_mapath)		
			
			#self.population[i].display_path_list()
		print("finished assigning cost ")

	def display_sorted_costs(self):
		print("sorted costs :")
		for i in range(len(self.population)):
			#self.population[i].display_path_list()
			print("for MApath no %d the new cost equals %d"%(i,self.population[i].cost_mapath))
	
	def get_best_MA_GTO(self):
		return copy.deepcopy(self.population[0])
	
	def modify_exploration(self):
		for i in range(len(self.population)):
			r=numpy.random.uniform()
			if(r<self.p):
				self.modify_UnkownLocation(i)
			elif(r>=0.5):
				self.modify_moveToGorilla(i)
			elif(r<0.5):
				self.modify_KnownPosition(i)
	def modify_exploitation(self):
		for i in range(len(self.population)):
			r4=[]
			for i in range(self.length*2): # to cover random change in x and y coordinates of each point in the path
				r4.append(numpy.random.uniform())
			F=[]
			for i in range(self.length*2):
				F.append(numpy.cos(r4[i]*2)+1)
			C=[]
			for i in range(self.length*2):
				C.append(F[i]*(1-(self.iteration/self.maxiterations)))
			C_mag=0
			for i in range(len(C)):
				C_mag+=pow(C[i],2)
			C_mag=numpy.sqrt(C_mag)
			if(C_mag>=0.8):
				self.modify_FollowSilverback(i)
			else:
				self.modify_CompititionForFemales(i)
			
	
	def modify(self):
		print("modify new MA_path")
		print("size of population =",len(self.population))
		print("size of old population =",len(self.old_population))
		for i in range(len(self.population)):
			print(" looping on the population !")
			p=numpy.random.uniform()
			if(p >= 0.5):
				print(" I will perform modify spiral !")
				self.modify_spiral(i)
				print(" I finished modify spiral !")
			else:
				r=[]
				#no_of_paths = 0
				#for j in range(len(self.Robot_list)):
				#	no_of_paths = no_of_paths + len(self.Robot_list[j].tasks) # no of paths = no of tasks
				#no_of_cordinates = no_of_paths * self.length * 2      # no_of_paths * no of points in each path in between * 2 coordinates x,y
				for k in range(2*self.length):
					r.append(numpy.random.uniform())
				
				A=[]
				print("value of a = ",self.a)
				for m in range(2*self.length):
					A.append(2*self.a*r[m]-self.a)
					print("Value appended in A:",A[m])
				A_mag=0
				for i in range(len(A)):
					A_mag+=pow(A[i],2)
				A_mag=numpy.sqrt(A_mag)
				print("before modify : magnitude A ",A_mag)
				if(A_mag<1):
					print(" I will perform modify encircling !")
					self.modify_EncirclingPrey(i)
					print(" I finished modify encircling !")
				else:
					print(" I will perform modify search for prey !")
					self.modify_SearchforPrey(i)
					print(" I finished modify search for prey !")
					
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
	def modify_UnkownLocation(self):
		for i in range(len(self.RobotList)):
			for j in range(len(self.RobotList[i].paths_list)):
				temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
				temp_path.modify_UnkownLocation()
				self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)
	def modify_moveToGorilla(self,maxiterations,iteration,Xrand):
		for i in range(len(self.RobotList)):
			for j in range(len(self.RobotList[i].paths_list)):
				temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
				temp_path.modify_moveToGorilla(maxiterations,iteration,Xrand.RobotList[i].paths_list[j])
				self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)
	def modify_KnownPosition(self,maxiterations,iteration,GXrand):
		for i in range(len(self.RobotList)):
			for j in range(len(self.RobotList[i].paths_list)):
				temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
				temp_path.modify_KnownPosition(maxiterations,iteration,GXrand.RobotList[i].paths_list[j])
				self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)
	def modify_CompititionForFemales(self,beta,N1,N2,XsilverBack):
		for i in range(len(self.RobotList)):
			for j in range(len(self.RobotList[i].paths_list)):
				temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
				temp_path.modify_CompititionForFemales(beta,N1,N2,XsilverBack.RobotList[i].paths_list[j])
				self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)
	# def modify_FollowSilverback(self,maxiterations,iterations,candidate_population,Xsilverback):
	# 	for i in range(len(self.RobotList)):
	# 		for j in range(len(self.RobotList[i].paths_list)):
	# 			temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
	# 			temp_path.modify_FollowSilverback(maxiterations,iterations,candidate_population)
	# 			self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)
	# def modify_EncirclingPrey(self,a,X_best):
	# 	for i in range(len(self.RobotList)):
	# 		for j in range(len(self.RobotList[i].paths_list)):
	# 			temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
	# 			temp_path.modify_EncirclingPrey(a,X_best.RobotList[i].paths_list[j])
	# 			self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)
	# def modify_SearchforPrey(self,a,X_rand):
	# 	for i in range(len(self.RobotList)):
	# 		for j in range(len(self.RobotList[i].paths_list)):
	# 			temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
	# 			temp_path.modify_SearchforPrey(a,X_rand.RobotList[i].paths_list[j])
	# 			self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)
	# def modify_spiral(self,a,X_best,b):
	# 	l=2*(random.random())-1
	# 	for i in range(len(self.RobotList)):
	# 		for j in range(len(self.RobotList[i].paths_list)):
	# 			temp_path =copy.deepcopy(self.RobotList[i].paths_list[j])
	# 			temp_path.modify_spiral(a,X_best.RobotList[i].paths_list[j],b,l)
	# 			self.RobotList[i].paths_list[j]=copy.deepcopy(temp_path)
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

	#####################	TASK ALLOCATION GA ALGORITHM 	#####################

	print ("************************************************** TASK ALLOCATION RESULTS ************************************************** ")

	task_list= [Task(My_Point(15,80),120),Task(My_Point(40,40),400),Task(My_Point(50,70),100),Task(My_Point(80,90),120),Task(My_Point(90,10),50),
	Task(My_Point(75,60),70),Task(My_Point(75,30),50),Task(My_Point(90,30),50),Task(My_Point(45,20),70)]
	
	Robot_list=[Robot(My_Point(10,45)),Robot(My_Point(15,45))]
	
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
	#fig2 = plt.figure()
	#plt.ylabel("Best Cost TA")
	#plt.xlabel("Iterations")
	#plt.plot(best_costlist)
	#plt.show()


	#####################	Obstacles Plot ####################
	

	fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
	

	ax1.legend(loc='upper left')
	ax1.set_xlabel('X(m)')
	ax1.set_ylabel('Y(m)')
	ax1.set_title('Initial Path')
	
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
	#obstacle_list2.append(rect33)
    
	for obstacle in obstacle_list:
		obstacle.draw(ax1)


	####################	PATH PLANNING GTO ALGORITHM ####################
	i_max=350
	i=0
	#update
	N1=4
	N2=3
	GTO_Path_Planning = Gorilla_MAPath(best_taskAllocation.Robot_list,2,500,obstacle_list,i_max,N1,N2)	

	GTO_Path_Planning.assign_total_path_to_population()	
	print ("ALL paths are assigned using assign_total_path")
	GTO_Path_Planning.assign_cost_to_population()
	print("before iterations finished assign costs !")
	GTO_Path_Planning.sort_the_population()
	print("before iterations finished sorting !")
	GTO_Path_Planning.display_sorted_costs()
	Best_MA_PATH=GTO_Path_Planning.get_best_MA_GTO()
	print("before iterations , got the current best cost")
	Best_Cost_pp=Best_MA_PATH.cost_mapath
	print("initial best cost", Best_Cost_pp)
	GTO_Path_Planning.get_best_MA_GTO().draw(ax1)		

	cost_list=[]
	W=0.8
	while(i<i_max):
		print("For PP iteration ",i)
		GTO_Path_Planning.iteration=i
		print("self .a =",GTO_Path_Planning.iteration)
		GTO_Path_Planning.candidate_population=copy.deepcopy(GTO_Path_Planning.population)
		print("copied the pop")
		GTO_Path_Planning.modify_exploration()
		print("modification done !")
		GTO_Path_Planning.assign_total_path_to_candidate_population()	
		print ("ALL paths are assigned using assign_total_path")
		GTO_Path_Planning.assign_cost_to_candidate_population()
		GTO_Path_Planning.Compare()
		GTO_Path_Planning.sort_the_population()
		GTO_Path_Planning.candidate_population=copy.deepcopy(GTO_Path_Planning.population)
		GTO_Path_Planning.modify_exploitation()
		GTO_Path_Planning.Compare()
		GTO_Path_Planning.sort_the_population()
		GTO_Path_Planning.display_sorted_costs()
		#Best_MA_PATH=GTO_Path_Planning.get_best_MA_GTO()
		current_best_cost_pp = GTO_Path_Planning.population[0].cost_mapath
		if(current_best_cost_pp < Best_Cost_pp):
			Best_MA_PATH=GTO_Path_Planning.get_best_MA_GTO()
			Best_Cost_pp = current_best_cost_pp
		print("current best cost PP = ",Best_Cost_pp)
		cost_list.append(Best_Cost_pp)
		i=i+1
	print("best path is :")
	Best_MA_PATH.display_path_list()
	print(" And its best cost = ",Best_MA_PATH.cost_mapath)
	print("The best cost =",Best_Cost_pp)


	Best_MA_PATH.draw(ax2)
	for obstacle in obstacle_list2:
		obstacle.draw(ax2)
	ax2.legend(loc='upper left')
#
	ax2.set_xlabel('X(m)')
	ax2.set_ylabel('Y(m)')
	ax2.set_title('Optimized Path')
	
	fig2 = plt.figure()
	plt.ylabel("Best Cost")
	plt.xlabel("Iterations")
	plt.plot(cost_list)
	

	plt.show()
	




	

	








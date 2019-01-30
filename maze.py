import numpy as np
import sys
from solvers import solve
#Maze generation strategy:
'''
sample maze:
__________
|__|v  s_|
|v  <_|__|
|v |__|__|
|e_|__|__|
'''
class maze(object):
	def __init__(self,rows,cols):
		self.rows=rows
		self.cols=cols
		self.states=[] #Initialize an empty dictionary with fields: name and connections
		print("Initialized maze")
	def genwalls(self):
		self.walls=np.random.randint(2,size=(self.rows,2*self.cols))
		print("Generated walls ")
	def gridcount(self,a,b): #generalizes cell numbering so that it can work in non square mazes too
		counter=0
		for i in range(self.rows):
			for j in range(self.cols):
				if(i==a and j==b):
					return counter
				counter+=1
		return -1 #equivalent to cell does not exist
	def gengrid(self): #Generates states and also prints the maze
		for i in range(self.rows):
			if(i==0):
				for k in range(self.cols):
					sys.stdout.write("___")
				print(" ")
			for j in range(self.cols):
				cell={}
				connections=[]
				#cell["name "]=i*self.rows+j
				cell["name "]=self.gridcount(i,j)

				if(self.walls[i,2*j]==1): #Wall exists
					sys.stdout.write("|")
				if(self.walls[i,2*j]==0): #Wall does not exist
					sys.stdout.write(" ")
					#if(i>0 and j-1>0 and i<self.rows and j-1<self.cols): #Calculation lies within the maze
					if(self.gridcount(i,j-1)>=0): #Calculation lies within the maze
						#connections.append(i*self.rows+j-1) #Current cell is connected to immediate left cell
						connections.append(self.gridcount(i,j-1)) #Current cell is connected to immediate left cell
				if(self.walls[i,2*j+1]==1):
					sys.stdout.write("__")
				if(self.walls[i,2*j+1]==0):
					sys.stdout.write("  ")
					#if(i+1>0 and j>0 and i+1<self.rows and j<self.cols): #Calculation lies within the maze
					if(self.gridcount(i+1,j)>=0): #Calculation lies within the maze
						#connections.append((i+1)*self.rows+j) #Current cell is connected to bottom cell
						connections.append(self.gridcount(i+1,j)) #Current cell is connected to bottom cell
				#Other connections
				try:
					if(self.walls[i-1,2*j+1]==0): #ceiling
						#if(i-1>0 and j>0 and i-1<self.rows and j<self.cols): #Calculation lies within the maze
						if(self.gridcount(i-1,j)>=0): #Calculation lies within the maze
							#connections.append((i-1)*self.rows+j) #Current cell is connected to upwards cell
							connections.append(self.gridcount(i-1,j)) #Current cell is connected to upwards cell
				except:
					pass
				try:
					if(self.walls[i,2*j+2]==0): #Right wall
						#if(i>0 and j+1>0 and i<self.rows and j+1<self.cols): #Calculation lies within the maze
						if(self.gridcount(i,j+1)>=0): #Calculation lies within the maze
							#connections.append(i*self.rows+j+1) #Current cell is connected to immediate right cell
							connections.append(self.gridcount(i,j+1)) #Current cell is connected to immediate right cell
				except:
					pass
				cell["connections "]=connections
				self.states.append(cell)
				if(j==self.cols-1):
					sys.stdout.write("|")
			print(" ")
def main():
	m=maze(10,15) #Number of rows and number of columns
	m.genwalls()
	m.gengrid() #Prints the maze as well as generates the states
	#for i in range(len(m.states)):
		#print(m.states[i])
	#s=solve(m.states,1,5,"dfs")
	s=solve(m.states,m.cols,1,5,"astar") #arguments -- state space of the maze, number of columns, start node number, goal node number, algorithm name
	'''
	try:
		#s=solve(m.states,m.cols,1,5,"bfs")
		s=solve(m.states,m.cols,1,5,"dfs")
	except:
		print("Generated maze is very complex.. this happens rarely, please try again")
	'''

if __name__ == '__main__':
	main()
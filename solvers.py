import numpy as np
import sys
import copy

class solve(object):
	def __init__(self,state_space,num_cols,start,goal,technique): #num_cols is used by the heuristic for a*
		self.start=start
		self.num_cols=num_cols
		self.goal=goal
		self.state_space=state_space
		self.path=[]
		self.exact_path=[]
		self.explored=[]
		print("Initialized solver, using technique ",technique)
		if(technique=="bfs"):
			stack=[]
			stack.append(state_space[start])
			self.BFS(stack)
			if(goal not in self.path):
				print("WARNING ! a valid path does not exist, printing all expanded nodes")
			else:
				print("CONGRATULATIONS ! A path exists, printing all expanded nodes")
			print(self.path)
		if(technique=="dfs"):
			stack=[]
			stack.append(state_space[start])
			self.DFS(stack)
			if(goal not in self.path):
				print("WARNING ! a valid path does not exist, printing all expanded nodes")
			else:
				print("CONGRATULATIONS ! A path exists, printing all expanded nodes")
			print(self.path)
		if(technique=="astar"):
			stack=[]
			#Modifying the state_space
			for i in state_space:
				i["g"]=0
				i["h"]=0
			state_space[start]["parent "]=-1 #To denote that it is the root
			stack.append(state_space[start])
			self.Astar(stack)
			if(goal not in self.path):
				print("WARNING ! a valid path does not exist, printing all expanded nodes")
				print(self.path)
			else:
				print("CONGRATULATIONS ! A path exists, printing all expanded nodes")
				print(self.path_from_visited(self.path))
	def path_from_visited(self,path):
		p=[]
		pointer=self.goal
		while pointer!=self.start:
			p.append(pointer)
			pointer=copy.copy(self.state_space[pointer]["parent "])
		p.reverse()
		return p

	def expand_stack(self,stack):
		head=stack.pop(0)
		connects=head["connections "]
		if(connects==[]):
			return stack
		for i in connects:
			if(i not in self.explored):
				stack.append(self.state_space[i])
		return stack
	def not_worth_visiting(self,node,closed_nodes,open_nodes):
		for i in open_nodes:
			if(i["parent "]==self.state_space[node["name "]]["parent "]): 
				if(i["g"]+i["h"]<node["g"]+node["h"]):
					return True
		for i in closed_nodes:
			if(i["parent "]==self.state_space[node["name "]]["parent "]):#wrong implementation...should be : if parent(i) has same level in BFS as node
				if(i["g"]+i["h"]<node["g"]+node["h"]):
					return True
		return False
	def heuristic_cost(self,current,goal): #Just pass the state numbers here
		#return goal-current #Because the of the naming of nodes in the grid, this returns Manhattan distance
		y_dist=np.abs((goal-current)/self.num_cols)
		x_dist=np.abs((goal-current))%self.num_cols
		return y_dist+x_dist #returns Manhattan distance
	def expand_value_stack(self,stack):
		#head=stack.pop(0)
		f_vals=[]
		for i in stack:
			f_vals.append(i["g"]+i["h"])
		head=stack.pop(np.argmin(f_vals)) #pop the min f value node from the open list 
		connects=head["connections "]
		for i in connects:
			self.state_space[i]["parent "]=head["name "]
			try:
				self.state_space[i]["connections "].remove(head["name "])
			except:
				pass #Connection has been removed earlier
			self.state_space[i]["g"]=head["g"]+1
			self.state_space[i]["h"]=self.heuristic_cost(self.state_space[i]["name "],self.goal)
		if(connects==[]):
			return stack
		open_list=copy.copy(stack)
		for i in connects:
			if(self.not_worth_visiting(self.state_space[i],self.explored,open_list)==False):
				stack.append(self.state_space[i])
				#print("stack append operation ",self.state_space[i])
		#print("stack append is over")
		self.explored.append(head)
		#print("Iteration 1 stack... ",stack)
		return stack
	def explore_stack(self,stack):
		tail=stack[-1]
		connects=tail["connections "]
		if(connects==[]):
			stack.pop(-1)
			return stack
		for i in connects:
			if(i not in self.explored):
				stack.append(self.state_space[i])
				return stack
		stack.pop(-1)
		return stack
	def BFS(self,stack):
		for i in stack:
			if(self.goal==i["name "]):
				return self.goal
		if (len(self.explored)==len(self.state_space)-1):
			print("Explored everything")
			return #End search, because we have searched the entire state space and could not find the goal
		else:
			try:
				head=stack[0]
			except:
				return #End search because no path exists to the goal
			if(head["name "] not in self.explored):
				self.explored.append(head["name "])
				self.path.append(head["name "])
			self.path.append(self.BFS(self.expand_stack(stack)))
	def DFS(self,stack):
		for i in stack:
			if(self.goal==i["name "]):
				return self.goal
		if (len(self.explored)==len(self.state_space)-1):
			print("Explored everything")
			return #End search, because we have searched the entire state space and could not find the goal
		else:
			try:
				tail=stack[-1]
			except:
				return #End search because no path exists to the goal
			if(tail["name "] not in self.explored):
				self.explored.append(tail["name "])
				self.path.append(tail["name "]) #Will return an inverted path
			self.path.append(self.DFS(self.explore_stack(stack)))
	def Astar(self,stack): #A* can be built using similar structure of BFS
		for i in stack:
			if(self.goal==i["name "]):
				return self.goal
		if (len(self.explored)==len(self.state_space)-1):
			print("Explored everything")
			return #End search, because we have searched the entire state space and could not find the goal
		else:
			try:
				f_vals=[]
				for i in stack:
					f_vals.append(i["g"]+i["h"])
				head=stack[np.argmin(f_vals)] #Take the min f value node from the open list
			except:
				return #End search because no path exists to the goal
			if(head["name "] not in self.explored):
				self.path.append(head["name "])
				self.exact_path.append(head["name "])
			self.path.append(self.Astar(self.expand_value_stack(stack)))

	'''
	// A* Search Algorithm (From Geeks for Geeks)
	1.  Initialize the open list
	2.  Initialize the closed list
	    put the starting node on the open 
	    list (you can leave its f at zero)

	3.  while the open list is not empty
	    a) find the node with the least f on 
	       the open list, call it "q"

	    b) pop q off the open list
	  
	    c) generate q's 8 successors and set their 
	       parents to q
	   
	    d) for each successor
	        i) if successor is the goal, stop search
	          successor.g = q.g + distance between 
	                              successor and q
	          successor.h = distance from goal to 
	          successor (This can be done using many 
	          ways, we will discuss three heuristics- 
	          Manhattan, Diagonal and Euclidean 
	          Heuristics)
	          
	          successor.f = successor.g + successor.h

	        ii) if a node with the same position as 
	            successor is in the OPEN list which has a 
	           lower f than successor, skip this successor

	        iii) if a node with the same position as 
	            successor  is in the CLOSED list which has
	            a lower f than successor, skip this successor
	            otherwise, add  the node to the open list
	     end (for loop)
	  
	    e) push q on the closed list
	    end (while loop) 
	'''


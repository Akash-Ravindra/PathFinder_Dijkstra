from copy import copy, deepcopy
from operator import add
import numpy as np
import time
from shapely.geometry import Polygon,Point
import math
from queue import PriorityQueue

from sympy import true

'''
Custom data type to store each node
'''
class Node:
    def __init__(self,position = [0,0], parent = None, cost = np.inf,is_visited = None):
        self.__attrdict = {"position":np.array(position),"parent":np.array(parent),"cost":cost,"is_visited":is_visited }

    '''set the position of the state'''
    def set_position(self,position = np.zeros([1,2])):
        self.__attrdict['position'] = np.array(position)
    '''Get the position of the current node'''
    def get_position(self):
        return self.__attrdict['position'] 

    def set_is_visited(self,is_visited = True):
        self.__attrdict['is_visited'] = is_visited
    '''Get the position of the current node'''
    def get_is_visited(self):
        return self.__attrdict['is_visited'] 

    def set_node_parameters(self,dict):
        for keys in dict.keys():
            self.__attrdict[keys] = dict[keys]
    '''Get the index of the parent'''
    def get_parent(self):
        return self.__attrdict['parent']
    '''Set the index of the parent of the current state in the all_state list'''
    def set_parent(self, parent):
        self.__attrdict['parent'] = parent

    '''set the index of the current state'''
    def set_cost(self,cost):
        self.__attrdict['cost'] = cost
    def get_cost(self):
        return self.__attrdict['cost']

    '''Get cartisian coordinates of each node'''
    def get_cartisian(self):
        x,y = self.get_position()
        x,y = y,x
        return np.array((x,abs(y-Maze.lim_x)))


    '''Overload the == operator to compare two states'''
    def __eq__(self, node):
        return np.equal(self.get_position(),node.get_position()).all()
    '''Overload the str to print the object'''
    def __str__(self) -> str:
        return "Self: "+str(self.get_position())+" Parent: "+str(self.get_parent())+" Cost: "+str(self.get_cost())
    '''Overload the str to print the object'''
    def __repr__(self) -> str:
        return (str(self.get_position()))+'\n'
    def __gt__(self,value):
        return (self.__attrdict['cost']>value)
    def __lt__(self,value):
        return (self.__attrdict['cost']<value)


class Maze:
    arrow_obstacle = Polygon([(115,210),(80,180),(105,100),(36,185)]) ##The arrow polygon obstacle
    circle = Point((300,185)).buffer(40)
    action_set = {'u':np.array([-1,0]),'d':np.array([1,0]),\
                  'l':np.array([0,-1]),'r':np.array([0,1]),\
                  'ul':np.array([-1,-1]),'ur':np.array([-1,1]),\
                  'dl':np.array([1,-1]),'dr':np.array([1,1])} ##action set dict, used to append add to the current index
    lim_x = 249## xlimit in terms of array
    lim_y = 399## Y limit in terms of array
    def __init__(self,padding = 5,radius = 0,cost:dict = {'u':1,'d':1,'l':1,'r':1,'ur':1.4,'ul':1.4,'dr':1.4,'dl':1.4}) -> None:
        self.__maze = np.array([[Node([x,y])for y in range(Maze.lim_y+1)]
                                for x in range(Maze.lim_x+1)]
                               ,dtype=Node) ##Empty np array with X*Y Node objects
        self.__cost = cost              ## Cost for each action
        self.__padding = padding+radius ##padding for the robot
        self.__open_list = PriorityQueue()           ##Open list containing the accessable nodes
        self.__close_list = []          ##closed list containing the visited nodes
        self.path = []
        self.start_goal = []

    def solve_maze(self,start,goal):
        start = self.cartisian_to_idx(list(start))
        goal = self.cartisian_to_idx(list(goal))
        self.start_goal.append(start)
        self.start_goal.append(goal)
        self.update_maze_arrow()
        self.update_maze_circle()
        ## Check if the start and goal are accessable
        if(type(self.__maze[start])==None):
            print("Start inside a Obstacle")
            return False
        if(type(self.__maze[goal])==None):
            print("Goal inside a Obstacle")
            return False
        ## Set the cost of the start node to zero
        self.__maze[start].set_cost(0)
        ## Apppend the start node and end node into list
        self.__open_list.put(self.__maze[start])
        self.__open_list.put(self.__maze[goal])
        ##sort list, look around, pop open list and append to closed list
        while True:
            # sort the open list so that lowest cost in at idx 0
            # self.__open_list.sort()
            # print("Open : ",self.__open_list,list(map(Node.get_cost,self.__open_list)))
            # look around the first node in the open list
            NoI = self.__open_list.get()
            NoI.set_is_visited()
            if(self.__maze[self.start_goal[-1]].get_cost()<NoI.get_cost()):
                print("Found the shortest path")
                break
            if(self.__open_list.empty()):
                print("queue empyt")
                break
            print(NoI)
            self.look_around(NoI.get_position())
            # Add the first node to the closed list and pop it from open list
            self.__close_list.append(NoI)
            
            # print("Closed List: ",self.__close_list)
            
    def back_track(self):
        self.path.clear()
        self.path.append(self.__maze[self.start_goal[-1]])
        while True:
            node = self.path[-1]
            self.path.append(self.__maze[tuple(node.get_parent())])
            if(self.path[-1]==self.__maze[self.start_goal[0]]):
                break
        self.path.reverse()
        print(list(map(str,map(Node.get_position,self.path))))
        return self.path
                        
        
    ##Look around the POI
    def look_around(self,points:np.ndarray):
        #itr through all the possible actions
        actions = tuple(points+np.array(list(Maze.action_set.values())))
        for action,key in zip(actions,Maze.action_set.keys()):
            ## Check if the idx is with in maze size
            if action[0]>=0 and action[0]<=Maze.lim_x and action[1]>=0 and action[1]<=Maze.lim_y:
                ## Check if that element is a Node obj or a None(meaning it is an obstacle)
                if(self.__maze[action[0],action[1]] and not self.__maze[action[0],action[1]].get_is_visited()):
                    ### check if the node is an obstacle
                    self.add_to_list(points,action,self.__cost[key])
                    ### First time coming here, check if it is an obstacle
                    # elif(self.point_not_in_obstacle(temp)):
                    #     self.add_to_list(points,temp,self.__cost[i])
        
    ## Add to open list and update node parameters
    def add_to_list(self,parent,child,cost):
        ## If the cost is lesser than the existing cost 
        child = tuple(child)
        parent = tuple(parent)
        if((self.__maze[parent].get_cost()+cost)<self.__maze[child].get_cost()):
            ## update the cost with the new cost
            self.__maze[child].set_cost(cost+self.__maze[parent].get_cost())
            ## Set the new parent of the node
            self.__maze[child].set_parent(parent)
            
            ## Add the node to the open list
            self.__open_list.put(self.__maze[child])
            # print("open list : ",self.__open_list,list(map(Node.get_cost,self.__open_list)))
        
    ## getter for open list
    def get_open_list(self):
        return np.array(self.__open_list)
    ## getter for close list
    def get_close_list(self):
        return np.array(self.__close_list)
    def get_maze(self):
        return self.__maze
    
    ## Check if a given point is in obstacle, idx is the index
    def point_not_in_obstacle(self,idx):
        ## if the point is in circle or arrow or hexagon return false
        point = self.idx_to_cartisian(idx)
        if((self._is_in_circle(point) or self._is_in_arrow(point) or self._is_in_hexagon(point))):
            self.__maze[idx] = None
            return False
        ## set the obstacle ness of the node
        return True
    
    ## HELPER, Converts idx to cartisian
    def idx_to_cartisian(self, xy):
        xy = list(xy)
        xy[0],xy[1] = xy[1],xy[0]
        xy[1] = abs(xy[1]-Maze.lim_x)
        return (xy[0],xy[1])
    
    ## HELPER, Converts cartisian to idx
    def cartisian_to_idx(self, xy):
        xy = list(xy)
        xy[0],xy[1] = xy[1],xy[0]
        xy[0] = abs(xy[0]-Maze.lim_x)
        return (xy[0],xy[1])

    ## Defines a circle and checks if the idx is within the given circle
    def _is_in_circle(self,idx,radius=40,center=(300,185)):
        point_geom = Point(idx)
        return Maze.circle.intersects(point_geom)
    
    ## Defines a arrow and checks if the idx is within the given arrow
    def _is_in_arrow(self,idx):
        point_geom = Point((idx))
        return Maze.arrow_obstacle.intersects(point_geom)
    
    ## Defines a hexagon and checks if the idx is within the given hexagon
    def _is_in_hexagon(self,center=(200,100),side=35):
        return False
        pass
    
    def update_maze_circle(self):
        coords = Maze.circle.buffer(5).bounds
        start = ((math.floor(coords[0]),math.floor(coords[1])))
        end = ((math.floor(coords[2]),math.floor(coords[3])))
        
        x,y = np.ogrid[start[0]:end[0],start[1]:end[1]]
        
        x = x.T
        padding = int(self.__padding)
        for yi in y[0]:
            for xi in x[0]:
                if self._is_in_circle((xi,yi)):
                    idx,idy = self.cartisian_to_idx((xi,yi))
                    self.__maze[idx-padding:idx+padding,idy-padding:idy+padding] = None 
        pass
    
    
    def update_maze_arrow(self):
        coords = Maze.arrow_obstacle.buffer(5).bounds
        start = ((math.floor(coords[0]),math.floor(coords[1])))
        end = ((math.floor(coords[2]),math.floor(coords[3])))
        
        x,y = np.ogrid[start[0]:end[0],start[1]:end[1]]
        
        x = x.T
        padding = int(self.__padding)
        for yi in y[0]:
            for xi in x[0]:
                if self._is_in_arrow((xi,yi)):
                    idx,idy = self.cartisian_to_idx((xi,yi))
                    self.__maze[idx-padding:idx+padding,idy-padding:idy+padding] = None 
        pass
from copy import copy, deepcopy
from operator import add
import numpy as np
import time
from shapely.geometry import Polygon,Point

'''
Custom data type to store each node
'''
class Node:
    def __init__(self,position = [0,0], parent = [0,0], cost = np.inf,is_not_obstacle = None):
        self.__attrdict = {"position":np.array(position),"parent":np.array(parent),"cost":cost,"is_not_obstacle":is_not_obstacle }

    '''set the position of the state'''
    def set_position(self,position = np.zeros([1,2])):
        self.__attrdict['position'] = position
    '''Get the position of the current node'''
    def get_position(self):
        return self.__attrdict['position'] 

    def set_is_not_obstacle(self,obstacle = False):
        self.__attrdict['is_not_obstacle'] = obstacle
    '''Get the position of the current node'''
    def get_is_not_obstacle(self):
        return self.__attrdict['is_not_obstacle'] 

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
    action_set = {'u':[-1,0],'d':[1,0],'l':[0,-1],'r':[0,1],'ul':[-1,-1],'ur':[-1,1],'dl':[1,-1],'dr':[1,1]} ##action set dict, used to append add to the current index
    lim_x = 249## xlimit in terms of array
    lim_y = 399## Y limit in terms of array
    def __init__(self,padding = 5,radius = 0,cost:dict = {'u':1,'d':1,'l':1,'r':1,'ur':1.4,'ul':1.4,'dr':1.4,'dl':1.4}) -> None:
        self.__maze = np.array([[Node([x,y])for y in range(Maze.lim_y+1)]
                                for x in range(Maze.lim_x+1)]
                               ,dtype=Node) ##Empty np array with X*Y Node objects
        self.__cost = cost              ## Cost for each action
        self.__padding = padding+radius ##padding for the robot
        self.__open_list = []           ##Open list containing the accessable nodes
        self.__close_list = []          ##closed list containing the visited nodes
        self.path = []
        self.start_goal = []

    def solve_maze(self,start,goal):
        start = self.cartisian_to_idx(list(start))
        goal = self.cartisian_to_idx(list(goal))
        self.start_goal.append(start)
        self.start_goal.append(goal)
        ## Check if the start and goal are accessable
        if(not self.point_not_in_obstacle(start)):
            print("Start inside a Obstacle")
            return False
        if(not self.point_not_in_obstacle(goal)):
            print("Goal inside a Obstacle")
            return False
        ## Set the cost of the start node to zero
        self.__maze[start].set_cost(0)
        ## Apppend the start node and end node into list
        self.__open_list.append(self.__maze[start])
        self.__open_list.append(self.__maze[goal])
        ##sort list, look around, pop open list and append to closed list
        while True:
            # sort the open list so that lowest cost in at idx 0
            self.__open_list.sort()
            if(self.__maze[self.start_goal[-1]].get_cost()<self.__open_list[0].get_cost()):
                break
            # print("Open : ",self.__open_list,list(map(Node.get_cost,self.__open_list)))
            # look around the first node in the open list
            self.look_around(self.__open_list[0].get_position())
            # Add the first node to the closed list and pop it from open list
            self.__close_list.append(self.__open_list[0])
            self.__open_list.pop(0)
            # print("Closed List: ",self.__close_list)
            
    def back_track(self):
        self.path.clear()
        self.path.append(self.__maze[self.start_goal[-1]])
        while True:
            node = self.path[-1]
            self.path.append(self.__maze[node.get_parent])
            if(self.path[-1]==self.__maze[self.start_goal[0]]):
                break
        self.path.reverse()
        print(list(map(Node.get_position,self.path)))
                        
        
    ##Look around the POI
    def look_around(self,points):
        #itr through all the possible actions
        for i in Maze.action_set:
            ## calc the idx of the cell for a particular action
            temp = list(map(add,points,Maze.action_set[i]))
            ## Check if the idx is with in maze size
            if temp[0]>=0 and temp[0]<=Maze.lim_x and temp[1]>=0 and temp[1]<=Maze.lim_y:
                ## Check if that element is a Node obj or a None(meaning it is an obstacle)
                if(self.__maze[tuple(temp)]):
                    ### check if the node is an obstacle
                    if(self.__maze[tuple(temp)].get_is_not_obstacle()):
                        self.add_to_list(points,temp,self.__cost[i])
                    ### First time coming here, check if it is an obstacle
                    elif(self.point_not_in_obstacle(temp)):
                        self.add_to_list(points,temp,self.__cost[i])
        
    ## Add to open list and update node parameters
    def add_to_list(self,parent,child,cost):
        ## If the cost is lesser than the existing cost 
        if(self.__maze[tuple(child)] in self.__close_list):
            # print("Present", tuple(child), self.__close_list.index(self.__maze[tuple(child)]))
            return
        if((self.__maze[tuple(parent)].get_cost()+cost)<self.__maze[tuple(child)].get_cost()):
            ## update the cost with the new cost
            self.__maze[tuple(child)].set_cost(self.__maze[tuple(parent)].get_cost()+cost)
            ## Set the new parent of the node
            self.__maze[tuple(child)].set_parent(tuple(parent))
            ## Add the node to the open list
            self.__open_list.append(self.__maze[tuple(child)])
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
        if((self._is_in_circle(idx) or self._is_in_arrow(idx) or self._is_in_hexagon(idx))):
            self.__maze[idx] = None
            return False
        ## set the obstacle ness of the node
        self.__maze[tuple(idx)].set_is_not_obstacle()
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
        circle = Point(center).buffer(radius)
        point_geom = Point(self.idx_to_cartisian(idx)).buffer(self.__padding)
        return circle.intersects(point_geom)
    
    ## Defines a arrow and checks if the idx is within the given arrow
    def _is_in_arrow(self,idx):
        point_geom = Point(self.idx_to_cartisian(idx)).buffer(self.__padding)
        return Maze.arrow_obstacle.intersects(point_geom)
        pass
    
    ## Defines a hexagon and checks if the idx is within the given hexagon
    def _is_in_hexagon(self,center=(200,100),side=35):
        return False
        pass



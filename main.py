# %%
from maze import Maze
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.animation import FuncAnimation
import numpy as np
import argparse



def get_inputs():
    parser = argparse.ArgumentParser()
    parser.add_argument('-x0', type=np.int0, default=0, help="The x coordinate of the Start Node")
    parser.add_argument('-y0', type=np.int0, default=0, help="The y coordinate of the Start Node")
    parser.add_argument('-x1', type=np.int0, default=400, help="The x coordinate of the Goal Node")
    parser.add_argument('-y1', type=np.int0, default=250, help="The y coordinate of the Goal Node")
    
    args = parser.parse_args()
    start =(args.x0,args.y0)
    goal = (args.x1,args.y1)
    return [start,goal]

def main(start,goal):
    a = Maze()
    print(start)
    if (a.solve_maze(start,goal)):
        path = a.back_track()    
        a.simple_plot_path()
        a.game_plot()
    print(('-'*50)+"\n\t\tCLOSING\t\t\n"+('-'*50))
if __name__== '__main__':
    start,goal = get_inputs()
    main(start=start,goal=goal)
    



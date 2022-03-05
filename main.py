# %%
from maze import Maze
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.animation import FuncAnimation
import numpy as np
import argparse



def get_inputs():
    parser = argparse.ArgumentParser(description='Finds the path from start to goal using Dijkstra\'s algorithm',usage='\t\t\tpython main.py -x0 100 -y0 50 -x1 400 -y1 250\t\t\tSilent Operation\n\t\t\tpython main.py -x0 100 -y0 50 -x1 400 -y1 250 --output\t\tVerbose Operation ',)
    parser.add_argument('-x0', type=np.int0, default=0, help="The x coordinate of the Start Node")
    parser.add_argument('-y0', type=np.int0, default=0, help="The y coordinate of the Start Node")
    parser.add_argument('-x1', type=np.int0, default=400, help="The x coordinate of the Goal Node")
    parser.add_argument('-y1', type=np.int0, default=250, help="The y coordinate of the Goal Node")
    parser.add_argument('-o', '--output', action='store_true', help="Turns on verbose")
    
    
    args = parser.parse_args()
    start =(args.x0,args.y0)
    goal = (args.x1,args.y1)
    v = args.output
    return [start,goal,v]

def main(start,goal,verbose=False):
    a = Maze(verbose=verbose)
    if (a.solve_maze(start,goal)):
        path = a.back_track()    
        a.simple_plot_path()
        a.game_plot()
    print(('-'*50)+"\n\t\tCLOSING\t\t\n"+('-'*50))
    
if __name__== '__main__':
    start,goal,v = get_inputs()
    main(start=start,goal=goal,verbose=v)
    



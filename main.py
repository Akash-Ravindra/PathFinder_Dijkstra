# %%
from eight_puzzle import eight_Puzzle as ep
from eight_puzzle import State
import numpy as np

# %% [markdown]
# # Solve Puzzle
# The instance of eight_puzzle is created with the initial state as an input
# 
# | 1 | 2 | 3 |
# |---|---|---|
# | 4 | 5 | 6 |
# | 7 | 8 | 0 | 
# 
# The above is encoded as [1,4,7,2,5,8,3,6,0]
# 
# | 4 | 1 | 3 |
# |---|---|---|
# | 7 | 2 | 5 |
# | 0 | 8 | 6 |
# 
# Would be [4,7,0,1,2,8,3,5,6]

# %%
initial_state = [4,7,0,1,2,8,3,5,6] ##Input the initial state here ## CHANGE THIS IF NEEDED

puzzle = ep(np.array(initial_state))  ##Create instance of the puzzle object, with the standard goal

# goal_state = [1,2,3,4,5,6,7,8,0]    ##Required goal state, usually set by default,
# puzzle.set_goal_state(np.array(goal_state)) ## uncomment if the above line is changed

# %%
puzzle.solve_puzzle() ## Attempts to solve the puzzle and returns the search time

# %%
puzzle.back_track() ## Back tracks from the goal to the initial state and prints it

# %%
puzzle.save_goal_path()         ##Saves the path from initial state to the goal

# %%
puzzle.save_all_visited_nodes() ##Save all the visited nodes during search

# %%
puzzle.save_node_idx()          ##Save all the nodes index, parent index and cost



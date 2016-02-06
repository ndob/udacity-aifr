# ----------
# User Instructions:
# 
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's 
# optimal path to the position specified in goal; 
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]

cost = [2, 1, 20] # cost has 3 values, corresponding to making 
                  # a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------


def get_next(dynamic_grid, node, cost):
    value = 9999999
    turn = ' '
    print "-------------_"
    print "current:", node
    # iterate possible turns
    for i in range(len(action)):

        cur_action = (node[2] + action[i]) % len(forward)
        new_x = node[1] + forward[cur_action][1]
        new_y = node[0] + forward[cur_action][0]

        if new_x < 0 or new_x >= len(dynamic_grid[0]) or new_y < 0 or new_y >= len(dynamic_grid):
            continue

        total_cost = dynamic_grid[new_y][new_x] * cost[i]

        if total_cost < value and dynamic_grid[new_y][new_x] != 99:
            value = total_cost
            turn = [new_y, new_x, cur_action, i]

    return turn

def get_minimum_cost(dynamic_grid, x, y):
    value = 999999
    for i in range(len(forward)):
        new_x = x + forward[i][1]
        new_y = y + forward[i][0]

        if new_x < 0 or new_x >= len(dynamic_grid[0]) or new_y < 0 or new_y >= len(dynamic_grid):
            continue

        if dynamic_grid[new_y][new_x] < value:
            value = dynamic_grid[new_y][new_x]

    return value

def compute_cell_values(grid, dynamic_grid, x, y):
    for i in range(len(forward)):
        new_x = x + forward[i][1]
        new_y = y + forward[i][0]

        print  new_x, new_y
        if new_x < 0 or new_x >= len(grid[0]) or new_y < 0 or new_y >= len(grid):
            continue
        
        if grid[new_y][new_x] == 1:
            new_cost = 99
        else:
            new_cost = min(get_minimum_cost(dynamic_grid, new_x, new_y) + 1, 99)

        # if the cost doesn't change, no need to recount the neighbours
        if new_cost >= dynamic_grid[new_y][new_x]:
            continue
        else:
            dynamic_grid[new_y][new_x] = new_cost

        compute_cell_values(grid, dynamic_grid, new_x, new_y)

def compute_value(grid, init, goal, cost):
    dynamic_grid = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]

    dynamic_grid[goal[0]][goal[1]] = 0
    compute_cell_values(grid, dynamic_grid, goal[1], goal[0])    
    return dynamic_grid

def optimum_policy2D(grid, init, goal, cost):
    dynamic_grid = compute_value(grid, init, goal, cost)
    optimal_turns = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    node = init
    optimal_turns[node[0]][node[1]] = "#"
    while True:
        # check if goal was found
        if node[0] == goal[0] and node[1] == goal[1]:
            optimal_turns[node[0]][node[1]] = "*"
            break

        old_y = node[0]
        old_x = node[1]
        node = get_next(dynamic_grid, node, cost)

        turn_marker = action_name[node[3]] 

        optimal_turns[old_y][old_x] = turn_marker

    return optimal_turns

val = optimum_policy2D(grid, init, goal, cost)

for i in range(len(val)):
    print(val[i])
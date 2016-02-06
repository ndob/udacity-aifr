# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]

goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def get_minimum_cost(dynamic_grid, x, y):
    value = 999999
    for i in range(len(delta)):
        new_x = x + delta[i][1]
        new_y = y + delta[i][0]

        if new_x < 0 or new_x >= len(dynamic_grid[0]) or new_y < 0 or new_y >= len(dynamic_grid):
            continue

        if dynamic_grid[new_y][new_x] < value:
            value = dynamic_grid[new_y][new_x]

    return value

def get_optimal_turn(dynamic_grid, x, y):
    if dynamic_grid[y][x] == 99:
        return ' '

    value = 99999
    turn = ' '

    for i in range(len(delta)):
        new_x = x + delta[i][1]
        new_y = y + delta[i][0]

        if new_x < 0 or new_x >= len(dynamic_grid[0]) or new_y < 0 or new_y >= len(dynamic_grid):
            continue

        if dynamic_grid[new_y][new_x] < value:
            value = dynamic_grid[new_y][new_x]
            turn = delta_name[i]

    return turn

def compute_cell_values(grid, dynamic_grid, cost, x, y):
    for i in range(len(delta)):
        new_x = x + delta[i][1]
        new_y = y + delta[i][0]

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

        compute_cell_values(grid, dynamic_grid, cost, new_x, new_y)

def compute_value(grid, goal, cost):
    dynamic_grid = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]

    dynamic_grid[goal[0]][goal[1]] = 0
    compute_cell_values(grid, dynamic_grid, cost, goal[1], goal[0])    
    return dynamic_grid

def optimum_policy(grid, goal, cost):
    optimal_turns = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    dynamic_grid = compute_value(grid, goal, cost)

    for y in range(len(dynamic_grid)):
        for x in range(len(dynamic_grid[y])):
            if goal[0] == y and goal[1] == x:
                optimal_turns[y][x] = "*"
            else:
                optimal_turns[y][x] = get_optimal_turn(dynamic_grid, x, y)

    return optimal_turns 

val = optimum_policy(grid, goal, cost)
for i in range(len(val)):
    print(val[i])

# --------------
# USER INSTRUCTIONS
#
# Write a function called stochastic_value that 
# returns two grids. The first grid, value, should 
# contain the computed value of each cell as shown 
# in the video. The second grid, policy, should 
# contain the optimum policy for each cell.
#
# --------------
# GRADING NOTES
#
# We will be calling your stochastic_value function
# with several different grids and different values
# of success_prob, collision_cost, and cost_step.
# In order to be marked correct, your function must
# RETURN (it does not have to print) two grids,
# value and policy.
#
# When grading your value grid, we will compare the
# value of each cell with the true value according
# to this model. If your answer for each cell
# is sufficiently close to the correct answer
# (within 0.001), you will be marked as correct.

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>'] # Use these when creating your policy grid.

# ---------------------------------------------
#  Modify the function stochastic_value below
# ---------------------------------------------


def get_cost(x, y, delta_index, dynamic_grid, success_prob, adjacent_prob, collision_cost):
    new_x = x + delta[delta_index][1]
    new_y = y + delta[delta_index][0]

    if new_x < 0 or new_x >= len(dynamic_grid[0]) or new_y < 0 or new_y >= len(dynamic_grid):
        return collision_cost

    success_value = success_prob * dynamic_grid[new_y][new_x]

    adj1_index = (delta_index - 1) % 4
    adj1_x = x + delta[adj1_index][1]
    adj1_y = y + delta[adj1_index][0]
    if adj1_x < 0 or adj1_x >= len(dynamic_grid[0]) or adj1_y < 0 or adj1_y >= len(dynamic_grid):
        adj1_value = adjacent_prob * collision_cost
    else:
        adj1_value = adjacent_prob * dynamic_grid[adj1_y][adj1_x]

    adj2_index = (delta_index + 1) % 4
    adj2_x = x + delta[adj2_index][1]
    adj2_y = y + delta[adj2_index][0]
    if adj2_x < 0 or adj2_x >= len(dynamic_grid[0]) or adj2_y < 0 or adj2_y >= len(dynamic_grid):
        adj2_value = adjacent_prob * collision_cost
    else:
        adj2_value = adjacent_prob * dynamic_grid[adj2_y][adj2_x]

    return success_value + adj1_value + adj2_value

def get_minimum_cost(dynamic_grid, success_prob, cost_step, collision_cost, x, y):
    adjacent_prob = (1.0 - success_prob) * 0.5
    value = collision_cost
    for i in range(len(delta)):
        total_value = get_cost(x, y, i, dynamic_grid, success_prob, adjacent_prob, collision_cost)
        
        if total_value < value:
            value = total_value

    return min(dynamic_grid[y][x], value + cost_step)

def get_optimal_turn(dynamic_grid, collision_cost, x, y):
    if dynamic_grid[y][x] >= collision_cost:
        return ' '

    adjacent_prob = (1.0 - success_prob) * 0.5
    value = collision_cost
    for i in range(len(delta)):
        total_value = get_cost(x, y, i, dynamic_grid, success_prob, adjacent_prob, collision_cost)
        
        if total_value < value:
            value = total_value
            turn = delta_name[i]

    return turn

def compute_cell_values(grid, dynamic_grid, cost_step, collision_cost, success_prob, x, y):
    for i in range(len(delta)):
        new_x = x + delta[i][1]
        new_y = y + delta[i][0]

        if new_x < 0 or new_x >= len(grid[0]) or new_y < 0 or new_y >= len(grid):
            continue

        if grid[new_y][new_x] == 1:
            new_cost = collision_cost
        else:
            new_cost = min(get_minimum_cost(dynamic_grid, success_prob, cost_step, collision_cost, new_x, new_y), collision_cost)

        # if the cost doesn't change, no need to recount the neighbours
        if abs(new_cost - dynamic_grid[new_y][new_x]) < 0.001:
            continue
        else:
            dynamic_grid[new_y][new_x] = new_cost
        print new_x, new_y, new_cost
        compute_cell_values(grid, dynamic_grid, cost_step, collision_cost, success_prob, new_x, new_y)

def optimum_policy(grid, dynamic_grid, policy, collision_cost, goal):
    for y in range(len(dynamic_grid)):
        for x in range(len(dynamic_grid[y])):
            if goal[0] == y and goal[1] == x:
                policy[y][x] = "*"
            else:
                policy[y][x] = get_optimal_turn(dynamic_grid, collision_cost, x, y)

    return policy 

def stochastic_value(grid,goal,cost_step,collision_cost,success_prob):
    failure_prob = (1.0 - success_prob)/2.0 # Probability(stepping left) = prob(stepping right) = failure_prob
    value = [[collision_cost for col in range(len(grid[0]))] for row in range(len(grid))]
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    
    value[goal[0]][goal[1]] = 0.0
    compute_cell_values(grid, value, cost_step, collision_cost, success_prob, goal[1], goal[0])  
    optimum_policy(grid, value, policy, collision_cost, goal)

    return value, policy

# ---------------------------------------------
#  Use the code below to test your solution
# ---------------------------------------------

grid = [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 1, 1, 0]]
goal = [0, len(grid[0])-1] # Goal is in top right corner
cost_step = 1
collision_cost = 1000
success_prob = 0.5

value,policy = stochastic_value(grid,goal,cost_step,collision_cost,success_prob)
for row in value:
    print row
for row in policy:
    print row

# Expected outputs:
#
# [57.9029, 40.2784, 26.0665,  0.0000]
# [47.0547, 36.5722, 29.9937, 27.2698]
# [53.1715, 42.0228, 37.7755, 45.0916]
# [77.5858, 100.00, 100.00, 73.5458]
#
# ['>', 'v', 'v', '*']
# ['>', '>', '^', '<']
# ['>', '^', '^', '<']
# ['^', ' ', ' ', '^']
